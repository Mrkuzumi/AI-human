#pragma once

// Generic "hold this pose for N seconds" detector that runs on the board's PS.
//
// head_pat_trigger.hpp was written for exactly one gesture: a hand near the top
// of the performer's head.  Face-cupping and hugging need the same machinery
// (anchor, scale, region, hysteresis, dwell, drop-out grace) but with different
// geometry and a requirement on how many hands must be inside, so this header
// generalises that shape rather than copying it twice.
//
// Two anchors are supported:
//
//   Anchor::Nose   scale = ear-to-ear head width.  The region is an ellipse-ish
//                  box around the nose, shifted up for a head pat and down for
//                  cupping the face.
//   Anchor::Chest  scale = shoulder width, anchored at the shoulder midpoint.
//                  Used by the hug detector, where both hands meet in front of
//                  the chest.
//
// Like the head-pat detector this publishes a *state* (0/1) plus a progress
// value, never an edge, so Unity can consume it as a one-shot trigger today and
// as a sustained bool later without any protocol change.
//
// The reported ``outside`` metric is scaled by the anchor's scale: 0.0 means
// every required hand is inside the region, and a positive value is how far the
// best hand still is from it, in units of head/shoulder widths.  That makes the
// number directly comparable across gestures and readable in the bridge's
// status line while tuning.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "motion_udp_sender.hpp"

namespace gesture_region
{
// COCO-17 body keypoints.
constexpr std::size_t BODY_COUNT = 17;
constexpr std::size_t BODY_NOSE = 0;
constexpr std::size_t BODY_LEFT_EAR = 3;
constexpr std::size_t BODY_RIGHT_EAR = 4;
constexpr std::size_t BODY_LEFT_SHOULDER = 5;
constexpr std::size_t BODY_RIGHT_SHOULDER = 6;

// face51: index 13 is the nose tip.
constexpr std::size_t FACE_NOSE_TIP = 13;

// MediaPipe-order hand keypoints.
constexpr std::size_t HAND_COUNT = 21;
constexpr std::size_t HAND_WRIST = 0;
constexpr std::size_t HAND_PALM = 9;

enum class Anchor
{
    Nose,
    Chest,
};

struct Settings
{
    // Wire contract: the id the PC maps back to a gesture name.
    uint16_t id = 0;
    /// Allows a config file to switch one region off without removing it.
    bool enabled = true;

    Anchor anchor = Anchor::Nose;
    /// Allow the face model's nose tip to stand in for the pose nose.
    bool use_face_nose = true;

    // Region, in units of the anchor scale (head width / shoulder width).
    // The band runs from ``region_inner_half_width`` out to ``region_half_width``
    // on each side, so a positive inner value turns the box into a ring: the
    // hands must be AWAY from the anchor, which is how an open-armed hug is told
    // apart from clasped or crossed hands near the chest.
    double region_above = 0.1;
    double region_below = 0.9;
    double region_half_width = 1.0;
    double region_inner_half_width = 0.0;

    // How many hands must be inside, and whether they must straddle the anchor.
    int min_hands = 1;
    bool require_both_sides = false;
    /// Also require the wrist, not just the palm (stricter, rarely wanted).
    bool whole_hand = false;

    // Timing, in seconds.
    double dwell_seconds = 0.4;
    double grace_seconds = 0.35;
    double release_seconds = 0.3;
    double cooldown_seconds = 0.5;

    double min_confidence = 0.3;
    /// A cupping hand occludes the face, so the anchor is cached briefly.
    double anchor_cache_seconds = 0.5;
    double scale_cache_seconds = 3.0;
    double scale_smoothing = 0.35;
    /// Scale fallback as a fraction of frame height when nothing was measured.
    double scale_fallback = 0.20;
    /// Leaving needs to be this much further out than entering (hysteresis).
    double exit_scale = 1.35;
};

struct State
{
    bool active = false;
    double progress = 0.0;
    double dwell = 0.0;
    double outside = 0.0;
    bool has_outside = false;
    double scale_px = 0.0;
    double anchor_x = 0.0;
    double anchor_y = 0.0;
    int hands_inside = 0;
    bool sides_ok = false;
    bool anchor_cached = false;
    const char *anchor_source = "none";
};

namespace detail
{
struct AnchorPoint
{
    std::optional<std::array<double, 2>> point;
    const char *source = "none";
    bool cached = false;
};

struct HandMetric
{
    double enter = 0.0;
    double exit = 0.0;
    double palm_x = 0.0;
    int index = -1;
};
} // namespace detail

class Detector
{
public:
    Detector() = default;
    explicit Detector(Settings settings) : settings_(settings) {}

    const Settings &settings() const { return settings_; }

    /// Advance one frame and return the current gesture state.
    State update(double now,
                 float frame_width,
                 float frame_height,
                 const motion_udp::Detection *body,
                 const motion_udp::Detection *face,
                 const std::vector<motion_udp::Detection> &hands)
    {
        const double width = frame_width > 0.0f ? static_cast<double>(frame_width) : 1920.0;
        const double height = frame_height > 0.0f ? static_cast<double>(frame_height) : 1080.0;

        const double scale = anchorScale(body, width, height, now);
        const detail::AnchorPoint point = resolveAnchor(body, face, now);
        if (!point.point)
            return miss(scale, now, 0.0, 0.0, "none", false);

        const double anchor_x = (*point.point)[0] * width;
        const double anchor_y = (*point.point)[1] * height;

        const std::vector<detail::HandMetric> metrics =
            handMetrics(hands, anchor_x, anchor_y, scale, width, height);

        // While the gesture is already high the more permissive exit scale
        // applies, so a hand hovering on the border cannot chatter.
        int inside = 0;
        bool left = false;
        bool right = false;
        double best_metric = 0.0;
        bool have_metric = false;
        for (const auto &metric : metrics)
        {
            const double value = active_ ? metric.exit : metric.enter;
            if (!have_metric || value < best_metric)
            {
                best_metric = value;
                have_metric = true;
            }
            if (value <= 0.0)
            {
                ++inside;
                if (metric.palm_x < anchor_x)
                    left = true;
                else
                    right = true;
            }
        }

        const bool sides_ok = !settings_.require_both_sides || (left && right);
        const bool in_zone = metrics.empty()
                                 ? false
                                 : (inside >= settings_.min_hands && sides_ok);
        const double outside_metric = in_zone ? 0.0 : (have_metric ? best_metric : 0.0);

        if (metrics.empty())
        {
            // No usable hand at all: grace still keeps the dwell alive.
            return miss(scale, now, anchor_x, anchor_y, point.source, point.cached);
        }

        advance(now, in_zone);

        State state;
        state.active = active_;
        state.progress = progress();
        state.dwell = dwell_;
        state.outside = outside_metric;
        state.has_outside = true;
        state.scale_px = scale;
        state.anchor_x = anchor_x;
        state.anchor_y = anchor_y;
        state.hands_inside = inside;
        state.sides_ok = sides_ok;
        state.anchor_cached = point.cached;
        state.anchor_source = point.source;
        return state;
    }

    bool active() const { return active_; }
    double dwell() const { return dwell_; }

private:
    /// Ear-to-ear span, or shoulder span for the chest anchor, in pixels.
    double anchorScale(const motion_udp::Detection *body, double width, double height, double now)
    {
        const bool chest = settings_.anchor == Anchor::Chest;
        if (body != nullptr && body->keypoints.size() >= BODY_COUNT)
        {
            const std::size_t first = chest ? BODY_LEFT_SHOULDER : BODY_LEFT_EAR;
            const std::size_t second = chest ? BODY_RIGHT_SHOULDER : BODY_RIGHT_EAR;
            const auto &a = body->keypoints[first];
            const auto &b = body->keypoints[second];
            if (a[2] >= settings_.min_confidence && b[2] >= settings_.min_confidence)
            {
                const double dx = static_cast<double>(a[0] - b[0]) * width;
                const double dy = static_cast<double>(a[1] - b[1]) * height;
                const double span = std::hypot(dx, dy);
                if (span >= 0.05 * height && span <= 0.75 * height)
                {
                    if (!scale_cache_)
                        scale_cache_ = span;
                    else
                        *scale_cache_ += settings_.scale_smoothing * (span - *scale_cache_);
                    scale_stamp_ = now;
                    return *scale_cache_;
                }
            }
        }

        if (scale_cache_ && (now - scale_stamp_) <= settings_.scale_cache_seconds)
            return *scale_cache_;

        const double fallback = chest ? 0.30 : settings_.scale_fallback;
        return fallback * height;
    }

    /// Anchor in normalised image coordinates, with a short-lived cache.
    detail::AnchorPoint resolveAnchor(const motion_udp::Detection *body,
                                      const motion_udp::Detection *face,
                                      double now)
    {
        if (settings_.anchor == Anchor::Chest)
        {
            if (body != nullptr && body->keypoints.size() >= BODY_COUNT)
            {
                const auto &left = body->keypoints[BODY_LEFT_SHOULDER];
                const auto &right = body->keypoints[BODY_RIGHT_SHOULDER];
                if (left[2] >= settings_.min_confidence && right[2] >= settings_.min_confidence)
                {
                    anchor_cache_ = std::array<double, 2>{
                        (static_cast<double>(left[0]) + right[0]) * 0.5,
                        (static_cast<double>(left[1]) + right[1]) * 0.5};
                    anchor_source_ = "shoulders";
                    anchor_stamp_ = now;
                    return detail::AnchorPoint{anchor_cache_, anchor_source_, false};
                }
            }
            if (anchor_cache_ && (now - anchor_stamp_) <= settings_.anchor_cache_seconds)
                return detail::AnchorPoint{anchor_cache_, anchor_source_, true};
            return detail::AnchorPoint{std::nullopt, "none", false};
        }

        const std::array<float, 3> *best = nullptr;
        const char *source = "none";
        if (body != nullptr && body->keypoints.size() > BODY_NOSE)
        {
            const auto &point = body->keypoints[BODY_NOSE];
            if (point[2] >= settings_.min_confidence)
            {
                best = &point;
                source = "body";
            }
        }
        if (settings_.use_face_nose && face != nullptr && face->keypoints.size() > FACE_NOSE_TIP)
        {
            const auto &point = face->keypoints[FACE_NOSE_TIP];
            if (point[2] >= settings_.min_confidence && (best == nullptr || point[2] > (*best)[2]))
            {
                best = &point;
                source = "face";
            }
        }

        if (best != nullptr)
        {
            anchor_cache_ = std::array<double, 2>{(*best)[0], (*best)[1]};
            anchor_source_ = source;
            anchor_stamp_ = now;
            return detail::AnchorPoint{anchor_cache_, source, false};
        }
        if (anchor_cache_ && (now - anchor_stamp_) <= settings_.anchor_cache_seconds)
            return detail::AnchorPoint{anchor_cache_, anchor_source_, true};
        return detail::AnchorPoint{std::nullopt, "none", false};
    }

    /// Scale-free distance outside the region; 0.0 means inside.
    ///
    /// With ``region_inner_half_width`` at 0 this is a plain box.  With a positive
    /// inner value only the ring between the two half widths counts as inside, so
    /// a hand resting on the centre line is treated as outside.
    double outside(double px,
                   double py,
                   double anchor_x,
                   double anchor_y,
                   double scale,
                   double multiplier) const
    {
        const double outer = settings_.region_half_width * scale * multiplier;
        const double inner = settings_.region_inner_half_width * scale * multiplier;
        const double top = anchor_y - settings_.region_above * scale * multiplier;
        const double bottom = anchor_y + settings_.region_below * scale * multiplier;

        const double distance_x = std::abs(px - anchor_x);
        double dx = 0.0;
        if (distance_x > outer)
            dx = distance_x - outer;
        else if (distance_x < inner)
            dx = inner - distance_x;
        const double dy = std::max(0.0, std::max(top - py, py - bottom));
        return std::hypot(dx, dy) / scale;
    }

    /// Per-hand enter/exit metrics, best (smallest) first.
    std::vector<detail::HandMetric> handMetrics(const std::vector<motion_udp::Detection> &hands,
                                                double anchor_x,
                                                double anchor_y,
                                                double scale,
                                                double width,
                                                double height) const
    {
        std::vector<detail::HandMetric> out;
        for (std::size_t index = 0; index < hands.size(); ++index)
        {
            const auto &candidate = hands[index];
            if (candidate.keypoints.size() < HAND_COUNT || candidate.score < settings_.min_confidence)
                continue;
            const auto &wrist = candidate.keypoints[HAND_WRIST];
            const auto &palm = candidate.keypoints[HAND_PALM];
            if (wrist[2] < settings_.min_confidence || palm[2] < settings_.min_confidence)
                continue;

            double enter = 0.0;
            double exit_metric = 0.0;
            bool first = true;
            auto visit = [&](const std::array<float, 3> &point) {
                const double px = static_cast<double>(point[0]) * width;
                const double py = static_cast<double>(point[1]) * height;
                const double enter_metric = outside(px, py, anchor_x, anchor_y, scale, 1.0);
                const double leave_metric =
                    outside(px, py, anchor_x, anchor_y, scale, settings_.exit_scale);
                enter = first ? enter_metric : std::max(enter, enter_metric);
                exit_metric = first ? leave_metric : std::min(exit_metric, leave_metric);
                first = false;
            };
            visit(palm);
            if (settings_.whole_hand)
                visit(wrist);
            if (first)
                continue;

            detail::HandMetric metric;
            metric.enter = enter;
            metric.exit = exit_metric;
            metric.palm_x = static_cast<double>(palm[0]) * width;
            metric.index = static_cast<int>(index);
            out.push_back(metric);
        }

        std::sort(out.begin(), out.end(), [](const detail::HandMetric &a, const detail::HandMetric &b) {
            return a.enter < b.enter;
        });
        return out;
    }

    double progress() const
    {
        if (settings_.dwell_seconds <= 0.0)
            return 0.0;
        return std::min(1.0, dwell_ / settings_.dwell_seconds);
    }

    /// No usable hand or anchor this frame; grace still keeps the dwell alive.
    State miss(double scale,
               double now,
               double anchor_x,
               double anchor_y,
               const char *source,
               bool cached)
    {
        advance(now, false);

        State state;
        state.active = active_;
        state.progress = progress();
        state.dwell = dwell_;
        state.has_outside = false;
        state.scale_px = scale;
        state.anchor_x = anchor_x;
        state.anchor_y = anchor_y;
        state.anchor_cached = cached;
        state.anchor_source = source;
        return state;
    }

    /// Wall-clock dwell, so one late frame cannot wipe out the progress.
    void advance(double now, bool in_zone)
    {
        if (active_)
        {
            if (in_zone)
            {
                last_inside_ = now;
            }
            else if (last_inside_ && (now - *last_inside_) >= settings_.release_seconds)
            {
                active_ = false;
                armed_at_ = now;
                start_.reset();
                last_inside_.reset();
                dwell_ = 0.0;
            }
            return;
        }

        if (in_zone)
        {
            if (!start_)
                start_ = now;
            last_inside_ = now;
            dwell_ = now - *start_;
            if (dwell_ >= settings_.dwell_seconds && (now - armed_at_) >= settings_.cooldown_seconds)
                active_ = true;
            return;
        }

        if (start_ && last_inside_ && (now - *last_inside_) > settings_.grace_seconds)
        {
            start_.reset();
            last_inside_.reset();
            dwell_ = 0.0;
        }
    }

    Settings settings_{};
    bool active_ = false;
    double dwell_ = 0.0;
    std::optional<double> start_;
    std::optional<double> last_inside_;
    double armed_at_ = 0.0;
    // Cached anchor/scale; the trailing underscore keeps them from shadowing
    // the same-named methods.
    std::optional<std::array<double, 2>> anchor_cache_;
    const char *anchor_source_ = "none";
    double anchor_stamp_ = 0.0;
    std::optional<double> scale_cache_;
    double scale_stamp_ = 0.0;
};
} // namespace gesture_region
