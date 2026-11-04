#pragma once

// Cross-frame "pat the head" detector that runs on the board's PS side.
//
// This is a C++ port of pc/reference/gesture_trigger.py.  The thresholds, the
// region geometry and the hysteresis state machine are kept identical on
// purpose: the PC detector stays available (--gesture-source pc|board|both) so
// the two implementations can be diffed frame by frame while bringing the
// board version up.
//
// A real hand near the performer's own head stands in for "patting the digital
// human's head".  The nose is the most reliably detected head landmark, so it
// is the anchor and the test region is shifted upward, toward the crown.
//
// The detector publishes a *state* (0/1) rather than an edge event, so Unity can
// consume it as a one-shot trigger today and as a sustained bool later.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

#include "motion_udp_sender.hpp"

namespace head_pat
{
// COCO-17 body keypoints.
constexpr std::size_t BODY_COUNT = 17;
constexpr std::size_t BODY_NOSE = 0;
constexpr std::size_t BODY_LEFT_EAR = 3;
constexpr std::size_t BODY_RIGHT_EAR = 4;

// 51-point inner-face layout: nose tip is 68-point index 30 -> v1 index 13.
constexpr std::size_t FACE_NOSE_TIP = 13;

// Hand landmarks follow the MediaPipe order used by the finger model.
constexpr std::size_t HAND_COUNT = 21;
constexpr std::size_t HAND_WRIST = 0;
constexpr std::size_t HAND_PALM = 9;

struct Settings
{
    double dwell_seconds = 1.0;
    double grace_seconds = 0.25;
    double release_seconds = 0.3;
    double cooldown_seconds = 0.5;
    double min_confidence = 0.3;
    double nose_cache_seconds = 0.5;
    double region_half_width = 0.8;
    double region_above = 1.3;
    double region_below = 0.2;
    double exit_scale = 1.35;
    double head_width_fallback = 0.20;
    double head_width_cache_seconds = 3.0;
    double head_width_smoothing = 0.35;
    bool whole_hand = true;
};

struct State
{
    bool active = false;
    double progress = 0.0;
    double dwell = 0.0;
    double outside = 0.0;
    bool has_outside = false;
    double head_width_px = 0.0;
    double nose_x = 0.0;
    double nose_y = 0.0;
    const char *nose_source = "none";
    bool nose_cached = false;
    int hand_index = -1;
};

namespace detail
{
struct Nose
{
    std::optional<std::array<double, 2>> point;
    const char *source = "none";
    bool cached = false;
};

struct HandMetric
{
    std::optional<double> enter;
    double exit = 0.0;
    int index = -1;
};
} // namespace detail

class Trigger
{
public:
    Trigger() = default;
    explicit Trigger(const Settings &settings) : settings_(settings) {}

    const Settings &settings() const { return settings_; }

    /// Ear-to-ear span in pixels, cached while the ears are occluded.
    ///
    /// A fixed fallback is a poor substitute: the measured span is roughly a
    /// tenth of the frame height, so the 0.20 fallback would balloon the region
    /// by about 60% exactly while the hand is covering the face.  Reusing the
    /// last good measurement keeps the region tied to the real head size.
    double headWidth(const motion_udp::Detection *body, double width, double height, double now)
    {
        if (body != nullptr && body->keypoints.size() >= BODY_COUNT)
        {
            const auto &left = body->keypoints[BODY_LEFT_EAR];
            const auto &right = body->keypoints[BODY_RIGHT_EAR];
            if (left[2] >= settings_.min_confidence && right[2] >= settings_.min_confidence)
            {
                const double dx = static_cast<double>(left[0] - right[0]) * width;
                const double dy = static_cast<double>(left[1] - right[1]) * height;
                const double span = std::hypot(dx, dy);
                if (span >= 0.05 * height && span <= 0.60 * height)
                {
                    // Light EMA: raw spans jitter by tens of pixels and the
                    // region size should not follow that noise.
                    if (!head_cache_)
                        head_cache_ = span;
                    else
                        *head_cache_ += settings_.head_width_smoothing * (span - *head_cache_);
                    head_stamp_ = now;
                    return *head_cache_;
                }
            }
        }

        if (head_cache_ && (now - head_stamp_) <= settings_.head_width_cache_seconds)
            return *head_cache_;
        return settings_.head_width_fallback * height;
    }

    /// Prefer the pose nose, fall back to the face nose tip, then to cache.
    ///
    /// A patting hand occludes the face, so a short-lived cache keeps the anchor
    /// stable exactly while the gesture is being performed.
    detail::Nose resolveNose(const motion_udp::Detection *body,
                             const motion_udp::Detection *face,
                             double now)
    {
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
        if (face != nullptr && face->keypoints.size() > FACE_NOSE_TIP)
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
            nose_cache_ = std::array<double, 2>{(*best)[0], (*best)[1]};
            nose_source_ = source;
            nose_stamp_ = now;
            return detail::Nose{nose_cache_, source, false};
        }
        if (nose_cache_ && (now - nose_stamp_) <= settings_.nose_cache_seconds)
            return detail::Nose{nose_cache_, nose_source_, true};
        return detail::Nose{std::nullopt, "none", false};
    }

    /// Scale-free distance outside the head region; 0.0 means inside.
    double outside(double px, double py, double nose_x, double nose_y, double head_width, double scale) const
    {
        const double half_width = settings_.region_half_width * head_width * scale;
        const double top = nose_y - settings_.region_above * head_width * scale;
        const double bottom = nose_y + settings_.region_below * head_width * scale;
        const double dx = std::max(0.0, std::abs(px - nose_x) - half_width);
        const double dy = std::max(0.0, std::max(top - py, py - bottom));
        return std::hypot(dx, dy) / head_width;
    }

    /// Closest hand by the enter metric, plus its hysteresis (exit) metric.
    detail::HandMetric hand(const std::vector<motion_udp::Detection> &hands,
                            double nose_x,
                            double nose_y,
                            double head_width,
                            double width,
                            double height) const
    {
        detail::HandMetric best;
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
                const double enter_metric = outside(px, py, nose_x, nose_y, head_width, 1.0);
                const double leave_metric =
                    outside(px, py, nose_x, nose_y, head_width, settings_.exit_scale);
                enter = first ? enter_metric : std::max(enter, enter_metric);
                exit_metric = first ? leave_metric : std::min(exit_metric, leave_metric);
                first = false;
            };
            visit(palm);
            if (settings_.whole_hand)
                visit(wrist);
            if (first)
                continue;

            if (!best.enter || enter < *best.enter)
            {
                best.enter = enter;
                best.exit = exit_metric;
                best.index = static_cast<int>(index);
            }
        }
        return best;
    }

    double progress() const
    {
        if (settings_.dwell_seconds <= 0.0)
            return 0.0;
        return std::min(1.0, dwell_ / settings_.dwell_seconds);
    }

    /// Advance the detector by one frame and return the current signal bit.
    State update(double now,
                 float frame_width,
                 float frame_height,
                 const motion_udp::Detection *body,
                 const motion_udp::Detection *face,
                 const std::vector<motion_udp::Detection> &hands)
    {
        const double width = frame_width > 0.0f ? static_cast<double>(frame_width) : 1920.0;
        const double height = frame_height > 0.0f ? static_cast<double>(frame_height) : 1080.0;
        const double head_width = headWidth(body, width, height, now);
        const detail::Nose nose = resolveNose(body, face, now);
        if (!nose.point)
            return miss(head_width, now, 0.0, 0.0, "none", false);

        const double nose_x = (*nose.point)[0] * width;
        const double nose_y = (*nose.point)[1] * height;
        const detail::HandMetric metrics = hand(hands, nose_x, nose_y, head_width, width, height);
        if (!metrics.enter)
            return miss(head_width, now, nose_x, nose_y, nose.source, nose.cached);

        const double outside_metric = active_ ? metrics.exit : *metrics.enter;
        advance(now, outside_metric <= 0.0);

        State state;
        state.active = active_;
        state.progress = progress();
        state.dwell = dwell_;
        state.outside = outside_metric;
        state.has_outside = true;
        state.head_width_px = head_width;
        state.nose_x = nose_x;
        state.nose_y = nose_y;
        state.nose_source = nose.source;
        state.nose_cached = nose.cached;
        state.hand_index = metrics.index;
        return state;
    }

    /// No usable hand or nose this frame; grace still keeps the dwell alive.
    State miss(double head_width,
               double now,
               double nose_x,
               double nose_y,
               const char *source,
               bool cached)
    {
        advance(now, false);

        State state;
        state.active = active_;
        state.progress = progress();
        state.dwell = dwell_;
        state.has_outside = false;
        state.head_width_px = head_width;
        state.nose_x = nose_x;
        state.nose_y = nose_y;
        state.nose_source = source;
        state.nose_cached = cached;
        return state;
    }

    /// Wall-clock dwell, so a single late frame cannot wipe out progress.
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

        // Outside the region, or no usable hand: only a gap longer than the
        // grace period abandons the gesture in progress.
        if (start_ && last_inside_ && (now - *last_inside_) > settings_.grace_seconds)
        {
            start_.reset();
            last_inside_.reset();
            dwell_ = 0.0;
        }
    }

    bool active() const { return active_; }
    double dwell() const { return dwell_; }

private:
    Settings settings_{};
    bool active_ = false;
    double dwell_ = 0.0;
    std::optional<double> start_;
    std::optional<double> last_inside_;
    double armed_at_ = 0.0;
    // Cached anchors; the trailing underscore keeps them from shadowing the
    // same-named accessors (resolveNose, headWidth).
    std::optional<std::array<double, 2>> nose_cache_;
    const char *nose_source_ = "none";
    double nose_stamp_ = 0.0;
    std::optional<double> head_cache_;
    double head_stamp_ = 0.0;
};
} // namespace head_pat
