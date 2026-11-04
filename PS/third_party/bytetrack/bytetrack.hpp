/*
   ByteTrack C++ — self-contained multi-object tracker.

   Based on "ByteTrack: Multi-Object Tracking by Associating Every Detection Box"
   (Zhang, Sun, Jiang et al., ECCV 2022).

   Key algorithm:
     - Kalman filter (8-D: cx,cy,w,h,vx,vy,vw,vh) with velocity update
     - Two-stage association: high dets ↔ all tracks, then low dets ↔ remaining
     - Track state machine: New → Tracked → Lost → Removed
     - IoU matching sorted by score descending (greedy linear-assignment approx)
     - Returns detection→track assignments to avoid external re-matching

   License: MIT (see LICENSE file)
*/

#pragma once

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace bytetrack {

// ============================================================
// Detection
// ============================================================
struct Detection {
    cv::Rect2f box;
    float score;
    int class_id;

    Detection() : box(), score(0.0f), class_id(-1) {}
    Detection(cv::Rect2f b, float s, int c) : box(b), score(s), class_id(c) {}
};

// ============================================================
// KalmanFilter — 8-D: [cx, cy, w, h, vx, vy, vw, vh]
// ============================================================
class KalmanFilter {
public:
    KalmanFilter();
    void initiate(const cv::Rect2f &box);
    cv::Rect2f predict();
    cv::Rect2f update(const cv::Rect2f &box);
    cv::Rect2f getState() const;
    const std::vector<float> &mean() const { return mean_; }

private:
    static constexpr int kStateDim = 8;
    std::vector<float> mean_;
    std::vector<float> covariance_;
};

// ============================================================
// STrack
// ============================================================
enum class TrackState : uint8_t { New = 0, Tracked = 1, Lost = 2, Removed = 3 };

class STrack {
public:
    STrack(const Detection &det, int track_id, int frame_id);

    int trackId() const { return track_id_; }
    TrackState state() const { return state_; }
    bool isActivated() const { return state_ == TrackState::Tracked; }
    bool isLost() const { return state_ == TrackState::Lost; }
    bool isRemoved() const { return state_ == TrackState::Removed; }

    const cv::Rect2f &lastBox() const { return last_box_; }
    const cv::Rect2f &predictedBox() const { return predicted_box_; }
    float score() const { return score_; }

    int startFrame() const { return start_frame_; }
    int lastUpdateFrame() const { return last_update_frame_; }
    int lostCount() const { return lost_count_; }
    int trackletLength() const { return last_update_frame_ - start_frame_ + 1; }

    void activate(int frame_id);
    void reActivate(const Detection &det, int frame_id);
    void predict();
    void update(const Detection &det, int frame_id);
    void markLost();
    void markRemoved();

private:
    int track_id_;
    TrackState state_;
    int start_frame_;
    int last_update_frame_;
    int lost_count_;

    cv::Rect2f last_box_;
    cv::Rect2f predicted_box_;
    float score_;

    KalmanFilter kalman_;
};

// ============================================================
// TrackingConfig
// ============================================================
struct TrackingConfig {
    float track_high_thresh = 0.35f;
    float track_low_thresh  = 0.10f;
    float new_track_thresh  = 0.45f;
    float match_thresh      = 0.80f;   // max IoU distance (1 - IoU); 0.80 → IoU≥0.20
    int   track_buffer      = 30;
    float frame_rate        = 22.0f;
    int   display_prediction_frames = 3;
    bool  small_object_match = true;
    float small_object_max_side = 120.0f;
    float small_center_distance_ratio = 0.80f;
    float small_area_ratio_min = 0.40f;
    float small_area_ratio_max = 2.50f;
    float small_second_match_iou = 0.15f;
    bool  debug_log         = false;
};

// ============================================================
// Detection→track assignment (returned by update())
// ============================================================
struct TrackAssignment {
    int track_id;
    int detection_idx;  // index into the detections vector passed to update()
    int class_id;       // detection's class_id
    float score;        // detection's score
};

// Immutable snapshot for display-only prediction. These tracks must not be
// fed into class voting or actuator decisions.
struct TrackSnapshot {
    int track_id;
    cv::Rect2f box;
    float score;
    int lost_count;
};

// ============================================================
// BYTETracker
// ============================================================
class BYTETracker {
public:
    explicit BYTETracker(const TrackingConfig &cfg = TrackingConfig());

    // Core update. Returns detection→track assignments.
    // active_tracks: output — currently active (Tracked) tracks.
    // assignments:    output — which detection matched which track.
    void update(std::vector<Detection> &detections, int frame_id,
                std::vector<std::shared_ptr<STrack>> &active_tracks,
                std::vector<TrackAssignment> &assignments,
                int prediction_steps = 1);

    std::vector<TrackSnapshot> recentLostSnapshots(int max_lost_count) const;

    int totalTracksCreated()   const { return total_tracks_created_; }
    int totalTracksConfirmed() const { return total_tracks_confirmed_; }
    int totalTracksLost()      const { return total_tracks_lost_; }
    int totalTracksRemoved()   const { return total_tracks_removed_; }

    static float computeIoU(const cv::Rect2f &a, const cv::Rect2f &b);

private:
    TrackingConfig cfg_;

    std::vector<std::shared_ptr<STrack>> tracked_stracks_;
    std::vector<std::shared_ptr<STrack>> lost_stracks_;
    std::vector<std::shared_ptr<STrack>> removed_stracks_;

    int track_id_counter_ = 0;
    int frame_id_ = 0;

    int total_tracks_created_   = 0;
    int total_tracks_confirmed_ = 0;
    int total_tracks_lost_      = 0;
    int total_tracks_removed_   = 0;

    // Sorted IoU matching (optional track_ids for deterministic tie-break)
    static std::vector<std::pair<int, int>>
    iouMatch(const std::vector<cv::Rect2f> &det_boxes,
             const std::vector<cv::Rect2f> &track_boxes,
             float match_thresh,
             const std::vector<int> *track_ids = nullptr);

    static std::vector<std::pair<int, int>>
    adaptiveMatch(const std::vector<cv::Rect2f> &det_boxes,
                  const std::vector<cv::Rect2f> &track_boxes,
                  float match_thresh,
                  const TrackingConfig &cfg,
                  bool second_stage,
                  const std::vector<int> *track_ids = nullptr);

    static std::vector<cv::Rect2f>
    detectionsToRects(const std::vector<Detection> &dets);

    static std::vector<cv::Rect2f>
    tracksToPredictedRects(const std::vector<std::shared_ptr<STrack>> &tracks);
};

// ============================================================
// KalmanFilter implementation
// ============================================================
inline KalmanFilter::KalmanFilter() {
    mean_.resize(kStateDim, 0.0f);
    covariance_.resize(kStateDim * kStateDim, 0.0f);
    for (int i = 0; i < kStateDim; i++)
        covariance_[i * kStateDim + i] = 10.0f;
    for (int i = 4; i < kStateDim; i++)
        covariance_[i * kStateDim + i] = 1000.0f;
}

inline void KalmanFilter::initiate(const cv::Rect2f &box) {
    mean_[0] = box.x + box.width * 0.5f;
    mean_[1] = box.y + box.height * 0.5f;
    mean_[2] = (box.width  > 0) ? std::log(box.width)  : 0.0f;
    mean_[3] = (box.height > 0) ? std::log(box.height) : 0.0f;
    mean_[4] = 0.0f; mean_[5] = 0.0f; mean_[6] = 0.0f; mean_[7] = 0.0f;
}

inline cv::Rect2f KalmanFilter::predict() {
    // Extrapolate center motion only. Scale velocity from noisy detector boxes
    // is unstable and can make a short-lost box grow exponentially.
    mean_[0] += mean_[4];
    mean_[1] += mean_[5];
    // Add process noise to diagonal
    float q[8] = {1.0f, 1.0f, 1.0f, 1.0f, 0.01f, 0.01f, 0.0001f, 0.0001f};
    for (int i = 0; i < kStateDim; i++)
        covariance_[i * kStateDim + i] += q[i];
    return getState();
}

inline cv::Rect2f KalmanFilter::update(const cv::Rect2f &box) {
    // Innovation y = z - H*x  (position only)
    float z_cx = box.x + box.width * 0.5f;
    float z_cy = box.y + box.height * 0.5f;
    float z_w  = (box.width  > 0) ? std::log(box.width)  : mean_[2];
    float z_h  = (box.height > 0) ? std::log(box.height) : mean_[3];

    float y_cx = z_cx - mean_[0];
    float y_cy = z_cy - mean_[1];
    float y_w  = z_w  - mean_[2];
    float y_h  = z_h  - mean_[3];

    float r[4] = {1.0f, 1.0f, 1.0f, 1.0f};  // measurement noise

    // Update position + velocity: mean[i] += K * y, vel += K_v * y
    for (int i = 0; i < 4; i++) {
        float P_ii = covariance_[i * kStateDim + i];
        float K = P_ii / (P_ii + r[i]);
        float innov = (i == 0) ? y_cx : ((i == 1) ? y_cy : ((i == 2) ? y_w : y_h));
        mean_[i] += K * innov;
        // Feed the full filtered innovation into velocity. A small fractional
        // gain lags fast motion and swaps IDs when two targets cross.
        mean_[i + 4] += K * innov;
        covariance_[i * kStateDim + i] = (1.0f - K) * P_ii;
    }

    return getState();
}

inline cv::Rect2f KalmanFilter::getState() const {
    float cx = mean_[0], cy = mean_[1];
    float w  = std::exp(mean_[2]), h = std::exp(mean_[3]);
    return cv::Rect2f(cx - w * 0.5f, cy - h * 0.5f, w, h);
}

// ============================================================
// STrack implementation
// ============================================================
inline STrack::STrack(const Detection &det, int track_id, int frame_id)
    : track_id_(track_id), state_(TrackState::New),
      start_frame_(frame_id), last_update_frame_(frame_id), lost_count_(0),
      last_box_(det.box), predicted_box_(det.box), score_(det.score)
{
    kalman_.initiate(det.box);
}

inline void STrack::activate(int frame_id) {
    state_ = TrackState::Tracked;
    last_update_frame_ = frame_id;
}

inline void STrack::reActivate(const Detection &det, int frame_id) {
    kalman_.update(det.box);
    last_box_ = det.box;
    predicted_box_ = det.box;
    score_ = det.score;
    state_ = TrackState::Tracked;
    last_update_frame_ = frame_id;
    lost_count_ = 0;
}

inline void STrack::predict() {
    predicted_box_ = kalman_.predict();
    lost_count_++;
    if (state_ == TrackState::New && lost_count_ > 2)
        state_ = TrackState::Removed;
}

inline void STrack::update(const Detection &det, int frame_id) {
    kalman_.update(det.box);
    last_box_ = det.box;
    predicted_box_ = det.box;
    score_ = det.score;
    last_update_frame_ = frame_id;
    lost_count_ = 0;
}

inline void STrack::markLost() {
    if (state_ != TrackState::Lost) state_ = TrackState::Lost;
}

inline void STrack::markRemoved() { state_ = TrackState::Removed; }

// ============================================================
// BYTETracker implementation
// ============================================================
inline BYTETracker::BYTETracker(const TrackingConfig &cfg) : cfg_(cfg) {}

inline float BYTETracker::computeIoU(const cv::Rect2f &a, const cv::Rect2f &b) {
    float x1 = std::max(a.x, b.x);
    float y1 = std::max(a.y, b.y);
    float x2 = std::min(a.x + a.width,  b.x + b.width);
    float y2 = std::min(a.y + a.height, b.y + b.height);
    float iw = x2 - x1, ih = y2 - y1;
    if (iw <= 0.0f || ih <= 0.0f) return 0.0f;
    float inter = iw * ih;
    float a_area = std::max(0.0f, a.width)  * std::max(0.0f, a.height);
    float b_area = std::max(0.0f, b.width)  * std::max(0.0f, b.height);
    float u = a_area + b_area - inter;
    return (u > 0.0f) ? (inter / u) : 0.0f;
}

inline std::vector<cv::Rect2f>
BYTETracker::detectionsToRects(const std::vector<Detection> &dets) {
    std::vector<cv::Rect2f> r; r.reserve(dets.size());
    for (auto &d : dets) r.push_back(d.box);
    return r;
}

inline std::vector<cv::Rect2f>
BYTETracker::tracksToPredictedRects(
    const std::vector<std::shared_ptr<STrack>> &tracks) {
    std::vector<cv::Rect2f> r; r.reserve(tracks.size());
    for (auto &t : tracks) r.push_back(t->predictedBox());
    return r;
}

inline std::vector<std::pair<int, int>>
BYTETracker::iouMatch(const std::vector<cv::Rect2f> &det_boxes,
                       const std::vector<cv::Rect2f> &track_boxes,
                       float max_cost,
                       const std::vector<int> *track_ids) {
    // max_cost is IoU distance: 1 - IoU.  e.g. 0.80 means IoU >= 0.20
    struct Pair { int d, t; int tid; float iou; };
    std::vector<Pair> pairs;
    for (size_t d = 0; d < det_boxes.size(); d++) {
        for (size_t t = 0; t < track_boxes.size(); t++) {
            float iou = computeIoU(det_boxes[d], track_boxes[t]);
            float cost = 1.0f - iou;
            if (cost <= max_cost)
                pairs.push_back({static_cast<int>(d), static_cast<int>(t),
                                 track_ids ? (*track_ids)[t] : static_cast<int>(t),
                                 iou});
        }
    }
    // Deterministic sort: IoU desc, then track_id asc, then detection_idx asc
    std::sort(pairs.begin(), pairs.end(),
              [](const Pair &a, const Pair &b) {
                  if (std::abs(a.iou - b.iou) > 1e-6f) return a.iou > b.iou;
                  if (a.tid != b.tid) return a.tid < b.tid;
                  return a.d < b.d;
              });

    std::vector<bool> det_used(det_boxes.size(), false);
    std::vector<bool> trk_used(track_boxes.size(), false);
    std::vector<std::pair<int, int>> matches;

    for (auto &p : pairs) {
        if (!det_used[p.d] && !trk_used[p.t]) {
            matches.emplace_back(p.d, p.t);
            det_used[p.d] = true;
            trk_used[p.t] = true;
        }
    }
    return matches;
}

inline std::vector<std::pair<int, int>>
BYTETracker::adaptiveMatch(const std::vector<cv::Rect2f> &det_boxes,
                           const std::vector<cv::Rect2f> &track_boxes,
                           float max_cost,
                           const TrackingConfig &cfg,
                           bool second_stage,
                           const std::vector<int> *track_ids) {
    struct Pair { int d, t, tid; float quality, iou; };
    std::vector<Pair> pairs;

    for (size_t d = 0; d < det_boxes.size(); ++d) {
        const auto &det = det_boxes[d];
        const float det_area = std::max(0.0f, det.width)
                             * std::max(0.0f, det.height);
        const float det_side = std::max(det.width, det.height);
        const float det_diag = std::hypot(det.width, det.height);
        if (det_area <= 1.0f) continue;

        for (size_t t = 0; t < track_boxes.size(); ++t) {
            const auto &trk = track_boxes[t];
            const float trk_area = std::max(0.0f, trk.width)
                                 * std::max(0.0f, trk.height);
            const float trk_side = std::max(trk.width, trk.height);
            const float trk_diag = std::hypot(trk.width, trk.height);
            if (trk_area <= 1.0f) continue;

            const float iou = computeIoU(det, trk);
            const bool standard_ok = (1.0f - iou) <= max_cost;
            const bool small_pair = cfg.small_object_match
                && std::min(det_side, trk_side) <= cfg.small_object_max_side;

            const float area_ratio = det_area / trk_area;
            const bool area_ok = area_ratio >= cfg.small_area_ratio_min
                              && area_ratio <= cfg.small_area_ratio_max;

            // A small track must not absorb a detector box whose area jumps
            // outside the configured range, even when containment gives it a
            // superficially acceptable IoU.
            if (small_pair && !area_ok) continue;

            const float det_cx = det.x + det.width * 0.5f;
            const float det_cy = det.y + det.height * 0.5f;
            const float trk_cx = trk.x + trk.width * 0.5f;
            const float trk_cy = trk.y + trk.height * 0.5f;
            const float center_distance = std::hypot(det_cx - trk_cx,
                                                     det_cy - trk_cy);
            const float reference_diag = std::max(1.0f,
                0.5f * (det_diag + trk_diag));
            const float center_ratio = center_distance / reference_diag;
            const bool center_ok = center_ratio
                                <= cfg.small_center_distance_ratio;
            const bool small_iou_ok = !second_stage
                || iou >= cfg.small_second_match_iou;
            const bool adaptive_ok = small_pair && area_ok && center_ok
                                  && small_iou_ok;
            if (!standard_ok && !adaptive_ok) continue;

            float quality = iou;
            if (small_pair) {
                const float center_score = std::max(0.0f,
                    1.0f - center_ratio / cfg.small_center_distance_ratio);
                const float scale_score = std::min(area_ratio,
                                                   1.0f / area_ratio);
                quality = 0.55f * iou + 0.30f * center_score
                        + 0.15f * scale_score;
            }
            pairs.push_back({static_cast<int>(d), static_cast<int>(t),
                             track_ids ? (*track_ids)[t]
                                       : static_cast<int>(t),
                             quality, iou});
        }
    }

    std::sort(pairs.begin(), pairs.end(),
              [](const Pair &a, const Pair &b) {
                  if (std::abs(a.quality - b.quality) > 1e-6f)
                      return a.quality > b.quality;
                  if (std::abs(a.iou - b.iou) > 1e-6f)
                      return a.iou > b.iou;
                  if (a.tid != b.tid) return a.tid < b.tid;
                  return a.d < b.d;
              });

    std::vector<bool> det_used(det_boxes.size(), false);
    std::vector<bool> trk_used(track_boxes.size(), false);
    std::vector<std::pair<int, int>> matches;
    for (const auto &pair : pairs) {
        if (det_used[pair.d] || trk_used[pair.t]) continue;
        matches.emplace_back(pair.d, pair.t);
        det_used[pair.d] = true;
        trk_used[pair.t] = true;
    }
    return matches;
}

inline void BYTETracker::update(
    std::vector<Detection> &detections, int frame_id,
    std::vector<std::shared_ptr<STrack>> &active_tracks,
    std::vector<TrackAssignment> &assignments,
    int prediction_steps)
{
    frame_id_ = frame_id;
    active_tracks.clear();
    assignments.clear();

    // ---- Step 1: predict all tracks ----
    // A voice inference can stretch the wall-clock interval between two
    // visual results. Advance the motion model by the equivalent number of
    // visual periods so association does not assume a fixed one-frame gap.
    prediction_steps = std::max(1, std::min(prediction_steps, 3));
    for (int step = 0; step < prediction_steps; ++step) {
        for (auto &t : tracked_stracks_) t->predict();
        for (auto &t : lost_stracks_)    t->predict();
    }

    // ---- Step 2: split detections ----
    std::vector<Detection> det_high, det_low;
    std::vector<int> det_high_orig_idx;  // index into original detections
    std::vector<int> det_low_orig_idx;   // P1 fix: track indices for low-dets too
    for (size_t i = 0; i < detections.size(); i++) {
        if (detections[i].score >= cfg_.track_high_thresh) {
            det_high.push_back(detections[i]);
            det_high_orig_idx.push_back(static_cast<int>(i));
        } else if (detections[i].score >= cfg_.track_low_thresh) {
            det_low.push_back(detections[i]);
            det_low_orig_idx.push_back(static_cast<int>(i));
        }
    }

    // ---- Step 3: first association — high dets ↔ ALL tracks (tracked + lost) ----
    // P0 fix: include lost_stracks_ so briefly-lost tracks can recover
    std::vector<std::shared_ptr<STrack>> all_tracks;
    all_tracks.reserve(tracked_stracks_.size() + lost_stracks_.size());
    for (auto &t : tracked_stracks_) all_tracks.push_back(t);
    for (auto &t : lost_stracks_)    all_tracks.push_back(t);

    std::vector<bool> track_matched(all_tracks.size(), false);
    std::vector<bool> det_high_matched(det_high.size(), false);

    {
        auto pred_boxes = tracksToPredictedRects(all_tracks);
        auto det_boxes  = detectionsToRects(det_high);
        std::vector<int> all_tids; all_tids.reserve(all_tracks.size());
        for (auto &t : all_tracks) all_tids.push_back(t->trackId());
        auto matches = adaptiveMatch(det_boxes, pred_boxes, cfg_.match_thresh,
                                     cfg_, false, &all_tids);

        for (auto &m : matches) {
            int di = m.first, ti = m.second;
            auto &t = all_tracks[ti];
            bool was_lost = t->isLost();

            t->update(det_high[di], frame_id_);
            if (was_lost) {
                // Recover from lost: move back to tracked
                t->activate(frame_id_);
                total_tracks_confirmed_++;  // re-confirmed
            }
            track_matched[ti] = true;
            det_high_matched[di] = true;

            assignments.push_back({
                t->trackId(),
                det_high_orig_idx[di],
                det_high[di].class_id,
                det_high[di].score
            });
        }
    }

    // ---- Step 4: second association — low dets ↔ unmatched tracks ----
    {
        std::vector<std::shared_ptr<STrack>> unmatched;
        std::vector<int> unmatched_orig_idx;
        for (size_t t = 0; t < all_tracks.size(); t++) {
            if (!track_matched[t]) {
                unmatched.push_back(all_tracks[t]);
                unmatched_orig_idx.push_back(static_cast<int>(t));
            }
        }

        if (!unmatched.empty() && !det_low.empty()) {
            auto pred_boxes = tracksToPredictedRects(unmatched);
            auto det_boxes  = detectionsToRects(det_low);
            std::vector<int> utids; utids.reserve(unmatched.size());
            for (auto &t : unmatched) utids.push_back(t->trackId());
            auto matches = adaptiveMatch(det_boxes, pred_boxes, 0.5f,
                                         cfg_, true, &utids);

            for (auto &m : matches) {
                int di = m.first, ti = m.second;
                int orig_ti = unmatched_orig_idx[ti];
                auto &t = all_tracks[orig_ti];
                bool was_lost = t->isLost();

                t->update(det_low[di], frame_id_);
                if (was_lost) {
                    t->activate(frame_id_);
                    total_tracks_confirmed_++;
                }
                track_matched[orig_ti] = true;

                // P1 fix: also push assignments for low-dets matches
                assignments.push_back({
                    t->trackId(),
                    det_low_orig_idx[di],
                    det_low[di].class_id,
                    det_low[di].score
                });
            }
        }
    }

    // ---- Step 5: rebuild tracked/lost from all_tracks ----
    std::vector<std::shared_ptr<STrack>> new_tracked, new_lost;
    int n_tracked_orig = static_cast<int>(tracked_stracks_.size());

    for (size_t t = 0; t < all_tracks.size(); t++) {
        bool was_tracked = (static_cast<int>(t) < n_tracked_orig);

        if (track_matched[t]) {
            // Matched this frame — keep/restore to tracked
            if (!all_tracks[t]->isActivated() && all_tracks[t]->state() != TrackState::Removed) {
                // New track that got matched — stays in tracked for promotion check
            }
            new_tracked.push_back(all_tracks[t]);
        } else {
            if (was_tracked) {
                all_tracks[t]->markLost();
                if (all_tracks[t]->isLost()) {
                    new_lost.push_back(all_tracks[t]);
                    total_tracks_lost_++;
                } else if (all_tracks[t]->isRemoved()) {
                    removed_stracks_.push_back(all_tracks[t]);
                    total_tracks_removed_++;
                }
            } else {
                // Was already lost — stays lost
                new_lost.push_back(all_tracks[t]);
            }
        }
    }

    // ---- Step 6: create new tracks from unmatched high dets ----
    for (size_t d = 0; d < det_high.size(); d++) {
        if (!det_high_matched[d] && det_high[d].score >= cfg_.new_track_thresh) {
            auto trk = std::make_shared<STrack>(det_high[d], track_id_counter_++, frame_id_);
            new_tracked.push_back(trk);
            total_tracks_created_++;
            // Push assignment so caller sees this detection→track mapping
            assignments.push_back({
                trk->trackId(),
                det_high_orig_idx[d],
                det_high[d].class_id,
                det_high[d].score
            });
        }
    }

    // ---- Step 7: remove old lost tracks ----
    std::vector<std::shared_ptr<STrack>> still_lost;
    for (auto &l : new_lost) {
        if (l->lostCount() >= cfg_.track_buffer) {
            l->markRemoved();
            removed_stracks_.push_back(l);
            total_tracks_removed_++;
        } else {
            still_lost.push_back(l);
        }
    }

    tracked_stracks_ = new_tracked;
    lost_stracks_    = still_lost;

    // ---- Step 8: promote surviving New tracks ----
    for (auto &t : tracked_stracks_) {
        if (t->state() == TrackState::New && t->trackletLength() >= 3) {
            t->activate(frame_id_);
            total_tracks_confirmed_++;
        }
    }

    // ---- Result: only Tracked tracks (not Lost) ----
    // P0 fix: don't return Lost tracks to caller
    for (auto &t : tracked_stracks_) {
        if (t->isActivated())
            active_tracks.push_back(t);
    }
}

inline std::vector<TrackSnapshot>
BYTETracker::recentLostSnapshots(int max_lost_count) const {
    std::vector<TrackSnapshot> snapshots;
    if (max_lost_count <= 0) return snapshots;
    snapshots.reserve(lost_stracks_.size());
    for (const auto &track : lost_stracks_) {
        if (!track || !track->isLost()) continue;
        if (track->lostCount() > max_lost_count) continue;
        // Display holds the last measured box. The predicted box is reserved
        // for association because extrapolated geometry must not leak into UI.
        snapshots.push_back({track->trackId(), track->lastBox(),
                             track->score(), track->lostCount()});
    }
    return snapshots;
}

} // namespace bytetrack

