#pragma once

#include <algorithm>
#include <cstdint>
#include <vector>

class PrimaryTrackSelector
{
public:
    explicit PrimaryTrackSelector(int reset_after_empty_frames = 15)
        : reset_after_empty_frames_(std::max(1, reset_after_empty_frames)) {}

    int update(const std::vector<int> &visible_track_ids)
    {
        if (visible_track_ids.empty())
        {
            if (++empty_frames_ >= reset_after_empty_frames_)
                primary_track_id_ = -1;
            return -1;
        }

        empty_frames_ = 0;
        if (primary_track_id_ < 0)
            primary_track_id_ = *std::min_element(visible_track_ids.begin(), visible_track_ids.end());

        if (std::find(visible_track_ids.begin(), visible_track_ids.end(), primary_track_id_) !=
            visible_track_ids.end())
            return primary_track_id_;
        return *std::min_element(visible_track_ids.begin(), visible_track_ids.end());
    }

    int boundTrackId() const { return primary_track_id_; }

    uint32_t logicalTrackId(int internal_track_id, int controller_track_id) const
    {
        return internal_track_id == controller_track_id
                   ? 1U
                   : static_cast<uint32_t>(internal_track_id) + 2U;
    }

private:
    int reset_after_empty_frames_;
    int empty_frames_ = 0;
    int primary_track_id_ = -1;
};
