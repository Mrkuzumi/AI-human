#include "primary_track_selector.hpp"

#include <cassert>

int main()
{
    PrimaryTrackSelector selector(3);
    int controller = selector.update({4, 7});
    assert(controller == 4);
    assert(selector.logicalTrackId(4, controller) == 1);
    assert(selector.logicalTrackId(7, controller) == 9);

    // A visible fallback controls the avatar while the primary is occluded.
    controller = selector.update({7});
    assert(controller == 7);
    assert(selector.logicalTrackId(7, controller) == 1);
    assert(selector.boundTrackId() == 4);

    // The original primary regains control when visible again.
    assert(selector.update({4, 7}) == 4);

    // A fully empty scene releases the old primary, allowing reacquisition.
    assert(selector.update({}) == -1);
    assert(selector.update({}) == -1);
    assert(selector.update({}) == -1);
    assert(selector.boundTrackId() == -1);
    assert(selector.update({12}) == 12);
    assert(selector.logicalTrackId(12, 12) == 1);
    return 0;
}
