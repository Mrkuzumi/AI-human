import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))

from finger_packet import FingerFrame  # noqa: E402
from gesture_trigger import (  # noqa: E402
    BODY_COUNT,
    BODY_LEFT_EAR,
    BODY_NOSE,
    BODY_RIGHT_EAR,
    FACE_NOSE_TIP,
    HAND_PALM,
    HAND_WRIST,
    HeadPatTrigger,
)
from motion_packet import Detection, MotionFrame  # noqa: E402


# Frame geometry: 1920x1080, nose at (0.50, 0.40) -> pixel (960, 432).
# Ears span 0.10 of the width -> head width 192 px, so the region is
# x 806..1114 px and y 182..470 px.
NOSE = (0.50, 0.40)
INSIDE_WRIST = (0.50, 0.35, 0.90)
INSIDE_PALM = (0.50, 0.32, 0.90)
FAR_WRIST = (0.50, 0.70, 0.90)
FAR_PALM = (0.50, 0.68, 0.90)


def motion(
    sequence=1,
    body=True,
    face=True,
    nose_conf=0.9,
    face_conf=0.9,
    ear_conf=0.9,
):
    body_detection = None
    if body:
        points = [(0.50, 0.50, 0.05)] * BODY_COUNT
        points[BODY_NOSE] = (NOSE[0], NOSE[1], nose_conf)
        points[BODY_LEFT_EAR] = (0.55, 0.40, ear_conf)
        points[BODY_RIGHT_EAR] = (0.45, 0.40, ear_conf)
        body_detection = Detection(0.95, (0.30, 0.20, 0.40, 0.50), tuple(points))
    face_detection = None
    if face:
        points = [(0.50, 0.50, 0.05)] * 51
        points[FACE_NOSE_TIP] = (NOSE[0], NOSE[1], face_conf)
        face_detection = Detection(0.95, (0.40, 0.30, 0.20, 0.20), tuple(points))
    return MotionFrame(
        sequence=sequence,
        capture_time_ns=0,
        emit_time_ns=0,
        body=body_detection,
        face=face_detection,
    )


def hand(wrist=INSIDE_WRIST, palm=INSIDE_PALM, score=0.9):
    points = [(0.50, 0.50, 0.05)] * 21
    points[HAND_WRIST] = wrist
    points[HAND_PALM] = palm
    return Detection(score, (0.40, 0.30, 0.20, 0.20), tuple(points))


def finger(sequence=1, hands=(None,)):
    if hands == (None,):
        hands = (hand(),)
    return FingerFrame(
        sequence=sequence,
        capture_time_ns=0,
        emit_time_ns=0,
        frame_width=1920,
        frame_height=1080,
        source_id=0,
        hands=tuple(hands),
    )


def no_hands(sequence=1):
    return FingerFrame(
        sequence=sequence,
        capture_time_ns=0,
        emit_time_ns=0,
        frame_width=1920,
        frame_height=1080,
        source_id=0,
        hands=(),
    )


class HeadPatDwellTests(unittest.TestCase):
    def test_dwell_needs_the_full_second(self):
        trigger = HeadPatTrigger()
        frame = motion()

        for stamp in (0.0, 0.25, 0.50, 0.75, 0.99):
            state = trigger.update(frame, finger(), now=stamp)
            self.assertEqual(state.active, 0, "fired too early at t={}".format(stamp))
            self.assertLess(state.progress, 1.0)

        self.assertEqual(trigger.update(frame, finger(), now=1.00).active, 1)

    def test_signal_stays_high_while_the_hand_rests(self):
        trigger = HeadPatTrigger()
        frame = motion()
        for stamp in (0.0, 0.50, 1.00):
            trigger.update(frame, finger(), now=stamp)

        high = sum(trigger.update(frame, finger(), now=1.0 + step * 0.1).active for step in range(1, 11))
        self.assertEqual(high, 10, "the signal must be a sustained state, not a one-frame pulse")

    def test_brief_dropout_does_not_reset_the_dwell(self):
        trigger = HeadPatTrigger()
        frame = motion()
        trigger.update(frame, finger(), now=0.0)
        trigger.update(frame, finger(), now=0.3)
        trigger.update(frame, finger(), now=0.6)

        state = trigger.update(frame, no_hands(), now=0.7)
        self.assertEqual(state.active, 0)
        self.assertAlmostEqual(state.dwell, 0.6, places=6)

        state = trigger.update(frame, finger(), now=0.8)
        self.assertAlmostEqual(state.dwell, 0.8, places=6)
        self.assertEqual(state.active, 0)

        self.assertEqual(trigger.update(frame, finger(), now=1.0).active, 1)

    def test_long_dropout_resets_the_dwell(self):
        trigger = HeadPatTrigger()
        frame = motion()
        trigger.update(frame, finger(), now=0.0)
        trigger.update(frame, finger(), now=0.3)

        state = trigger.update(frame, no_hands(), now=0.7)
        self.assertEqual(state.dwell, 0.0)
        self.assertEqual(state.active, 0)

    def test_release_then_rearm(self):
        trigger = HeadPatTrigger()
        frame = motion()
        for stamp in (0.0, 0.25, 0.50, 0.75, 1.00):
            trigger.update(frame, finger(), now=stamp)
        self.assertTrue(trigger.active)

        far = finger(hands=(hand(FAR_WRIST, FAR_PALM),))
        self.assertEqual(trigger.update(frame, far, now=1.20).active, 1, "must stay latched briefly")
        self.assertEqual(trigger.update(frame, far, now=1.40).active, 0, "must release when the hand leaves")

        state = trigger.update(frame, finger(), now=1.50)
        self.assertEqual(state.active, 0)
        self.assertEqual(trigger.update(frame, finger(), now=2.50).active, 1)

    def test_cooldown_delays_a_fast_second_hit(self):
        trigger = HeadPatTrigger(dwell_seconds=0.2, cooldown_seconds=0.5)
        frame = motion()
        trigger.update(frame, finger(), now=0.0)

        self.assertEqual(trigger.update(frame, finger(), now=0.3).active, 0)
        self.assertEqual(trigger.update(frame, finger(), now=0.6).active, 1)


class HeadPatGeometryTests(unittest.TestCase):
    def test_hand_at_the_nose_is_inside_the_region(self):
        trigger = HeadPatTrigger()
        state = trigger.update(
            motion(), finger(hands=(hand((0.50, 0.42, 0.90), (0.50, 0.40, 0.90)),)), now=0.0
        )
        self.assertEqual(state.outside, 0.0)

    def test_chin_height_does_not_count_as_patting_the_head(self):
        trigger = HeadPatTrigger()
        state = trigger.update(
            motion(), finger(hands=(hand((0.50, 0.48, 0.90), (0.50, 0.45, 0.90)),)), now=0.0
        )
        self.assertGreater(state.outside, 0.0)
        self.assertEqual(state.active, 0)

    def test_whole_hand_is_required_by_default(self):
        frame = motion()
        low_wrist = (0.50, 0.60, 0.90)
        good_palm = (0.50, 0.32, 0.90)
        hands = (hand(low_wrist, good_palm),)

        strict = HeadPatTrigger(whole_hand=True)
        self.assertGreater(strict.update(frame, finger(hands=hands), now=0.0).outside, 0.0)

        loose = HeadPatTrigger(whole_hand=False)
        self.assertEqual(loose.update(frame, finger(hands=hands), now=0.0).outside, 0.0)

    def test_head_width_falls_back_when_ears_are_missing(self):
        trigger = HeadPatTrigger()
        state = trigger.update(motion(ear_conf=0.05), finger(), now=0.0)
        self.assertAlmostEqual(state.head_width_px, 0.20 * 1080, places=6)

    def test_head_width_is_cached_once_measured(self):
        """Losing the ears must not balloon the region to the fixed fallback."""
        trigger = HeadPatTrigger()
        measured = trigger.update(motion(), finger(), now=0.0).head_width_px
        self.assertAlmostEqual(measured, 192.0, places=6)

        occluded = motion(ear_conf=0.05)
        state = trigger.update(occluded, finger(), now=0.2)
        self.assertAlmostEqual(state.head_width_px, measured, places=6)
        self.assertNotAlmostEqual(state.head_width_px, 0.20 * 1080)

        state = trigger.update(occluded, finger(), now=1.0)
        self.assertAlmostEqual(state.head_width_px, measured, places=6)

    def test_head_width_cache_expires(self):
        trigger = HeadPatTrigger(head_width_cache_seconds=1.0)
        trigger.update(motion(), finger(), now=0.0)
        stale = trigger.update(motion(ear_conf=0.05), finger(), now=2.0)
        self.assertAlmostEqual(stale.head_width_px, 0.20 * 1080, places=6)

    def test_head_width_smoothing_rejects_single_frame_spikes(self):
        """One noisy ear detection must not instantly resize the region."""
        trigger = HeadPatTrigger(head_width_smoothing=0.35)
        trigger.update(motion(), finger(), now=0.0)

        spiked = motion()
        points = list(spiked.body.keypoints)
        points[BODY_LEFT_EAR] = (0.62, 0.40, 0.90)
        points[BODY_RIGHT_EAR] = (0.42, 0.40, 0.90)
        spiked = MotionFrame(
            sequence=2,
            capture_time_ns=0,
            emit_time_ns=0,
            body=Detection(0.95, (0.1, 0.1, 0.8, 0.8), tuple(points)),
            face=spiked.face,
        )
        state = trigger.update(spiked, finger(), now=0.1)
        # Raw span is 0.20 * 1920 = 384 px; the EMA pulls it only part way
        # from the 192 px baseline instead of jumping straight to 384.
        expected = 192.0 + 0.35 * (384.0 - 192.0)
        self.assertAlmostEqual(state.head_width_px, expected, places=6)
        self.assertLess(state.head_width_px, 384.0)

    def test_thresholds_scale_with_head_width(self):
        """A near hand and a far hand both register, without re-tuning thresholds."""
        trigger = HeadPatTrigger()
        # Half-size head: nose and ears squeezed into the centre of the frame.
        small = motion()
        points = list(small.body.keypoints)
        points[BODY_LEFT_EAR] = (0.525, 0.40, 0.9)
        points[BODY_RIGHT_EAR] = (0.475, 0.40, 0.9)
        small = MotionFrame(
            sequence=1,
            capture_time_ns=0,
            emit_time_ns=0,
            body=Detection(0.95, (0.4, 0.3, 0.2, 0.2), tuple(points)),
            face=small.face,
        )
        state = trigger.update(small, finger(hands=(hand((0.50, 0.35, 0.9), (0.50, 0.33, 0.9)),)), now=0.0)
        self.assertAlmostEqual(state.head_width_px, 0.05 * 1920, places=6)
        self.assertEqual(state.outside, 0.0)


class HeadPatAnchorTests(unittest.TestCase):
    def test_face_nose_tip_is_used_when_the_pose_nose_is_occluded(self):
        trigger = HeadPatTrigger()
        state = trigger.update(motion(nose_conf=0.05, face_conf=0.90), finger(), now=0.0)
        self.assertEqual(state.nose_source, "face")
        self.assertEqual(state.outside, 0.0)

    def test_cached_nose_survives_a_fully_occluded_face(self):
        trigger = HeadPatTrigger()
        frame = motion()
        self.assertFalse(trigger.update(frame, finger(), now=0.0).nose_cached)

        occluded = motion(nose_conf=0.05, face_conf=0.05)
        state = trigger.update(occluded, finger(), now=0.2)
        self.assertTrue(state.nose_cached)
        self.assertEqual(state.nose_source, "body")
        self.assertEqual(state.outside, 0.0)

        expired = trigger.update(occluded, finger(), now=0.8)
        self.assertIsNone(expired.outside)

    def test_low_confidence_hand_is_ignored(self):
        trigger = HeadPatTrigger()
        weak = hand((0.50, 0.35, 0.05), (0.50, 0.32, 0.90))
        self.assertIsNone(trigger.update(motion(), finger(hands=(weak,)), now=0.0).outside)

    def test_missing_body_still_works_from_the_face_anchor(self):
        trigger = HeadPatTrigger()
        state = trigger.update(motion(body=False, face=True), finger(), now=0.0)
        self.assertEqual(state.nose_source, "face")
        self.assertEqual(state.outside, 0.0)


if __name__ == "__main__":
    unittest.main()
