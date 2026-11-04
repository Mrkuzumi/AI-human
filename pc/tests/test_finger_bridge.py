import json
import math
import random
import socket
import sys
import tempfile
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))

from finger_packet import FingerFrame, pack_finger_frame, unpack_finger_frame  # noqa: E402
from motion_bridge import (  # noqa: E402
    HAND_OUTPUT_BONES, FingerJsonlLogger, FingerSolver, build_parser, poll_finger_frame,
    select_finger_frame,
)
from motion_packet import Detection, MotionFrame  # noqa: E402


def _body():
    points = [(0.5, 0.5, 0.99) for _ in range(17)]
    points[7] = (0.78, 0.55, 0.99)
    points[8] = (0.22, 0.55, 0.99)
    points[9] = (0.78, 0.70, 0.99)
    points[10] = (0.22, 0.70, 0.99)
    return Detection(0.99, (0.2, 0.1, 0.6, 0.85), tuple(points))


def _hand(wrist_x, closed=False):
    points = [(wrist_x, 0.70, 0.99)]
    offsets = (-0.08, -0.04, 0.0, 0.04, 0.08)
    for offset in offsets:
        x = wrist_x + offset
        if closed:
            points.extend(
                ((x, 0.55, 0.99), (x + 0.06, 0.55, 0.99),
                 (x + 0.06, 0.62, 0.99), (x, 0.62, 0.99))
            )
        else:
            points.extend(
                ((x, 0.55, 0.99), (x + offset * 0.2, 0.45, 0.99),
                 (x + offset * 0.4, 0.35, 0.99), (x + offset * 0.6, 0.25, 0.99))
            )
    return Detection(0.98, (wrist_x - 0.12, 0.20, 0.24, 0.52), tuple(points))


def _wide_frame_detection(detection):
    ratio = 9.0 / 16.0
    return Detection(
        detection.score,
        (detection.box[0] * ratio, detection.box[1], detection.box[2] * ratio, detection.box[3]),
        tuple((x * ratio, y, confidence) for x, y, confidence in detection.keypoints),
    )


def _z_angle(quaternion):
    return math.degrees(2.0 * math.atan2(quaternion[2], quaternion[3]))


def _x_angle(quaternion):
    return math.degrees(2.0 * math.atan2(quaternion[0], quaternion[3]))


def _hand3d(surface="palm", closed=False, handedness="Right", wrist_x=0.22):
    points = [(0.0, 0.0, 0.0)] * 21
    for chain in ((1, 2, 3, 4), (5, 6, 7, 8), (9, 10, 11, 12), (13, 14, 15, 16), (17, 18, 19, 20)):
        if closed:
            values = ((0.02, 0.0, 0.0), (0.04, 0.0, 0.0), (0.04, 0.03, 0.0), (0.0, 0.03, 0.0))
        else:
            values = ((0.0, 0.03, 0.0), (0.0, 0.06, 0.0), (0.0, 0.09, 0.0), (0.0, 0.12, 0.0))
        for index, value in zip(chain, values):
            points[index] = value
    return {
        "handedness": handedness,
        "score": 0.99,
        "palm_normal": (0.0, 0.0, -0.9 if surface == "palm" else 0.9),
        "landmarks": ((wrist_x, 0.70, 0.0),) * 21,
        "world_landmarks": points,
    }


def _thumb_hand(side="Left", closed=False, thumb_closed=None):
    """A real palm span; a closed thumb can oppose without a bent silhouette."""
    if thumb_closed is None:
        thumb_closed = closed
    sign = 1.0 if side == "Left" else -1.0
    points = [(0.0, 0.0, 0.0)] * 21
    thumb = ((.025, .015, 0), (.024, .033, 0), (.023, .051, 0), (.022, .069, 0)) if thumb_closed else (
        (.025, .015, 0), (.05, .025, 0), (.075, .035, 0), (.10, .045, 0))
    points[1:5] = thumb
    for start, x, y in ((5, .025, .07), (9, 0, .08), (13, -.015, .07), (17, -.035, .06)):
        points[start:start + 4] = (
            ((x, y, 0), (x, y + .025, .015), (x, y + .01, .035), (x, y - .015, .025))
            if closed else tuple((x, y + .025 * i, 0) for i in range(4))
        )
    points = [(sign * x, y, z) for x, y, z in points]
    wrist = .78 if side == "Left" else .22
    projected = tuple((wrist + x, .70 - y, .99) for x, y, z in points)
    detection = Detection(.99, (wrist - .12, .45, .24, .27), projected)
    world = {"handedness": side, "score": .99, "palm_normal": (0, 0, -sign),
             "landmarks": tuple((x, y, 0) for x, y, _ in projected), "world_landmarks": points}
    return detection, world


def _rotation_angle(quaternion):
    return math.degrees(2 * math.acos(min(1.0, abs(quaternion[3]))))


class ThumbMappingTests(unittest.TestCase):
    def test_both_straight_opposed_thumbs_close_all_three_joints(self):
        for use3d in (False, True):
            left, left3d = _thumb_hand("Left", True)
            right, right3d = _thumb_hand("Right", True)
            solver = FingerSolver(smoothing=1, hand3d_drive=use3d)
            solved = dict(solver.solve((left, right), _body(), (left3d, right3d), 1))
            self.assertGreater(solver.diagnostics["output_thumb_oppositions"]["Left"], 40)
            for segment in ("Proximal", "Intermediate", "Distal"):
                l, r = solved["LeftThumb" + segment], solved["RightThumb" + segment]
                self.assertGreater(_rotation_angle(l), 20)
                for actual, expected in zip(r, (l[0], -l[1], -l[2], l[3])):
                    self.assertAlmostEqual(actual, expected)

    def test_open_thumb_and_fist_with_thumb_up_stay_extended(self):
        for side in ("Left", "Right"):
            for closed in (False, True):
                hand, world = _thumb_hand(side, closed, thumb_closed=False)
                solver = FingerSolver(smoothing=1, hand3d_drive=True)
                for frame in range(12):
                    solved = dict(solver.solve((hand,), _body(), (world,), frame))
                self.assertEqual(solver.diagnostics["output_thumb_oppositions"][side], 0)
                for segment in ("Proximal", "Intermediate", "Distal"):
                    self.assertLess(_rotation_angle(solved[side + "Thumb" + segment]), 1)
                if closed:
                    self.assertGreater(abs(_z_angle(solved[side + "MiddleIntermediate"])), 30)

    def test_world_geometry_overrides_false_projected_thumb_closure(self):
        projected, _ = _thumb_hand("Left", False)
        closed, _ = _thumb_hand("Left", True)
        points = list(projected.keypoints)
        points[1:5] = closed.keypoints[1:5]
        projected = Detection(projected.score, projected.box, tuple(points))
        _, world = _thumb_hand("Left", False)
        solver = FingerSolver(smoothing=1, hand3d_drive=True)
        solved = dict(solver.solve((projected,), _body(), (world,), 1))
        self.assertEqual(solver.diagnostics["output_thumb_oppositions"]["Left"], 0)
        self.assertLess(_rotation_angle(solved["LeftThumbProximal"]), 1)

    def test_world_thumb_bend_detects_depth_not_visible_in_2d(self):
        hand, world = _thumb_hand()
        world["world_landmarks"][3] = (.05, .025, .03)
        world["world_landmarks"][4] = (.06, .025, .04)
        solver = FingerSolver(smoothing=1, hand3d_drive=True)
        solved = dict(solver.solve((hand,), _body(), (world,), 1))
        self.assertGreater(_rotation_angle(solved["LeftThumbIntermediate"]), 30)

    def test_opposition_is_invariant_under_world_rotation_and_scale(self):
        solver = FingerSolver()
        for closed in (False, True):
            _, world = _thumb_hand(closed=closed)
            points = world["world_landmarks"]
            expected = solver._thumb_opposition(points, world=True)
            turned = [(3 * z + 2, 3 * x - 1, 3 * y + 5) for x, y, z in points]
            self.assertAlmostEqual(solver._thumb_opposition(turned, world=True), expected)

    def test_low_confidence_thumb_is_held_not_forced_closed_or_open(self):
        solver = FingerSolver(smoothing=1, missing_hold_frames=2)
        hand, _ = _thumb_hand(closed=True)
        active = dict(solver.solve((hand,), _body()))
        points = list(hand.keypoints)
        for i in (1, 2, 3, 4):
            points[i] = (points[i][0], points[i][1], 0)
        held = dict(solver.solve((Detection(hand.score, hand.box, tuple(points)),), _body()))
        for segment in ("Proximal", "Intermediate", "Distal"):
            self.assertEqual(held["LeftThumb" + segment], active["LeftThumb" + segment])

    def test_open_close_open_is_smoothed_and_missing_thumb_holds_then_relaxes(self):
        solver = FingerSolver(smoothing=.25, missing_hold_frames=2, hand3d_drive=True)
        opened, open3d = _thumb_hand()
        closed, closed3d = _thumb_hand(closed=True)
        solver.solve((opened,), _body(), (open3d,), 0)
        for detection, world, increasing in ((closed, closed3d, True), (opened, open3d, False)):
            previous = solver.diagnostics["output_thumb_oppositions"]["Left"]
            for frame in range(1, 21):
                solver.solve((detection,), _body(), (world,), frame)
                value = solver.diagnostics["output_thumb_oppositions"]["Left"]
                self.assertGreaterEqual(value, previous) if increasing else self.assertLessEqual(value, previous)
                self.assertLessEqual(abs(value - previous), 11.25)
                previous = value
        solver.solve((closed,), _body(), (closed3d,), 30)
        active = dict(solver.solve((closed,), _body(), (closed3d,), 31))["LeftThumbProximal"]
        self.assertEqual(dict(solver.solve((), _body()))["LeftThumbProximal"], active)
        self.assertEqual(dict(solver.solve((), _body()))["LeftThumbProximal"], active)
        self.assertLess(_rotation_angle(dict(solver.solve((), _body()))["LeftThumbProximal"]), _rotation_angle(active))

    def test_both_fists_close_when_thumb_landmarks_are_occluded(self):
        for side in ("Left", "Right"):
            hand, _ = _thumb_hand(side, closed=True)
            points = list(hand.keypoints)
            for index in range(1, 5):
                points[index] = (points[index][0], points[index][1], 0.01)
            solver = FingerSolver(smoothing=1)
            solved = dict(solver.solve((Detection(hand.score, hand.box, tuple(points)),), _body()))
            self.assertGreater(solver.diagnostics[side]["thumb_grasp"]["closure"], .95)
            for segment in ("Proximal", "Intermediate", "Distal"):
                self.assertGreater(_rotation_angle(solved[side + "Thumb" + segment]), 20)

    def test_hallucinated_3d_extended_thumb_cannot_override_visible_fist(self):
        for side in ("Left", "Right"):
            hand, closed3d = _thumb_hand(side, closed=True)
            _, wrong3d = _thumb_hand(side, closed=True, thumb_closed=False)
            # The inferred depth says "extended", but the image shows a folded thumb.
            wrong3d["landmarks"] = closed3d["landmarks"]
            for board_visible in (False, True):
                solver = FingerSolver(smoothing=1, hand3d_drive=True)
                for frame in range(20):
                    solved = dict(solver.solve((hand,) if board_visible else (), _body(), (wrong3d,), frame))
                self.assertGreater(solver.diagnostics[side]["thumb_grasp"]["closure"], .9)
                self.assertGreater(_rotation_angle(solved[side + "ThumbIntermediate"]), 35)

    def test_repeated_rgb_frame_does_not_accumulate_thumb_up_evidence(self):
        _, world = _thumb_hand(closed=True, thumb_closed=False)
        solver = FingerSolver(smoothing=1, hand3d_drive=True)
        solver.solve((), _body(), (world,), 7)
        first = solver.diagnostics["Left"]["thumb_grasp"]["extension_filtered"]
        for _ in range(20):
            solver.solve((), _body(), (world,), 7)
        self.assertEqual(solver.diagnostics["Left"]["thumb_grasp"]["extension_filtered"], first)
        for sequence in range(8, 20):
            solved = dict(solver.solve((), _body(), (world,), sequence))
        self.assertLess(_rotation_angle(solved["LeftThumbProximal"]), 1)
        # Inferred 3D extension without a usable silhouette is not proof of a visible thumb-up.
        world["landmarks"] = ((.78, .7, 0),) * 21
        for sequence in range(20, 35):
            solved = dict(solver.solve((), _body(), (world,), sequence))
        self.assertGreater(_rotation_angle(solved["LeftThumbProximal"]), 20)

    def test_single_curled_finger_does_not_force_thumb_into_fist(self):
        hand, _ = _thumb_hand()
        closed, _ = _thumb_hand(closed=True)
        points = list(hand.keypoints)
        points[5:9] = closed.keypoints[5:9]
        solver = FingerSolver(smoothing=1)
        solved = dict(solver.solve((Detection(hand.score, hand.box, tuple(points)),), _body()))
        self.assertEqual(solver.diagnostics["Left"]["thumb_grasp"]["closure"], 0)
        self.assertLess(_rotation_angle(solved["LeftThumbProximal"]), 1)

    def test_noisy_hidden_thumb_has_stable_natural_fist_pose_on_both_sides(self):
        noise = random.Random(1405)
        for side in ("Left", "Right"):
            solver = FingerSolver(smoothing=1, hand3d_drive=True)
            reference = None
            for frame in range(100):
                hand, world = _thumb_hand(side, closed=True)
                for i in range(1, 5):
                    world["world_landmarks"][i] = tuple(v + noise.uniform(-.025, .025) for v in world["world_landmarks"][i])
                solved = dict(solver.solve((hand,), _body(), (world,), frame))
                joints = [solved[side + "Thumb" + segment] for segment in ("Proximal", "Intermediate", "Distal")]
                if reference is None:
                    reference = joints
                for actual, expected in zip(joints, reference):
                    for a, b in zip(actual, expected):
                        self.assertAlmostEqual(a, b, places=6)

    def test_confirmed_thumb_up_folds_smoothly_when_thumb_becomes_hidden_in_fist(self):
        solver = FingerSolver(smoothing=.35, hand3d_drive=True)
        opened, opened3d = _thumb_hand(closed=True, thumb_closed=False)
        for frame in range(20):
            previous = dict(solver.solve((opened,), _body(), (opened3d,), frame))["LeftThumbProximal"]
        hand, world = _thumb_hand(closed=True)
        for frame in range(20, 40):
            solved = dict(solver.solve((hand,), _body(), (world,), frame))
            current = solved["LeftThumbProximal"]
            dot = min(1.0, abs(sum(a * b for a, b in zip(current, previous))))
            self.assertLess(math.degrees(2 * math.acos(dot)), 25)
            previous = current
        self.assertGreater(solver.diagnostics["output_thumb_oppositions"]["Left"], 44)
        self.assertGreater(_rotation_angle(solved["LeftThumbIntermediate"]), 40)

    def test_slow_fist_with_hidden_thumb_does_not_snap_at_first_closure_evidence(self):
        opened, _ = _thumb_hand()
        solver = FingerSolver(smoothing=1)
        previous = None
        for frame in range(121):
            points = list(opened.keypoints)
            curl = math.radians(80 * frame / 120)
            for start in (5, 9, 13, 17):
                for segment in (1, 2, 3):
                    x, y, _ = points[start + segment - 1]
                    points[start + segment] = (x + .025 * math.sin(segment * curl),
                                               y - .025 * math.cos(segment * curl), .99)
            for i in range(1, 5):
                points[i] = (*points[i][:2], 0)
            solved = dict(solver.solve((Detection(.99, opened.box, tuple(points)),), _body()))
            current = solved["LeftThumbIntermediate"]
            if previous:
                dot = min(1.0, abs(sum(a * b for a, b in zip(current, previous))))
                self.assertLess(math.degrees(2 * math.acos(dot)), 10)
            previous = current
        self.assertGreater(_rotation_angle(current), 40)


class FingerPacketTests(unittest.TestCase):
    def test_round_trip_zero_one_and_two_hands(self):
        for count in range(3):
            frame = FingerFrame(7, 100, 200, 1920, 1080, 0, tuple(_hand(0.3 + i * 0.4) for i in range(count)))
            decoded = unpack_finger_frame(pack_finger_frame(frame))
            self.assertEqual(
                (decoded.sequence, decoded.capture_time_ns, decoded.emit_time_ns,
                 decoded.frame_width, decoded.frame_height, decoded.source_id),
                (7, 100, 200, 1920, 1080, 0),
            )
            self.assertEqual(len(decoded.hands), count)
            for actual, expected in zip(decoded.hands, frame.hands):
                self.assertAlmostEqual(actual.score, expected.score, places=6)
                for actual_value, expected_value in zip(actual.box, expected.box):
                    self.assertAlmostEqual(actual_value, expected_value, places=6)
                for actual_point, expected_point in zip(actual.keypoints, expected.keypoints):
                    for actual_value, expected_value in zip(actual_point, expected_point):
                        self.assertAlmostEqual(actual_value, expected_value, places=6)

    def test_crc_corruption_is_rejected(self):
        packet = bytearray(pack_finger_frame(FingerFrame(7, 100, 200, 1920, 1080, 0, (_hand(0.5),))))
        packet[50] ^= 1
        with self.assertRaisesRegex(ValueError, "CRC32"):
            unpack_finger_frame(bytes(packet))

    def test_rejects_truncated_packet(self):
        with self.assertRaises(ValueError):
            unpack_finger_frame(b"FNG1")


class FingerSolverTests(unittest.TestCase):
    def test_wide_frame_hand_angles_match_square_pixel_geometry(self):
        hands = (_hand(0.22, True), _hand(0.78, False))
        body = _body()
        expected_solver = FingerSolver(smoothing=1.0)
        actual_solver = FingerSolver(smoothing=1.0)
        expected = dict(expected_solver.solve(hands, body, frame_width=1, frame_height=1))
        actual = dict(actual_solver.solve(
            tuple(_wide_frame_detection(hand) for hand in hands),
            _wide_frame_detection(body),
            frame_width=1920,
            frame_height=1080,
        ))
        self.assertEqual(actual_solver.assigned_sides, expected_solver.assigned_sides)
        for bone in HAND_OUTPUT_BONES:
            for actual_value, expected_value in zip(actual[bone], expected[bone]):
                self.assertAlmostEqual(actual_value, expected_value)

    def test_pose_wrists_assign_reversed_detector_order_to_anatomical_sides(self):
        solver = FingerSolver(smoothing=1.0)
        solved = dict(solver.solve((_hand(0.22, True), _hand(0.78, True)), _body()))
        self.assertEqual(tuple(solved), HAND_OUTPUT_BONES)
        self.assertEqual(solver.assigned_sides, ("Left", "Right"))
        self.assertGreater(_z_angle(solved["LeftMiddleIntermediate"]), 60.0)
        self.assertLess(_z_angle(solved["RightMiddleIntermediate"]), -60.0)

    def test_open_hand_produces_near_neutral_fingers(self):
        solved = dict(FingerSolver(smoothing=1.0).solve((_hand(0.78),), _body()))
        self.assertLess(abs(_z_angle(solved["LeftMiddleIntermediate"])), 5.0)
        self.assertLess(abs(_z_angle(solved["LeftMiddleDistal"])), 5.0)

    def test_missing_hand_holds_briefly_then_relaxes(self):
        solver = FingerSolver(smoothing=1.0, missing_hold_frames=2)
        active = dict(solver.solve((_hand(0.78, True),), _body()))["LeftMiddleIntermediate"]
        self.assertGreater(_z_angle(active), 60.0)
        self.assertEqual(dict(solver.solve((), _body()))["LeftMiddleIntermediate"], active)
        self.assertEqual(dict(solver.solve((), _body()))["LeftMiddleIntermediate"], active)
        relaxed = dict(solver.solve((), _body()))["LeftMiddleIntermediate"]
        self.assertAlmostEqual(_z_angle(relaxed), 0.0)

    def test_low_confidence_joint_is_held_instead_of_spiking(self):
        solver = FingerSolver(smoothing=1.0, missing_hold_frames=1)
        hand = _hand(0.78, True)
        active = dict(solver.solve((hand,), _body()))["LeftMiddleIntermediate"]
        points = list(hand.keypoints)
        points[10] = (points[10][0], points[10][1], 0.01)
        noisy = Detection(hand.score, hand.box, tuple(points))
        held = dict(solver.solve((noisy,), _body()))["LeftMiddleIntermediate"]
        self.assertEqual(held, active)

    def test_closed_hand_exposes_high_projected_reach_curl(self):
        solver = FingerSolver(smoothing=1.0)
        solver.solve((_hand(0.78, True),), _body())
        middle = solver.diagnostics["Left"]["fingers"]["Middle"]
        self.assertLess(middle["reach_ratio"], 0.5)
        self.assertGreater(middle["reach_curl"], 80.0)

    def test_hand_roll_is_relative_to_forearm(self):
        hand = _hand(0.78)
        points = list(hand.keypoints)
        points[9] = (0.63, 0.70, 0.99)
        rotated = Detection(hand.score, hand.box, tuple(points))
        solved = dict(FingerSolver(smoothing=1.0).solve((rotated,), _body()))
        self.assertGreater(abs(_z_angle(solved["LeftHand"])), 70.0)

    def test_3d_twist_uses_palm_down_as_neutral_for_both_hands(self):
        solver = FingerSolver(smoothing=1.0, hand3d_drive=True)
        palm = dict(_hand3d("palm"))
        points = list(palm["world_landmarks"])
        points[9] = (0.1, 0.0, 0.0)
        palm["world_landmarks"] = points
        palm["palm_normal"] = (0.0, 1.0, 0.0)
        neutral = dict(solver.solve((), _body(), (palm,), 1))
        turned = dict(palm)
        turned["palm_normal"] = (0.0, 0.0, -1.0)
        quarter = dict(solver.solve((), _body(), (turned,), 2))
        self.assertAlmostEqual(_x_angle(neutral["RightHand"]), 0.0)
        self.assertAlmostEqual(_x_angle(quarter["RightHand"]), 90.0)
        self.assertAlmostEqual(solver.diagnostics["Right"]["hand3d"]["twist_target"], 90.0)

        left = dict(palm)
        left["handedness"] = "Left"
        left["palm_normal"] = (0.0, -1.0, 0.0)
        left_neutral = dict(FingerSolver(smoothing=1.0, hand3d_drive=True).solve(
            (), _body(), (left,), 1
        ))
        self.assertAlmostEqual(_x_angle(left_neutral["LeftHand"]), 0.0)

    def test_3d_left_hand_uses_wrist_position_when_handedness_is_mirrored(self):
        hand3d = _hand3d("back", handedness="Right", wrist_x=0.78)
        points = list(hand3d["world_landmarks"])
        points[9] = (0.1, 0.0, 0.0)
        hand3d["world_landmarks"] = points
        solved = dict(FingerSolver(smoothing=1.0, hand3d_drive=True).solve(
            (_hand(0.78),), _body(), (hand3d,), 1, frame_width=1920, frame_height=1080
        ))
        self.assertAlmostEqual(_x_angle(solved["LeftHand"]), -90.0)
        self.assertAlmostEqual(_x_angle(solved["RightHand"]), 0.0)

    def test_widescreen_fallback_assigns_single_hand_using_image_center(self):
        solver = FingerSolver(smoothing=1)
        solver.solve((_hand(.40),), None, frame_width=1920, frame_height=1080)
        self.assertEqual(solver.assigned_sides, ("Right",))
        solver.solve((_hand(.60),), None, frame_width=1920, frame_height=1080)
        self.assertEqual(solver.assigned_sides, ("Left",))

    def test_both_palms_facing_camera_use_matching_unity_twist(self):
        left = _hand3d("back", handedness="Left", wrist_x=0.78)
        right = _hand3d("palm", handedness="Right", wrist_x=0.22)
        for hand, middle in ((left, (0.1, 0.0, 0.0)), (right, (-0.1, 0.0, 0.0))):
            points = list(hand["world_landmarks"])
            points[9] = middle
            hand["world_landmarks"] = points
        solved = dict(FingerSolver(smoothing=1.0, hand3d_drive=True).solve(
            (_hand(0.78), _hand(0.22)), _body(), (left, right), 1
        ))
        self.assertAlmostEqual(_x_angle(solved["LeftHand"]), -90.0)
        self.assertAlmostEqual(_x_angle(solved["RightHand"]), -90.0)

    def test_hand_roll_ignores_a_pose_wrist_far_from_the_detected_hand(self):
        body = _body()
        points = list(body.keypoints)
        points[9] = (0.78, 0.20, 0.99)
        body = Detection(body.score, body.box, tuple(points))
        solver = FingerSolver(smoothing=1.0)
        solver.solve((_hand(0.78),), body)
        self.assertIsNone(solver.diagnostics["Left"]["hand_roll_target"])

    def test_3d_closed_hand_fills_missing_2d_curl(self):
        solved = dict(FingerSolver(smoothing=1.0, hand3d_drive=True).solve(
            (), _body(), (_hand3d("back", closed=True),), 1
        ))
        self.assertLess(_z_angle(solved["RightMiddleIntermediate"]), -70.0)

    def test_finger_jsonl_logger_keeps_raw_points_and_solver_features(self):
        hand = _hand(0.78, True)
        finger_frame = FingerFrame(12, 100, 200, 1920, 1080, 0, (hand,))
        motion_frame = MotionFrame(12, 100, 200, body=_body())
        solver = FingerSolver(smoothing=1.0)
        solver.solve((hand,), motion_frame.body)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "finger.jsonl"
            logger = FingerJsonlLogger(str(path))
            logger.write(motion_frame, finger_frame, "192.168.126.49", solver.diagnostics)
            logger.close()
            record = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(record["motion_sequence"], 12)
        self.assertEqual(record["finger_lag"], 0)
        self.assertEqual(len(record["hands"][0]["keypoints"]), 21)
        self.assertIn("reach_ratio", record["solver"]["Left"]["fingers"]["Middle"])


class FingerSequenceTests(unittest.TestCase):
    def test_default_bridge_uses_same_frame_fingers_without_rgb_fusion(self):
        args = build_parser().parse_args([])
        self.assertEqual(args.finger_max_lag, 0)
        self.assertEqual(args.rgb_pose_model, "")
        self.assertFalse(args.rgb_hand_drive)

    def test_prefers_exact_sequence_then_recent_lag(self):
        old = FingerFrame(8, 100, 200, 1920, 1080, 0, ())
        exact = FingerFrame(10, 100, 200, 1920, 1080, 0, ())
        other_camera = FingerFrame(10, 100, 200, 1920, 1080, 1, ())
        self.assertIs(select_finger_frame((old, exact, other_camera), 10, 0, 2), exact)
        self.assertIs(select_finger_frame((old,), 10, 0, 2), old)
        self.assertIs(select_finger_frame((exact,), 10, 0, 0), exact)
        self.assertIsNone(select_finger_frame((old,), 10, 0, 0))
        self.assertIsNone(select_finger_frame((old,), 11, 0, 2))

    def test_sequence_wraparound_is_recent(self):
        wrapped = FingerFrame(0xFFFFFFFF, 100, 200, 1920, 1080, 0, ())
        self.assertIs(select_finger_frame((wrapped,), 0, 0, 1), wrapped)

    def test_zero_wait_still_drains_an_already_available_packet(self):
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            receiver.bind(("127.0.0.1", 0))
            receiver.setblocking(False)
            frame = FingerFrame(12, 100, 200, 1920, 1080, 0, ())
            sender.sendto(pack_finger_frame(frame), receiver.getsockname())
            selected, accepted, rejected = poll_finger_frame(receiver, {}, 12, 0, 0.0, 0)
        finally:
            sender.close()
            receiver.close()
        self.assertEqual((selected, accepted, rejected), (frame, 1, 0))


if __name__ == "__main__":
    unittest.main()
