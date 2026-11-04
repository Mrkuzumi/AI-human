import csv
import json
import math
import struct
import sys
import tempfile
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))

from motion_bridge import (  # noqa: E402
    BODY_BONES,
    BodySolver,
    FaceCsvLogger,
    FaceSolver,
    NEUTRAL_BLENDS,
    VmcSender,
    is_primary_frame,
    osc_message,
    poll_motion_frames,
    quaternion_multiply,
)
from motion_packet import Detection, MotionFrame, pack_frame  # noqa: E402


def _osc_string(data, offset):
    end = data.index(0, offset)
    value = data[offset:end].decode("utf-8")
    return value, (end + 4) & ~3


def _decode_osc(data):
    address, offset = _osc_string(data, 0)
    tags, offset = _osc_string(data, offset)
    values = []
    for tag in tags[1:]:
        if tag == "s":
            value, offset = _osc_string(data, offset)
        elif tag == "f":
            value = struct.unpack_from(">f", data, offset)[0]
            offset += 4
        elif tag == "i":
            value = struct.unpack_from(">i", data, offset)[0]
            offset += 4
        values.append(value)
    return address, tags, values


def _standing_pose():
    # Anatomical left is on the viewer's right; the default solver mirrors it for Unity.
    xy = (
        (0.50, 0.15), (0.53, 0.14), (0.47, 0.14), (0.56, 0.15), (0.44, 0.15),
        (0.58, 0.30), (0.42, 0.30), (0.68, 0.30), (0.32, 0.30),
        (0.78, 0.30), (0.22, 0.30), (0.55, 0.55), (0.45, 0.55),
        (0.55, 0.75), (0.45, 0.75), (0.55, 0.95), (0.45, 0.95),
    )
    return Detection(0.99, (0.2, 0.1, 0.6, 0.85), tuple((x, y, 0.99) for x, y in xy))


def _wide_frame_detection(detection):
    ratio = 9.0 / 16.0
    return Detection(
        detection.score,
        (detection.box[0] * ratio, detection.box[1], detection.box[2] * ratio, detection.box[3]),
        tuple((x * ratio, y, confidence) for x, y, confidence in detection.keypoints),
    )


class _QueuedReceiver:
    def __init__(self, packets):
        self.packets = list(packets)

    def recvfrom(self, _size):
        if not self.packets:
            raise BlockingIOError
        return self.packets.pop(0), ("192.168.126.10", 39540)


def _pose3d(yaw, left_arm="down", left_leg="down", visibility=0.99, swapped=False):
    radians = math.radians(yaw)
    right = (-math.cos(radians), 0.0, -math.sin(radians))
    up = (0.0, -1.0, 0.0)
    forward = (math.sin(radians), 0.0, -math.cos(radians))

    def camera(point):
        x, y, z = point
        return tuple(
            right[index] * x + up[index] * y + forward[index] * z
            for index in range(3)
        ) + (visibility,)

    local = [
        (0.0, 0.0, 0.0),
        (0.15, 0.0, 0.0), (0.15, -0.38, 0.0), (0.15, -0.76, 0.0),
        (-0.15, 0.0, 0.0), (-0.15, -0.38, 0.0), (-0.15, -0.76, 0.0),
        (0.0, 0.25, 0.0), (0.0, 0.50, 0.0), (0.0, 0.50, 0.0),
        (0.0, 0.72, 0.04),
        (-0.25, 0.54, 0.0), (-0.25, 0.24, 0.0), (-0.25, -0.06, 0.0),
        (0.25, 0.54, 0.0), (0.25, 0.24, 0.0), (0.25, -0.06, 0.0),
    ]
    if left_arm == "up":
        local[12], local[13] = (-0.25, 0.84, 0.0), (-0.25, 1.14, 0.0)
    if left_leg == "forward":
        local[5], local[6] = (-0.15, -0.34, 0.20), (-0.15, -0.68, 0.40)
    points = [camera(point) for point in local]
    if swapped:
        for left, right_index in ((1, 4), (2, 5), (3, 6), (11, 14), (12, 15), (13, 16)):
            points[left], points[right_index] = points[right_index], points[left]
    return tuple(points)


def _face(eye_height=0.01, mouth_gap=0.01, smile_width=0.16):
    points = [(0.5, 0.5, 0.99) for _ in range(51)]
    for start, left in ((0, 0.40), (5, 0.52)):
        points[start : start + 5] = [(left + offset, 0.34, 0.99) for offset in (0.0, 0.02, 0.04, 0.06, 0.08)]
    for start, left in ((19, 0.40), (25, 0.52)):
        points[start : start + 6] = [
            (left, 0.40, 0.99), (left + 0.02, 0.40 - eye_height, 0.99),
            (left + 0.06, 0.40 - eye_height, 0.99), (left + 0.08, 0.40, 0.99),
            (left + 0.06, 0.40 + eye_height, 0.99), (left + 0.02, 0.40 + eye_height, 0.99),
        ]
    points[31] = (0.50 - smile_width / 2.0, 0.62, 0.99)
    points[37] = (0.50 + smile_width / 2.0, 0.62, 0.99)
    points[43], points[47] = (0.44, 0.62, 0.99), (0.56, 0.62, 0.99)
    for upper, lower, x in ((44, 50, 0.47), (45, 49, 0.50), (46, 48, 0.53)):
        points[upper] = (x, 0.62 - mouth_gap / 2.0, 0.99)
        points[lower] = (x, 0.62 + mouth_gap / 2.0, 0.99)
    return Detection(0.99, (0.30, 0.20, 0.40, 0.55), tuple(points))


def _rotate_x_axis(quaternion, sign=1.0):
    x, y, z, w = quaternion
    # Rotate the arm's bind direction (+X for right, -X for left).
    return (
        sign * (1.0 - 2.0 * (y * y + z * z)),
        sign * (2.0 * (x * y + w * z)),
        sign * (2.0 * (x * z - w * y)),
    )


def _z_angle(quaternion):
    return math.degrees(2.0 * math.atan2(quaternion[2], quaternion[3]))


def _y_angle(quaternion):
    return math.degrees(2.0 * math.atan2(quaternion[1], quaternion[3]))


def _rotate_vector(quaternion, vector):
    x, y, z, w = quaternion
    vx, vy, vz = vector
    tx, ty, tz = 2.0 * (y * vz - z * vy), 2.0 * (z * vx - x * vz), 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def _arm_projection_error(solved, points, side, width=1, height=1):
    rotation = (0.0, 0.0, 0.0, 1.0)
    for bone in ("Hips", "Spine", "Chest", "UpperChest", side + "Shoulder"):
        rotation = quaternion_multiply(rotation, solved[bone])
    indices = (5, 7, 9) if side == "Left" else (6, 8, 10)
    errors = []
    for segment, a, b in zip(("UpperArm", "LowerArm"), indices, indices[1:]):
        rotation = quaternion_multiply(rotation, solved[side + segment])
        actual = _rotate_x_axis(rotation, -1.0 if side == "Left" else 1.0)
        expected = (-(points[b][0] - points[a][0]) * width, -(points[b][1] - points[a][1]) * height)
        delta = math.degrees(math.atan2(actual[1], actual[0]) - math.atan2(expected[1], expected[0]))
        errors.append(abs((delta + 180) % 360 - 180))
    return max(errors)


class OscEncodingTests(unittest.TestCase):
    def test_vmc_keeps_eyes_open_but_preserves_other_face_controls(self):
        sender = VmcSender("127.0.0.1", 39539)
        self.addCleanup(sender.socket.close)
        sent = []
        sender._send = lambda address, *values: sent.append((address, values))
        sender.send((), {"Blink_L": 0.9, "Blink_R": 0.7, "A": 0.6, "BrowUp": 0.4})
        blends = {values[0]: values[1] for address, values in sent if address == "/VMC/Ext/Blend/Val"}
        self.assertEqual(blends["Blink_L"], 0.0)
        self.assertEqual(blends["Blink_R"], 0.0)
        self.assertAlmostEqual(blends["A"], 0.6)
        self.assertAlmostEqual(blends["BrowUp"], 0.4)

    def test_only_track_one_is_primary(self):
        self.assertTrue(is_primary_frame(MotionFrame(1, 10, 20, track_id=1)))
        self.assertFalse(is_primary_frame(MotionFrame(1, 10, 20, track_id=2)))

    def test_vmc_bone_message_has_correct_types_and_alignment(self):
        packet = osc_message("/VMC/Ext/Bone/Pos", "Head", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        address, tags, values = _decode_osc(packet)
        self.assertEqual(address, "/VMC/Ext/Bone/Pos")
        self.assertEqual(tags, ",sfffffff")
        self.assertEqual(values[0], "Head")
        self.assertAlmostEqual(values[-1], 1.0)
        self.assertEqual(len(packet) % 4, 0)

    def test_status_message_uses_osc_int(self):
        self.assertEqual(_decode_osc(osc_message("/VMC/Ext/OK", 1)), ("/VMC/Ext/OK", ",i", [1]))

    def test_motion_receiver_drains_queue_and_keeps_latest_capture(self):
        receiver = _QueuedReceiver((
            pack_frame(MotionFrame(1, 300, 310, track_id=1)),
            pack_frame(MotionFrame(2, 300, 310, track_id=2)),
            b"invalid",
            pack_frame(MotionFrame(3, 250, 260, track_id=1)),
        ))
        frame, source, accepted, rejected, ignored = poll_motion_frames(receiver)
        self.assertEqual(frame.sequence, 1)
        self.assertEqual(source, ("192.168.126.10", 39540))
        self.assertEqual((accepted, rejected, ignored), (2, 1, 1))


class BodySolverTests(unittest.TestCase):
    def test_wide_frame_angles_match_square_pixel_geometry(self):
        pose = _standing_pose()
        expected = dict(BodySolver(smoothing=1.0).solve(pose, frame_width=1, frame_height=1))
        actual = dict(BodySolver(smoothing=1.0).solve(
            _wide_frame_detection(pose), frame_width=1920, frame_height=1080
        ))
        for bone in BODY_BONES:
            for actual_value, expected_value in zip(actual[bone], expected[bone]):
                self.assertAlmostEqual(actual_value, expected_value)

    def test_head_touch_boundary_is_continuous_on_either_side(self):
        for side, elbow, wrist, sign in (("Left", 7, 9, 1.0), ("Right", 8, 10, -1.0)):
            previous = None
            max_step = 0.0
            for i in range(121):
                base = _standing_pose()
                points = list(base.keypoints)
                points[7], points[9] = (0.58, 0.48, 0.99), (0.58, 0.65, 0.99)
                points[8], points[10] = (0.42, 0.48, 0.99), (0.42, 0.65, 0.99)
                points[elbow] = (0.50 + sign * 0.20, 0.25, 0.99)
                points[wrist] = (0.50 + sign * (0.11 - i * 0.0008), 0.10, 0.99)
                solved = dict(BodySolver(smoothing=1.0).solve(Detection(base.score, base.box, tuple(points))))
                current = tuple(_z_angle(solved[side + segment]) for segment in ("UpperArm", "LowerArm"))
                if previous is not None:
                    max_step = max(max_step, *(abs(a - b) for a, b in zip(previous, current)))
                previous = current
            self.assertLess(max_step, 5.0, side)

    def test_clasp_distance_boundary_is_continuous(self):
        previous = None
        max_step = 0.0
        for i in range(101):
            base = _standing_pose()
            points = list(base.keypoints)
            gap = 0.14 - i * 0.0008
            points[7], points[9] = (0.64, 0.48, 0.99), (0.50 + gap / 2.0, 0.58, 0.99)
            points[8], points[10] = (0.49, 0.35, 0.99), (0.50 - gap / 2.0, 0.58, 0.99)
            solved = dict(BodySolver(smoothing=1.0).solve(Detection(base.score, base.box, tuple(points))))
            current = tuple(_z_angle(solved[side + segment]) for side in ("Left", "Right")
                            for segment in ("UpperArm", "LowerArm"))
            if previous is not None:
                max_step = max(max_step, *(abs(a - b) for a, b in zip(previous, current)))
            previous = current
        self.assertLess(max_step, 5.0)

    def test_front_facing_t_pose_stays_forward_without_rotating_hips(self):
        solved = dict(BodySolver(smoothing=1.0).solve(_standing_pose()))
        self.assertEqual(tuple(solved), BODY_BONES)
        self.assertEqual(solved["Hips"], (0.0, 0.0, 0.0, 1.0))
        self.assertGreater(_rotate_x_axis(solved["LeftUpperArm"], -1.0)[2], 0.0)
        self.assertGreater(_rotate_x_axis(solved["RightUpperArm"])[2], 0.0)

    def test_missing_body_uses_natural_idle_pose(self):
        solved = dict(BodySolver(smoothing=1.0).solve(None))
        left = _rotate_x_axis(solved["LeftUpperArm"], -1.0)
        right = _rotate_x_axis(solved["RightUpperArm"])
        self.assertLess(left[1], -0.9)
        self.assertLess(right[1], -0.9)
        self.assertGreater(left[2], 0.0)
        self.assertGreater(right[2], 0.0)

    def test_short_occluded_arm_segments_fall_back_to_idle(self):
        pose = _standing_pose()
        points = list(pose.keypoints)
        points[7] = (points[5][0] + 0.001, points[5][1], 0.99)
        points[9] = (points[7][0] + 0.001, points[7][1], 0.99)
        solved = dict(BodySolver(smoothing=1.0).solve(Detection(pose.score, pose.box, tuple(points))))
        self.assertGreater(_z_angle(solved["LeftUpperArm"]), 70.0)

    def test_elbows_can_bend_upward_in_the_image_plane(self):
        pose = _standing_pose()
        points = list(pose.keypoints)
        points[7] = (0.68, 0.30, 0.99)
        points[9] = (0.68, 0.20, 0.99)
        points[8] = (0.32, 0.30, 0.99)
        points[10] = (0.32, 0.20, 0.99)
        solved = dict(BodySolver(smoothing=1.0).solve(Detection(pose.score, pose.box, tuple(points))))
        self.assertLess(_z_angle(solved["LeftLowerArm"]), -80.0)
        self.assertGreater(_z_angle(solved["RightLowerArm"]), 80.0)

    def test_asymmetric_clasp_keeps_each_measured_elbow(self):
        pose = _standing_pose()
        points = list(pose.keypoints)
        points[7], points[9] = (0.64, 0.48, 0.99), (0.52, 0.58, 0.99)
        points[8], points[10] = (0.49, 0.35, 0.99), (0.48, 0.58, 0.99)
        solved = dict(BodySolver(smoothing=1.0).solve(
            Detection(pose.score, pose.box, tuple(points))
        ))
        for side in ("Left", "Right"):
            self.assertLess(_arm_projection_error(solved, points, side), 1e-6)

    def test_hand_at_head_keeps_measured_segment_directions(self):
        pose = _standing_pose()
        points = list(pose.keypoints)
        points[7], points[9] = (0.58, 0.48, 0.99), (0.58, 0.65, 0.01)
        points[8], points[10] = (0.47, 0.20, 0.99), (0.50, 0.12, 0.99)
        solved = dict(BodySolver(smoothing=1.0).solve(
            Detection(pose.score, pose.box, tuple(points))
        ))
        self.assertGreater(_z_angle(solved["LeftUpperArm"]), 80.0)
        self.assertLess(_arm_projection_error(solved, points, "Right"), 1e-6)

    def test_either_bent_hand_at_head_keeps_its_own_pose(self):
        for side in ("Left", "Right"):
            with self.subTest(side=side):
                pose = _standing_pose()
                points = list(pose.keypoints)
                points[7], points[9] = (0.58, 0.48, 0.99), (0.58, 0.65, 0.99)
                points[8], points[10] = (0.42, 0.48, 0.99), (0.42, 0.65, 0.99)
                if side == "Left":
                    points[7], points[9] = (0.70, 0.25, 0.99), (0.51, 0.10, 0.99)
                else:
                    points[8], points[10] = (0.30, 0.25, 0.99), (0.49, 0.10, 0.99)
                solved = dict(BodySolver(smoothing=1.0).solve(
                    Detection(pose.score, pose.box, tuple(points))
                ))
                self.assertLess(_arm_projection_error(solved, points, side), 1e-6)

    def test_arm_forward_kinematics_matches_image_with_torso_roll_and_depth(self):
        for depth in (0, 10, 60):
            for upper in (-155, -100, -30, 0, 65, 110, 160):
                for lower in (-155, -80, 0, 80, 155):
                    base = _standing_pose()
                    points = list(base.keypoints)
                    points[5], points[6] = (.62, .25, .99), (.38, .35, .99)
                    for shoulder, elbow, wrist, sign in ((5, 7, 9, -1), (6, 8, 10, 1)):
                        for a, b, angle in ((shoulder, elbow, upper), (elbow, wrist, upper + lower)):
                            r = math.radians(angle)
                            points[b] = (points[a][0] - sign * .12 * math.cos(r),
                                         points[a][1] - sign * .12 * math.sin(r), .99)
                    solved = dict(BodySolver(smoothing=1, arm_forward=depth).solve(
                        Detection(base.score, base.box, tuple(points))))
                    for side in ("Left", "Right"):
                        self.assertLess(_arm_projection_error(solved, points, side), 1e-6,
                                        (side, depth, upper, lower))

    def test_left_arm_motion_changes_left_upper_arm(self):
        pose = _standing_pose()
        points = list(pose.keypoints)
        points[7] = (0.58, 0.20, 0.99)
        points[9] = (0.58, 0.10, 0.99)
        solved = dict(BodySolver(smoothing=1.0).solve(Detection(pose.score, pose.box, tuple(points))))
        self.assertGreater(abs(solved["LeftUpperArm"][2]), math.sin(math.radians(20)))
        self.assertGreater(abs(solved["LeftUpperArm"][2]), abs(solved["RightUpperArm"][2]))

    def test_3d_depth_distinguishes_arms_in_front_from_behind(self):
        front = [[0.0, 0.0, 0.0] for _ in range(17)]
        behind = [[0.0, 0.0, 0.0] for _ in range(17)]
        for shoulder, elbow, wrist, direction in ((11, 12, 13, 1.0), (14, 15, 16, -1.0)):
            front[elbow] = [direction, 0.0, -1.0]
            front[wrist] = [direction * 2.0, 0.0, -2.0]
            behind[elbow] = [direction, 0.0, 1.0]
            behind[wrist] = [direction * 2.0, 0.0, 2.0]

        front_pose = dict(BodySolver(smoothing=1.0, arm_depth_calibration_frames=0).solve(_standing_pose(), front))
        behind_pose = dict(BodySolver(smoothing=1.0, arm_depth_calibration_frames=0).solve(_standing_pose(), behind))
        self.assertGreater(_rotate_x_axis(front_pose["LeftUpperArm"], -1.0)[2], 0.0)
        self.assertGreater(_rotate_x_axis(front_pose["RightUpperArm"])[2], 0.0)
        self.assertLess(_rotate_x_axis(behind_pose["LeftUpperArm"], -1.0)[2], 0.0)
        self.assertLess(_rotate_x_axis(behind_pose["RightUpperArm"])[2], 0.0)

    def test_3d_depth_survives_short_detection_gaps(self):
        pose3d = [[0.0, 0.0, 0.0] for _ in range(17)]
        pose3d[12] = [1.0, 0.0, 1.0]
        pose3d[13] = [2.0, 0.0, 2.0]
        pose3d[15] = [-1.0, 0.0, 1.0]
        pose3d[16] = [-2.0, 0.0, 2.0]
        solver = BodySolver(smoothing=1.0, arm_depth_calibration_frames=0)
        solver.solve(_standing_pose(), pose3d)
        before = solver.arm_depths
        solver.solve(None)
        self.assertEqual(solver.arm_depths, before)

    def test_3d_depth_calibration_removes_neutral_bias(self):
        neutral = [[0.0, 0.0, 0.0] for _ in range(17)]
        front = [[0.0, 0.0, 0.0] for _ in range(17)]
        for shoulder, elbow, wrist, direction in ((11, 12, 13, 1.0), (14, 15, 16, -1.0)):
            neutral[elbow] = [direction, 0.0, 0.5]
            neutral[wrist] = [direction * 2.0, 0.0, 1.0]
            front[elbow] = [direction, 0.0, -0.5]
            front[wrist] = [direction * 2.0, 0.0, -1.0]

        solver = BodySolver(smoothing=1.0, arm_depth_calibration_frames=2)
        solver.solve(_standing_pose(), neutral)
        solver.solve(_standing_pose(), neutral)
        self.assertEqual(solver.arm_depths, (10.0, 10.0))
        solver.solve(_standing_pose(), front)
        self.assertGreater(solver.arm_depths[0], 10.0)
        self.assertGreater(solver.arm_depths[1], 10.0)

    def test_3d_torso_yaw_drives_hips_in_both_directions(self):
        for yaw in (0.0, 45.0, 90.0, -90.0, 135.0, 179.0):
            with self.subTest(yaw=yaw):
                solver = BodySolver(
                    smoothing=1.0,
                    turn_smoothing=1.0,
                    turn_max_speed=10_000.0,
                    arm_depth_calibration_frames=0,
                )
                solved = dict(solver.solve(_standing_pose(), _pose3d(yaw), 1, 1_000_000_000))
                self.assertAlmostEqual(_y_angle(solved["Hips"]), yaw, delta=0.1)

    def test_body_yaw_unwraps_across_the_back_without_snapping(self):
        solver = BodySolver(turn_smoothing=1.0, turn_max_speed=1_000.0)
        for sequence, yaw in enumerate((0.0, 45.0, 90.0, 135.0, 179.0, -140.0), 1):
            solved = dict(solver.solve(
                _standing_pose(), _pose3d(yaw), sequence, sequence * 100_000_000
            ))
            self.assertAlmostEqual(_y_angle(solved["Hips"]), yaw, delta=0.1)

    def test_body_yaw_rejects_a_left_right_label_flip(self):
        solver = BodySolver(
            smoothing=1.0,
            turn_smoothing=1.0,
            turn_max_speed=10_000.0,
            arm_depth_calibration_frames=0,
        )
        solver.solve(_standing_pose(), _pose3d(90.0, left_arm="up"), 1, 100_000_000)
        solved = dict(solver.solve(
            _standing_pose(), _pose3d(90.0, left_arm="up", swapped=True), 2, 150_000_000
        ))
        self.assertAlmostEqual(_y_angle(solved["Hips"]), 90.0, delta=0.1)
        self.assertGreater(_rotate_x_axis(solved["LeftUpperArm"], -1.0)[1], 0.9)

    def test_side_view_uses_body_local_3d_arms_and_legs(self):
        down = BodySolver(
            smoothing=1.0, turn_smoothing=1.0, turn_max_speed=10_000.0,
            arm_depth_calibration_frames=0,
        )
        raised = BodySolver(
            smoothing=1.0, turn_smoothing=1.0, turn_max_speed=10_000.0,
            arm_depth_calibration_frames=0,
        )
        down_pose = dict(down.solve(_standing_pose(), _pose3d(90.0), 1, 100_000_000))
        raised_pose = dict(raised.solve(
            _standing_pose(), _pose3d(90.0, left_arm="up", left_leg="forward"),
            1, 100_000_000,
        ))
        self.assertLess(_rotate_x_axis(down_pose["LeftUpperArm"], -1.0)[1], -0.9)
        self.assertGreater(_rotate_x_axis(raised_pose["LeftUpperArm"], -1.0)[1], 0.9)
        self.assertGreater(_rotate_vector(raised_pose["LeftUpperLeg"], (0.0, -1.0, 0.0))[2], 0.2)

    def test_low_visibility_turn_is_ignored_and_missing_pose_returns_forward(self):
        solver = BodySolver(turn_smoothing=1.0, turn_max_speed=10_000.0, turn_hold_frames=2)
        low = dict(solver.solve(_standing_pose(), _pose3d(90.0, visibility=0.1), 1, 100_000_000))
        self.assertAlmostEqual(_y_angle(low["Hips"]), 0.0, delta=0.1)
        solver.solve(_standing_pose(), _pose3d(90.0), 2, 200_000_000)
        solver.solve(_standing_pose(), None, -1, 0)
        solver.solve(_standing_pose(), None, -1, 0)
        returned = dict(solver.solve(_standing_pose(), None, -1, 0))
        self.assertAlmostEqual(_y_angle(returned["Hips"]), 0.0, delta=0.1)


class FaceSolverTests(unittest.TestCase):
    def test_neutral_calibration_then_blink_and_mouth(self):
        solver = FaceSolver(calibration_frames=2, smoothing=1.0)
        solver.solve(_face())
        self.assertEqual(solver.solve(_face()), NEUTRAL_BLENDS)
        changed = _face(eye_height=0.002, mouth_gap=0.10, smile_width=0.20)
        for _ in range(4):
            values = solver.solve(changed)
        for name in ("Blink_L", "Blink_R", "A"):
            self.assertGreater(values[name], 0.5)
        self.assertEqual(values["Joy"], 0.0)

    def test_smile_width_does_not_drive_unsupported_joy_expression(self):
        solver = FaceSolver(calibration_frames=1, smoothing=1.0)
        solver.solve(_face())
        for _ in range(4):
            values = solver.solve(_face(smile_width=0.30))
        self.assertEqual(values["Joy"], 0.0)

    def test_nose_offset_does_not_erase_valid_eye_and_lip_measurements(self):
        solver = FaceSolver(calibration_frames=1, smoothing=1.0)
        solver.solve(_face())
        for _ in range(4):
            solver.solve(_face(eye_height=0.0, mouth_gap=0.10))

        profile = _face(eye_height=0.0, mouth_gap=0.10)
        points = list(profile.keypoints)
        points[13] = (0.58, points[13][1], points[13][2])
        values = solver.solve(Detection(profile.score, profile.box, tuple(points)))

        self.assertGreater(values["A"], .5)
        self.assertGreater(values["Blink_L"], .5)
        self.assertFalse(solver.diagnostics["profile_suppressed"])
        self.assertTrue(solver.diagnostics["profile_detected"])
        self.assertGreater(abs(solver.diagnostics["yaw_proxy"]), FaceSolver.PROFILE_YAW_THRESHOLD)
        self.assertTrue(solver.diagnostics["measurement_valid"])
        self.assertIsNotNone(solver.diagnostics["mouth_open_ratio"])

    def test_brow_eye_distances_are_recorded_for_later_avatar_mapping(self):
        solver = FaceSolver(calibration_frames=1, smoothing=1.0)
        solver.solve(_face())
        diagnostics = solver.diagnostics
        self.assertGreater(diagnostics["left_brow_eye_ratio"], 0.0)
        self.assertGreater(diagnostics["right_brow_eye_ratio"], 0.0)

    def test_diagnostics_expose_raw_ratios_targets_and_outputs(self):
        solver = FaceSolver(calibration_frames=1, smoothing=1.0)
        solver.solve(_face())
        changed = _face(eye_height=0.002, mouth_gap=0.10)
        for _ in range(4):
            values = solver.solve(changed)
        diagnostics = solver.diagnostics
        self.assertTrue(diagnostics["measurement_valid"])
        self.assertAlmostEqual(diagnostics["min_required_conf"], 0.99)
        self.assertAlmostEqual(diagnostics["target_blink_l"], values["Blink_L"])
        self.assertAlmostEqual(diagnostics["target_a"], values["A"])

    def test_calibration_skips_collapsed_open_eye_ratio(self):
        solver = FaceSolver(calibration_frames=2, smoothing=1.0)
        solver.solve(_face(eye_height=0.0, mouth_gap=0.0))
        self.assertEqual(solver.calibration_progress[0], 0)
        solver.solve(_face())
        solver.solve(_face())
        self.assertEqual(solver.calibration_progress, (2, 2))
        self.assertGreater(solver.diagnostics["base_left_eye"], 0.0)

    def test_one_collapsed_eye_frame_does_not_trigger_blink(self):
        solver = FaceSolver(calibration_frames=1, smoothing=1.0)
        solver.solve(_face())
        values = solver.solve(_face(eye_height=0.0))
        self.assertEqual(values["Blink_L"], 0.0)
        self.assertEqual(values["Blink_R"], 0.0)

    def test_bilateral_blink_still_requires_persistent_evidence(self):
        solver = FaceSolver(calibration_frames=1, smoothing=1.0)
        solver.solve(_face())
        closed = _face(eye_height=0.0)
        solver.solve(closed)
        solver.solve(closed)
        values = solver.solve(closed)
        self.assertGreater(values["Blink_L"], 0.5)
        self.assertEqual(values["Blink_L"], values["Blink_R"])

    def test_zero_mouth_baseline_stays_closed_but_large_gap_opens(self):
        solver = FaceSolver(calibration_frames=1, smoothing=1.0)
        solver.solve(_face(mouth_gap=0.0))
        solver.solve(_face(mouth_gap=0.015))
        self.assertEqual(solver.solve(_face(mouth_gap=0.015))["A"], 0.0)
        for _ in range(4):
            values = solver.solve(_face(mouth_gap=0.10))
        self.assertGreater(values["A"], 0.5)

    def test_transient_missing_face_holds_active_expression(self):
        solver = FaceSolver(calibration_frames=1, smoothing=1.0)
        solver.solve(_face())
        changed = _face(eye_height=0.0, mouth_gap=0.10)
        for _ in range(4):
            active = solver.solve(changed)
        self.assertGreater(active["Blink_L"], 0.5)
        self.assertGreater(active["A"], 0.5)
        self.assertEqual(solver.solve(None), active)

    def test_diagnostics_mark_missing_face_without_changing_solver_behavior(self):
        solver = FaceSolver(calibration_frames=1, smoothing=1.0)
        solver.solve(_face())
        self.assertEqual(solver.solve(None), NEUTRAL_BLENDS)
        self.assertFalse(solver.diagnostics["face_present"])
        self.assertFalse(solver.diagnostics["measurement_valid"])

    def test_wink_is_independent_and_bad_mouth_does_not_block_eye_calibration(self):
        solver = FaceSolver(calibration_frames=2, smoothing=1)
        def bad_mouth(left_closed=False):
            face = _face()
            p = list(face.keypoints)
            p[44] = (*p[44][:2], .01)
            if left_closed:
                for i in (26, 27, 29, 30):
                    p[i] = (p[i][0], p[25][1], .99)
            return Detection(face.score, face.box, tuple(p))
        for _ in range(2):
            solver.solve(bad_mouth())
        for _ in range(4):
            values = solver.solve(bad_mouth(True))
        self.assertGreater(values["Blink_L"], .9)
        self.assertEqual(values["Blink_R"], 0)
        self.assertEqual(values["A"], 0)
        self.assertEqual(solver.diagnostics["left_eye_calibration"], 2)
        self.assertEqual(solver.diagnostics["mouth_calibration"], 0)

    def test_bad_eye_does_not_block_mouth(self):
        solver = FaceSolver(calibration_frames=1, smoothing=1)
        solver.solve(_face())
        face = _face(mouth_gap=.10)
        p = list(face.keypoints)
        p[19] = (*p[19][:2], .01)
        for _ in range(4):
            values = solver.solve(Detection(face.score, face.box, tuple(p)))
        self.assertGreater(values["A"], .5)
        self.assertFalse(solver.diagnostics["right_eye_valid"])
        self.assertTrue(solver.diagnostics["mouth_valid"])

    def test_brow_height_drives_only_the_real_brow_controls(self):
        solver = FaceSolver(calibration_frames=1, smoothing=1)
        solver.solve(_face())
        face = _face()
        p = list(face.keypoints)
        for i in range(10):
            p[i] = (p[i][0], p[i][1] - .06, p[i][2])
        for _ in range(4):
            values = solver.solve(Detection(face.score, face.box, tuple(p)))
        self.assertGreater(values["BrowUp"], .9)
        self.assertEqual(values["BrowDown"], 0)
        self.assertEqual(values["A"], 0)
        self.assertEqual(values["Blink_L"], 0)

    def test_eye_and_mouth_measurements_are_invariant_to_head_roll(self):
        face = _face(mouth_gap=.08)
        solver = FaceSolver(calibration_frames=1)
        original = solver._measure(face, solver._scaled_points(face, 1920, 1080), 1920)
        c, s = math.cos(.5), math.sin(.5)
        rotated = tuple(((x * 1920 * c - y * 1080 * s) / 1920,
                         (x * 1920 * s + y * 1080 * c) / 1080, conf)
                        for x, y, conf in face.keypoints)
        changed = Detection(face.score, face.box, rotated)
        result = solver._measure(changed, solver._scaled_points(changed, 1920, 1080), 1920)
        for expected, actual in zip(original, result):
            self.assertAlmostEqual(expected, actual)

    def test_face_csv_logger_writes_one_complete_row(self):
        solver = FaceSolver(calibration_frames=1, smoothing=1.0)
        face = _face()
        solver.solve(face)
        frame = MotionFrame(7, 100, 200, face=face)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "nested" / "face.csv"
            logger = FaceCsvLogger(str(path))
            logger.write(frame, "192.168.126.49", solver.diagnostics)
            logger.close()
            with path.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["sequence"], "7")
        self.assertEqual(rows[0]["source_ip"], "192.168.126.49")
        self.assertEqual(rows[0]["measurement_valid"], "True")
        self.assertEqual(rows[0]["profile_suppressed"], "False")
        self.assertNotEqual(rows[0]["left_brow_eye_ratio"], "")
        self.assertEqual(len(json.loads(rows[0]["face_keypoints_json"])), 51)


if __name__ == "__main__":
    unittest.main()
