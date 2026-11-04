"""Torso yaw must not rotate the established camera-space limb targets twice."""
import math
import unittest

from test_motion_bridge import _pose3d, _standing_pose
from test_wrist_depth_mapping import world_hand
from motion_packet import Detection
from motion_bridge import (
    BodySolver, FingerSolver, matched_body_turn_pose, matched_arm_depth_pose,
    quaternion_multiply as mul, quaternion_rotate as rotate, quaternion_y as qy,
    quaternion_x as qx, _wrap_degrees,
)


def board_pose(pose, mirror=True):
    points = list(_standing_pose().keypoints)
    for a, b in ((5, 11), (7, 12), (9, 13), (6, 14), (8, 15), (10, 16),
                 (11, 4), (13, 5), (15, 6), (12, 1), (14, 2), (16, 3)):
        x, y, _, c = pose[b]
        points[a] = (.5 + .35*x, .55 + .35*y, c)
    points[1] = (.5, .15, 0.)
    points[2] = (.5, .15, 0.)
    return Detection(.99, (.1, .05, .8, .95), tuple(points))


def observation(body, pose):
    image = [(0., 0., 0., .99)] * 17
    for a, b in ((5, 11), (7, 12), (9, 13), (6, 14), (8, 15), (10, 16), (11, 4), (12, 1)):
        image[b] = (*body.keypoints[a][:2], 0., .99)
    return dict(points=pose, image_points=image)


def solver(**kwargs):
    return BodySolver(smoothing=1, turn_smoothing=1, turn_max_speed=10000,
                      arm_depth_only=True, conservative_body_turn=True, **kwargs)


class BodyTurnMappingTests(unittest.TestCase):
    def test_front_facing_and_no_rgb_preserve_current_mapping(self):
        body, pose = _standing_pose(), _pose3d(0)
        baseline = BodySolver(arm_depth_only=True)
        new = BodySolver(arm_depth_only=True, conservative_body_turn=True)
        for i in range(40):
            points = pose if i > 10 else None
            self.assertEqual(baseline.solve(body, points), new.solve(body, points, turn_pose3d=points))

    def test_full_turn_both_mirror_modes_with_camera_space_arms_and_legs(self):
        for mirror in (True, False):
            subject = solver(mirror=mirror)
            for i, yaw in enumerate(range(0, 361, 10), 1):
                pose = _pose3d(yaw, left_arm="up")
                body = board_pose(pose)
                result = dict(subject.solve(body, pose, i, i*100_000_000, turn_pose3d=pose))
                expected_yaw = -yaw if mirror else yaw
                self.assertLess(abs(_wrap_degrees(subject.body_turn_status[0]-expected_yaw)), .01)
                self.assertLess(math.dist(rotate(result["Hips"], (0, 0, 1)),
                                          rotate(qy(expected_yaw), (0, 0, 1))), 1e-8)
                baseline = dict(BodySolver(smoothing=1, arm_depth_only=True, mirror=mirror).solve(body, pose))
                for side in ("Left", "Right"):
                    for limb, axis in (("Arm", (-1 if side == "Left" else 1, 0, 0)), ("Leg", (0, -1, 0))):
                        upper, lower = side+"Upper"+limb, side+"Lower"+limb
                        for end in (upper, lower):
                            parent, base_parent = result["Hips"], baseline["Hips"]
                            if limb == "Arm":
                                for name in ("Spine", "Chest", "UpperChest", side+"Shoulder"):
                                    parent, base_parent = mul(parent, result[name]), mul(base_parent, baseline[name])
                            actual = mul(parent, result[upper])
                            target = mul(base_parent, baseline[upper])
                            if end == lower:
                                actual, target = mul(actual, result[lower]), mul(target, baseline[lower])
                            self.assertLess(math.dist(rotate(actual, axis), rotate(target, axis)), 1e-7,
                                            (mirror, yaw, end))
                            if limb == "Arm":
                                # Full parent frame protects the existing planar
                                # wrist bend, including when RGB hands disappear.
                                self.assertLess(math.dist(rotate(actual, (0, 0, 1)),
                                                          rotate(target, (0, 0, 1))), 1e-7)

    def test_torso_association_does_not_require_elbows_or_wrists(self):
        pose = list(_pose3d(80))
        for i in (12, 13, 15, 16):
            pose[i] = (*pose[i][:3], 0.)
        body = board_pose(pose)
        obs = observation(body, pose)
        self.assertIsNone(matched_arm_depth_pose(body, obs, 1080, 1080, .25))
        self.assertIsNotNone(matched_body_turn_pose(body, obs, 1080, 1080, .25))
        subject = solver()
        subject.solve(body, None, 1, 100000000, turn_pose3d=pose)
        self.assertAlmostEqual(subject.body_turn_status[0], -80)
        obs["image_points"] = [(p[0]+.4, *p[1:]) for p in obs["image_points"]]
        self.assertIsNone(matched_body_turn_pose(body, obs, 1080, 1080, .25))

    def test_back_facing_torso_does_not_lean_sideways_and_missing_rgb_holds_yaw(self):
        subject = solver(turn_hold_frames=2)
        pose = _pose3d(180)
        body = board_pose(pose)
        result = dict(subject.solve(body, pose, 1, 100000000, turn_pose3d=pose))
        for name in ("Spine", "Chest", "Neck", "Head"):
            self.assertLess(math.dist(result[name], (0, 0, 0, 1)), 1e-9)
        for _ in range(50):
            subject.solve(body)
        self.assertAlmostEqual(abs(subject.body_turn_status[0]), 180)
        self.assertEqual(subject.body_turn_status[2], 0)

    def test_one_label_flip_rejected_but_real_fast_turn_reacquires(self):
        subject = solver()
        body = _standing_pose()
        for seq, yaw in enumerate((0, 180, 0, 180, 180, 180), 1):
            pose = _pose3d(yaw)
            subject.solve(body, pose, seq, seq*100000000, turn_pose3d=pose)
            if seq < 6:
                self.assertAlmostEqual(subject.body_turn_status[0], 0)
        self.assertAlmostEqual(abs(subject.body_turn_status[0]), 180)
        # A held duplicate must not satisfy the three-observation condition.
        subject = solver()
        subject.solve(body, turn_pose3d=_pose3d(0), pose_sequence=1)
        for _ in range(20):
            subject.solve(body, turn_pose3d=_pose3d(180), pose_sequence=2)
        self.assertAlmostEqual(subject.body_turn_status[0], 0)

    def test_wrist_edge_uses_actual_turned_parent(self):
        for yaw in (45, 90, 135, 180, -90):
            pose = _pose3d(yaw)
            result = dict(solver().solve(board_pose(pose), pose, turn_pose3d=pose))
            finger = FingerSolver(hand_orientation_only=True)
            for side in ("Left", "Right"):
                parent = (0, 0, 0, 1)
                for name in ("Hips", "Spine", "Chest", "UpperChest", side+"Shoulder", side+"UpperArm", side+"LowerArm"):
                    parent = mul(parent, result[name])
                target = mul(parent, qx(80))
                self.assertAlmostEqual(finger._wrist_twist(world_hand(side, target), parent), 80, places=3)

    def test_nonfinite_torso_rejected(self):
        pose = list(_pose3d(60))
        pose[11] = (float("nan"), *pose[11][1:])
        self.assertIsNone(solver()._estimate_body_frame(pose))


if __name__ == "__main__":
    unittest.main()
