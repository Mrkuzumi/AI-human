"""Bridge DMV1 board motion packets to Virtual Motion Capture Protocol (VMC)."""

from __future__ import annotations

import argparse
import csv
import json
import math
import select
import socket
import statistics
import struct
import sys
import time
from collections import deque
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from finger_packet import FingerFrame, unpack_finger_frame
from gesture_trigger import HeadPatJsonlLogger, HeadPatTrigger
from gesture_packet import GestureFrame, GestureRecord, unpack_frame as unpack_gesture_frame
from motion_packet import Detection, MotionFrame, unpack_frame
from mode_control import ModeControlReceiver


def is_primary_frame(frame: MotionFrame) -> bool:
    return frame.track_id == 1


def _aspect_correct_detection(
    detection: Optional[Detection], frame_width: int, frame_height: int
) -> Optional[Detection]:
    if detection is None or frame_width <= 0 or frame_height <= 0:
        return detection
    scale = frame_width / frame_height
    return Detection(
        detection.score,
        (detection.box[0] * scale, detection.box[1], detection.box[2] * scale, detection.box[3]),
        tuple((point[0] * scale, point[1], point[2]) for point in detection.keypoints),
    )


Quaternion = Tuple[float, float, float, float]
BonePose = Tuple[str, Quaternion]
Pose3D = Sequence[Sequence[float]]

IDLE_ANGLES = {
    "LeftUpperArm": 78.0,
    "LeftLowerArm": 8.0,
    "RightUpperArm": -78.0,
    "RightLowerArm": -8.0,
}

BODY_BONES = (
    "Hips",
    "Spine",
    "Chest",
    "UpperChest",
    "LeftShoulder",
    "RightShoulder",
    "Neck",
    "Head",
    "LeftUpperArm",
    "LeftLowerArm",
    "RightUpperArm",
    "RightLowerArm",
    "LeftUpperLeg",
    "LeftLowerLeg",
    "RightUpperLeg",
    "RightLowerLeg",
)

FINGER_CHAINS = (
    ("Thumb", (1, 2, 3, 4)),
    ("Index", (5, 6, 7, 8)),
    ("Middle", (9, 10, 11, 12)),
    ("Ring", (13, 14, 15, 16)),
    ("Little", (17, 18, 19, 20)),
)

FINGER_BONES = tuple(
    side + finger + segment
    for side in ("Left", "Right")
    for finger, _points in FINGER_CHAINS
    for segment in ("Proximal", "Intermediate", "Distal")
)

HAND_BONES = ("LeftHand", "RightHand")
HAND_OUTPUT_BONES = HAND_BONES + FINGER_BONES


def _osc_string(value: str) -> bytes:
    raw = value.encode("utf-8") + b"\0"
    return raw + b"\0" * (-len(raw) % 4)


def osc_message(address: str, *values: object) -> bytes:
    """Encode the OSC scalar types used by VMC, without third-party packages."""
    tags = [","]
    payload = bytearray()
    for value in values:
        if isinstance(value, str):
            tags.append("s")
            payload.extend(_osc_string(value))
        elif isinstance(value, int):
            tags.append("i")
            payload.extend(struct.pack(">i", value))
        elif isinstance(value, float):
            tags.append("f")
            payload.extend(struct.pack(">f", value))
        else:
            raise TypeError("unsupported OSC value: {!r}".format(value))
    return _osc_string(address) + _osc_string("".join(tags)) + bytes(payload)


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _wrap_degrees(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def quaternion_z(degrees: float) -> Quaternion:
    half = math.radians(degrees) * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quaternion_x(degrees: float) -> Quaternion:
    half = math.radians(degrees) * 0.5
    return math.sin(half), 0.0, 0.0, math.cos(half)


def quaternion_y(degrees: float) -> Quaternion:
    half = math.radians(degrees) * 0.5
    return 0.0, math.sin(half), 0.0, math.cos(half)


def quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quaternion_inverse(rotation: Quaternion) -> Quaternion:
    x, y, z, w = rotation
    norm = sum(value * value for value in rotation)
    return -x / norm, -y / norm, -z / norm, w / norm


def quaternion_rotate(rotation: Quaternion, vector: Sequence[float]) -> Tuple[float, float, float]:
    return quaternion_multiply(
        quaternion_multiply(rotation, (*vector[:3], 0.0)), quaternion_inverse(rotation)
    )[:3]


def arm_quaternion(side: str, roll: float, forward: float) -> Quaternion:
    depth = forward if side == "Left" else -forward
    return quaternion_multiply(quaternion_z(roll), quaternion_y(depth))


def leg_quaternion(roll: float, forward: float) -> Quaternion:
    return quaternion_multiply(quaternion_z(roll), quaternion_x(-forward))


def _vector(first: Sequence[float], second: Sequence[float]) -> Tuple[float, float, float]:
    return tuple(float(second[index]) - float(first[index]) for index in range(3))


def _dot(first: Sequence[float], second: Sequence[float]) -> float:
    return sum(float(first[index]) * float(second[index]) for index in range(3))


def _cross(first: Sequence[float], second: Sequence[float]) -> Tuple[float, float, float]:
    return (
        first[1] * second[2] - first[2] * second[1],
        first[2] * second[0] - first[0] * second[2],
        first[0] * second[1] - first[1] * second[0],
    )


def _unit(vector: Sequence[float]) -> Optional[Tuple[float, float, float]]:
    length = math.sqrt(_dot(vector, vector))
    if length < 1e-6:
        return None
    return tuple(float(axis) / length for axis in vector)


def _angle_blend(first: float, second: float, weight: float) -> float:
    return _wrap_degrees(first + _wrap_degrees(second - first) * weight)


def _smoothstep(value: float) -> float:
    value = _clamp(value, 0.0, 1.0)
    return value * value * (3.0 - 2.0 * value)


def rgb_observation_matches(observation: Optional[dict], frame: MotionFrame,
                            source_host: str, max_skew_ms: float) -> bool:
    if not observation or frame.capture_time_ns <= 0:
        return False
    return (observation["source_id"] == frame.source_id
            and observation["source_host"] == source_host
            and observation["capture_ns"] > 0
            and abs(frame.capture_time_ns - observation["capture_ns"]) <= max_skew_ms * 1_000_000)


def matched_arm_depth_pose(body: Optional[Detection], observation: dict,
                           width: int, height: int, confidence: float) -> Optional[Pose3D]:
    """Reject depth from a different person/posture, independently for each arm."""
    image = observation.get("image_points", ())
    pose = observation.get("points", ())
    if body is None or len(image) != 17 or len(pose) != 17:
        return None
    aspect = width / height if width > 0 and height > 0 else 1.0
    filtered = list(pose)
    valid = 0
    for board, rgb in (((5, 7, 9), (11, 12, 13)), ((6, 8, 10), (14, 15, 16))):
        matches = all(body.keypoints[a][2] >= confidence
                      and len(image[b]) >= 4 and image[b][3] >= .6
                      and math.hypot((body.keypoints[a][0] - image[b][0]) * aspect,
                                     body.keypoints[a][1] - image[b][1]) <= .12
                      for a, b in zip(board, rgb))
        if matches:
            valid += 1
        else:
            for b in rgb:
                filtered[b] = (*pose[b][:3], 0.0)
    return tuple(filtered) if valid else None


def matched_body_turn_pose(body: Optional[Detection], observation: dict,
                           width: int, height: int, confidence: float) -> Optional[Pose3D]:
    """Associate the torso independently of wrists, which disappear during turns."""
    image, pose = observation.get("image_points", ()), observation.get("points", ())
    if body is None or len(body.keypoints) < 13 or len(image) != 17 or len(pose) != 17:
        return None
    aspect = width / height if width > 0 and height > 0 else 1.0
    for board, rgb in ((5, 11), (6, 14), (11, 4), (12, 1)):
        a, b = body.keypoints[board], image[rgb]
        if (len(b) < 4 or len(pose[rgb]) < 4
                or not all(math.isfinite(v) for v in (*a, *b, *pose[rgb]))
                or a[2] < confidence or b[3] < .35 or pose[rgb][3] < .35
                or math.hypot((a[0] - b[0]) * aspect, a[1] - b[1]) > .12):
            return None
    return pose


class BodySolver:
    """Combine board 2D joints with RGB 3D orientation for humanoid rotations."""

    def __init__(
        self,
        mirror: bool = True,
        confidence: float = 0.25,
        smoothing: float = 0.35,
        arm_forward: float = 10.0,
        arm_depth_scale: float = 1.0,
        arm_depth_calibration_frames: int = 30,
        min_segment_length: float = 0.02,
        body_turn: bool = True,
        turn_smoothing: float = 0.18,
        turn_deadzone: float = 3.0,
        turn_max_speed: float = 360.0,
        turn_hold_frames: int = 30,
        arm_depth_only: bool = False,
        conservative_body_turn: bool = False,
    ):
        self.mirror = mirror
        self.confidence = confidence
        self.smoothing = smoothing
        self.arm_forward = arm_forward
        self.arm_depth_scale = arm_depth_scale
        self.arm_depth_calibration_frames = arm_depth_calibration_frames
        self.min_segment_length = min_segment_length
        self.arm_depth_only = arm_depth_only
        self.conservative_body_turn = conservative_body_turn
        self.body_turn = body_turn and (not arm_depth_only or conservative_body_turn)
        self.turn_smoothing = turn_smoothing
        self.turn_deadzone = turn_deadzone
        self.turn_max_speed = turn_max_speed
        self.turn_hold_frames = turn_hold_frames
        self._angles: Dict[str, float] = {
            name: IDLE_ANGLES.get(name, 0.0) for name in BODY_BONES
        }
        self._arm_depths = {
            "LeftUpperArm": arm_forward,
            "LeftLowerArm": 0.0,
            "RightUpperArm": arm_forward,
            "RightLowerArm": 0.0,
        }
        self._leg_depths = {
            "LeftUpperLeg": 0.0,
            "LeftLowerLeg": 0.0,
            "RightUpperLeg": 0.0,
            "RightLowerLeg": 0.0,
        }
        self._depth_misses = 0
        self._arm_depth_misses = {side: 0 for side in ("Left", "Right")}
        self._depth_samples = []
        self._depth_baseline = {} if arm_depth_calibration_frames == 0 else None
        self._body_yaw = 0.0
        self._body_yaw_target = 0.0
        self._raw_body_yaw: Optional[float] = None
        self._body_turn_confidence = 0.0
        self._turn_misses = 0
        self._last_pose_sequence: Optional[int] = None
        self._last_pose_capture_ns = 0
        self._body_basis: Optional[Tuple[Tuple[float, float, float], ...]] = None
        self._body_labels_swapped = False
        self._turn_candidate: Optional[float] = None
        self._turn_candidate_frames = 0

    def _camera_arm_depths(self, points: Pose3D) -> Dict[str, float]:
        """Signed camera depth only; never replace the board's projected directions."""
        depths = {}
        for side, shoulder, elbow, wrist in (("Left", 11, 12, 13), ("Right", 14, 15, 16)):
            if any(not math.isfinite(float(v)) for i in (shoulder, elbow, wrist) for v in points[i][:4]):
                continue
            if min(self._point_confidence(points[i]) for i in (shoulder, elbow, wrist)) < .6:
                continue
            upper = self._depth_angle(points, shoulder, elbow)
            lower = self._depth_angle(points, elbow, wrist)
            if upper is None or lower is None:
                continue
            # Absolute depths, without subtracting the first observed posture. A
            # startup with arms forward must not be calibrated into zero depth.
            upper = _clamp(upper * self.arm_depth_scale, -75.0, 75.0)
            lower = _clamp(lower * self.arm_depth_scale, -75.0, 75.0)
            depths[side + "UpperArm"] = upper
            depths[side + "LowerArm"] = lower - upper
        return depths

    @property
    def arm_depths(self) -> Tuple[float, float]:
        return self._arm_depths["LeftUpperArm"], self._arm_depths["RightUpperArm"]

    @property
    def depth_calibration_progress(self) -> Tuple[int, int]:
        if self.arm_depth_only:
            return 0, 0
        return min(len(self._depth_samples), self.arm_depth_calibration_frames), self.arm_depth_calibration_frames

    @property
    def body_turn_status(self) -> Tuple[float, float, float, int]:
        return self._body_yaw, self._body_yaw_target, self._body_turn_confidence, self._turn_misses

    @staticmethod
    def _point_confidence(point: Sequence[float]) -> float:
        return float(point[3]) if len(point) >= 4 else 1.0

    def _estimate_body_frame(
        self, points: Pose3D
    ) -> Optional[Tuple[float, Tuple[Tuple[float, float, float], ...], float]]:
        if len(points) != 17 or any(
            not math.isfinite(float(value)) for i in (1, 4, 11, 14) for value in points[i][:4]
        ) or min(
            self._point_confidence(points[index]) for index in (1, 4, 11, 14)
        ) < max(self.confidence, 0.35):
            return None
        shoulder_right = _unit(_vector(points[11], points[14]))
        hip_right = _unit(_vector(points[4], points[1]))
        if shoulder_right is None or hip_right is None or _dot(shoulder_right, hip_right) < 0.2:
            return None
        right = _unit(tuple(shoulder_right[index] + hip_right[index] for index in range(3)))
        shoulder_center = tuple((points[11][index] + points[14][index]) * 0.5 for index in range(3))
        hip_center = tuple((points[4][index] + points[1][index]) * 0.5 for index in range(3))
        up = _unit(_vector(hip_center, shoulder_center))
        if right is None or up is None:
            return None
        right = _unit(tuple(right[index] - up[index] * _dot(right, up) for index in range(3)))
        if right is None:
            return None
        # MediaPipe camera coordinates use +z away from the camera. This order
        # makes a front-facing performer's chest direction point toward -z.
        forward = _unit(_cross(up, right))
        if forward is None or math.hypot(forward[0], forward[2]) < 0.5:
            return None
        yaw = _wrap_degrees(math.degrees(math.atan2(forward[0], -forward[2])))
        confidence = min(self._point_confidence(points[index]) for index in (1, 4, 11, 14))
        return yaw, (right, up, forward), confidence

    def _update_body_turn(
        self,
        points: Optional[Pose3D],
        pose_sequence: Optional[int],
        pose_capture_ns: int,
    ) -> Optional[Tuple[Tuple[float, float, float], ...]]:
        if not self.body_turn:
            self._body_yaw_target = 0.0
            self._body_basis = None
            return None
        is_new = pose_sequence is None or pose_sequence != self._last_pose_sequence
        if points is not None and not is_new:
            self._turn_misses = 0
            return self._body_basis
        estimated = self._estimate_body_frame(points) if points is not None else None
        if estimated is None:
            self._turn_misses += 1
            self._body_turn_confidence = 0.0
            if self._turn_misses > self.turn_hold_frames and not self.conservative_body_turn:
                self._body_yaw_target = 0.0
                self._raw_body_yaw = None
                self._body_basis = None
                self._body_labels_swapped = False
            return None

        yaw, basis, confidence = estimated
        right, up, forward = basis
        if self.conservative_body_turn:
            # Match the established camera -> Unity vector transform: (mirror*x,-y,-z).
            yaw *= -1.0 if self.mirror else 1.0
        gap_seconds = (
            (pose_capture_ns - self._last_pose_capture_ns) / 1_000_000_000.0
            if pose_capture_ns > self._last_pose_capture_ns > 0
            else 1.0 / 20.0
        )
        labels_swapped = False
        if self.conservative_body_turn and self._raw_body_yaw is not None:
            # A single left/right flip must not turn the avatar by 180 degrees.
            # Persistent new evidence can also be a real fast turn: accept it after
            # three distinct RGB observations instead of locking to the old facing.
            if gap_seconds < .75 and abs(_wrap_degrees(yaw - self._raw_body_yaw)) > 90.0:
                if self._turn_candidate is None or abs(_wrap_degrees(yaw - self._turn_candidate)) > 30.0:
                    self._turn_candidate_frames = 0
                self._turn_candidate = yaw
                self._turn_candidate_frames += 1
                if self._turn_candidate_frames < 3:
                    self._last_pose_sequence = pose_sequence
                    self._last_pose_capture_ns = pose_capture_ns
                    self._body_turn_confidence = 0.0
                    return None
            self._turn_candidate = None
            self._turn_candidate_frames = 0
        if self._raw_body_yaw is not None and gap_seconds < 0.75:
            if not self.conservative_body_turn and abs(_wrap_degrees(yaw - self._raw_body_yaw)) > 90.0:
                yaw = _wrap_degrees(yaw + 180.0)
                right = tuple(-axis for axis in right)
                forward = tuple(-axis for axis in forward)
                labels_swapped = True
            max_step = self.turn_max_speed * max(gap_seconds, 1.0 / 120.0) + 5.0
            delta = _wrap_degrees(yaw - self._body_yaw_target)
            self._body_yaw_target = _wrap_degrees(
                self._body_yaw_target + _clamp(delta, -max_step, max_step)
            )
        else:
            self._body_yaw_target = yaw
        if abs(self._body_yaw_target) < self.turn_deadzone:
            self._body_yaw_target = 0.0
        self._raw_body_yaw = yaw
        self._body_basis = (right, up, forward)
        self._body_labels_swapped = labels_swapped
        self._body_turn_confidence = confidence
        self._turn_misses = 0
        self._last_pose_sequence = pose_sequence
        self._last_pose_capture_ns = pose_capture_ns
        return self._body_basis

    @staticmethod
    def _body_local_segment(
        points: Pose3D,
        first: int,
        second: int,
        basis: Tuple[Tuple[float, float, float], ...],
    ) -> Optional[Tuple[float, float, float]]:
        segment = _vector(points[first], points[second])
        if math.sqrt(_dot(segment, segment)) < 1e-6:
            return None
        return tuple(_dot(segment, axis) for axis in basis)

    @staticmethod
    def _arm_segment_angles(side: str, segment: Sequence[float]) -> Tuple[float, float]:
        planar = math.hypot(segment[0], segment[1])
        roll = (
            math.degrees(math.atan2(-segment[1], -segment[0]))
            if side == "Left"
            else math.degrees(math.atan2(segment[1], segment[0]))
        )
        return _wrap_degrees(roll), math.degrees(math.atan2(segment[2], planar))

    @staticmethod
    def _leg_segment_angles(segment: Sequence[float]) -> Tuple[float, float]:
        planar = math.hypot(segment[0], segment[1])
        roll = _wrap_degrees(math.degrees(math.atan2(segment[1], segment[0])) + 90.0)
        return roll, math.degrees(math.atan2(segment[2], planar))

    def _line_angle(self, points: Sequence[Tuple[float, float, float]], a: int, b: int) -> Optional[float]:
        first, second = points[a], points[b]
        if first[2] < self.confidence or second[2] < self.confidence:
            return None
        dx = second[0] - first[0]
        if self.mirror:
            dx = -dx
        dy = -(second[1] - first[1])
        if math.hypot(dx, dy) < self.min_segment_length:
            return None
        return math.degrees(math.atan2(dy, dx))

    @staticmethod
    def _set_range(targets: Dict[str, float], name: str, value: float, low: float, high: float) -> None:
        targets[name] = _clamp(_wrap_degrees(value), low, high)

    def _set(self, targets: Dict[str, float], name: str, value: float, limit: float = 170.0) -> None:
        self._set_range(targets, name, value, -limit, limit)

    @staticmethod
    def _depth_angle(points: Pose3D, first: int, second: int) -> Optional[float]:
        dx = points[second][0] - points[first][0]
        dy = points[second][1] - points[first][1]
        dz = points[second][2] - points[first][2]
        planar = math.hypot(dx, dy)
        if math.hypot(planar, dz) < 1e-6:
            return None
        # Human3.6M camera z grows away from the camera, so -dz is forward.
        return math.degrees(math.atan2(-dz, planar))

    def solve(
        self,
        body: Optional[Detection],
        pose3d: Optional[Pose3D] = None,
        pose_sequence: Optional[int] = None,
        pose_capture_ns: int = 0,
        frame_width: int = 1,
        frame_height: int = 1,
        turn_pose3d: Optional[Pose3D] = None,
    ) -> Tuple[BonePose, ...]:
        body = _aspect_correct_detection(body, frame_width, frame_height)
        targets: Dict[str, float] = {
            name: IDLE_ANGLES.get(name, 0.0) for name in BODY_BONES
        }
        depth_targets = {
            "LeftUpperArm": self.arm_forward,
            "LeftLowerArm": 0.0,
            "RightUpperArm": self.arm_forward,
            "RightLowerArm": 0.0,
        }
        leg_depth_targets = {name: 0.0 for name in self._leg_depths}
        body_basis = self._update_body_turn(
            (turn_pose3d if self.conservative_body_turn else pose3d) if body is not None else None,
            pose_sequence,
            pose_capture_ns,
        )
        turn_mix = _clamp((abs(self._body_yaw_target) - 25.0) / 40.0, 0.0, 1.0)
        if pose3d is None:
            self._depth_misses += 1
            if self._depth_misses <= 12:
                depth_targets = dict(self._arm_depths)
                leg_depth_targets = dict(self._leg_depths)
        else:
            self._depth_misses = 0
        if self.arm_depth_only:
            measured_depths = self._camera_arm_depths(pose3d) if body is not None and pose3d is not None and len(pose3d) == 17 else {}
            for side in ("Left", "Right"):
                upper, lower = side + "UpperArm", side + "LowerArm"
                if upper in measured_depths:
                    self._arm_depth_misses[side] = 0
                    depth_targets.update({upper: measured_depths[upper], lower: measured_depths[lower]})
                else:
                    self._arm_depth_misses[side] += 1
                    if self._arm_depth_misses[side] <= 12:
                        depth_targets.update({upper: self._arm_depths[upper], lower: self._arm_depths[lower]})
                    else:
                        depth_targets.update({upper: self.arm_forward, lower: 0.0})
        if body is not None:
            p = body.keypoints
            hips = self._line_angle(p, 11, 12)
            shoulders = self._line_angle(p, 5, 6)
            eyes = self._line_angle(p, 1, 2)
            if self.conservative_body_turn:
                # A back-facing anatomical shoulder/eye line reverses its endpoints;
                # that is yaw, not a 180-degree sideways lean.
                hips, shoulders, eyes = tuple(
                    _wrap_degrees(2.0 * value) * .5 if value is not None else None
                    for value in (hips, shoulders, eyes))
            hip_roll = _clamp(hips or 0.0, -25.0, 25.0)
            shoulder_roll = _clamp(shoulders if shoulders is not None else hip_roll, -35.0, 35.0)
            torso_roll_weight = 1.0 - _clamp((abs(self._body_yaw_target) - 45.0) / 45.0, 0.0, 1.0)
            if self.conservative_body_turn:
                facing_angle = abs(_wrap_degrees(2.0 * self._body_yaw_target)) * .5
                torso_roll_weight = 1.0 - _clamp((facing_angle - 45.0) / 45.0, 0.0, 1.0)
            shoulder_roll *= torso_roll_weight
            self._set(targets, "Hips", 0.0, 0.0)
            self._set(targets, "Spine", shoulder_roll * 0.5, 20.0)
            self._set(targets, "Chest", shoulder_roll * 0.5, 20.0)
            head_delta = _wrap_degrees((eyes if eyes is not None else shoulder_roll) - shoulder_roll)
            self._set(targets, "Neck", head_delta * 0.4, 20.0)
            self._set(targets, "Head", head_delta * 0.6, 30.0)

            for side, shoulder, elbow, wrist, neutral in (
                ("Left", 5, 7, 9, 180.0),
                ("Right", 6, 8, 10, 0.0),
            ):
                upper = self._line_angle(p, shoulder, elbow)
                lower = self._line_angle(p, elbow, wrist)
                if upper is not None:
                    self._set(targets, side + "UpperArm", upper - neutral - shoulder_roll, 180.0)
                if upper is not None and lower is not None:
                    elbow_bend = _wrap_degrees(lower - upper)
                    # A front camera cannot classify one image-plane bend as a backwards elbow.
                    self._set(targets, side + "LowerArm", elbow_bend, 180.0)

            local_rolls: Dict[str, float] = {}
            if pose3d is not None and len(pose3d) == 17 and not self.arm_depth_only:
                raw_depths = {}
                if body_basis is not None:
                    arm_sources = (
                        (("Left", 14, 15, 16), ("Right", 11, 12, 13))
                        if self._body_labels_swapped
                        else (("Left", 11, 12, 13), ("Right", 14, 15, 16))
                    )
                    for side, shoulder, elbow, wrist in arm_sources:
                        upper_segment = self._body_local_segment(pose3d, shoulder, elbow, body_basis)
                        lower_segment = self._body_local_segment(pose3d, elbow, wrist, body_basis)
                        if upper_segment is None or lower_segment is None:
                            continue
                        upper_roll, upper_depth = self._arm_segment_angles(side, upper_segment)
                        lower_roll, lower_depth = self._arm_segment_angles(side, lower_segment)
                        local_rolls[side + "UpperArm"] = _wrap_degrees(upper_roll - shoulder_roll)
                        local_rolls[side + "LowerArm"] = _wrap_degrees(lower_roll - upper_roll)
                        raw_depths[side + "UpperArm"] = upper_depth
                        raw_depths[side + "LowerArm"] = _wrap_degrees(lower_depth - upper_depth)

                    leg_sources = (
                        (("Left", 1, 2, 3), ("Right", 4, 5, 6))
                        if self._body_labels_swapped
                        else (("Left", 4, 5, 6), ("Right", 1, 2, 3))
                    )
                    for side, hip, knee, ankle in leg_sources:
                        upper_segment = self._body_local_segment(pose3d, hip, knee, body_basis)
                        lower_segment = self._body_local_segment(pose3d, knee, ankle, body_basis)
                        if upper_segment is None or lower_segment is None:
                            continue
                        upper_roll, upper_depth = self._leg_segment_angles(upper_segment)
                        lower_roll, lower_depth = self._leg_segment_angles(lower_segment)
                        local_rolls[side + "UpperLeg"] = upper_roll
                        local_rolls[side + "LowerLeg"] = _wrap_degrees(lower_roll - upper_roll)
                        leg_depth_targets[side + "UpperLeg"] = _clamp(upper_depth * turn_mix, -70.0, 70.0)
                        leg_depth_targets[side + "LowerLeg"] = _clamp(
                            _wrap_degrees(lower_depth - upper_depth) * turn_mix, -100.0, 100.0
                        )
                else:
                    for side, shoulder, elbow, wrist in (
                        ("Left", 11, 12, 13),
                        ("Right", 14, 15, 16),
                    ):
                        upper_depth = self._depth_angle(pose3d, shoulder, elbow)
                        lower_depth = self._depth_angle(pose3d, elbow, wrist)
                        if upper_depth is not None:
                            raw_depths[side + "UpperArm"] = upper_depth
                        if upper_depth is not None and lower_depth is not None:
                            raw_depths[side + "LowerArm"] = _wrap_degrees(lower_depth - upper_depth)

                if len(raw_depths) == 4 and self._depth_baseline is None:
                    self._depth_samples.append(raw_depths)
                    if len(self._depth_samples) >= self.arm_depth_calibration_frames:
                        self._depth_baseline = {
                            name: statistics.median(sample[name] for sample in self._depth_samples)
                            for name in raw_depths
                        }
                if self._depth_baseline is not None:
                    for name, raw_depth in raw_depths.items():
                        neutral = self._depth_baseline.get(name, 0.0)
                        offset = self.arm_forward if name.endswith("UpperArm") else 0.0
                        limit = 75.0 if name.endswith("UpperArm") else 90.0
                        depth_targets[name] = _clamp(
                            offset + (raw_depth - neutral) * self.arm_depth_scale,
                            -limit,
                            limit,
                        )

            for side, hip, knee, ankle in (
                ("Left", 11, 13, 15),
                ("Right", 12, 14, 16),
            ):
                upper = self._line_angle(p, hip, knee)
                lower = self._line_angle(p, knee, ankle)
                if upper is not None:
                    self._set(targets, side + "UpperLeg", upper + 90.0, 90.0)
                if upper is not None and lower is not None:
                    self._set(targets, side + "LowerLeg", lower - upper, 120.0)

            for name, local_roll in local_rolls.items():
                limit = 140.0 if name.endswith("Arm") else 120.0
                self._set(
                    targets,
                    name,
                    _angle_blend(targets[name], local_roll, turn_mix),
                    limit,
                )

        for name, target in targets.items():
            delta = _wrap_degrees(target - self._angles[name])
            self._angles[name] = _wrap_degrees(self._angles[name] + delta * self.smoothing)
        if self.arm_depth_only:
            for side in ("Left", "Right"):
                upper, lower = side + "UpperArm", side + "LowerArm"
                old_lower = self._arm_depths[upper] + self._arm_depths[lower]
                target_lower = depth_targets[upper] + depth_targets[lower]
                self._arm_depths[upper] += (depth_targets[upper] - self._arm_depths[upper]) * self.smoothing
                lower_absolute = old_lower + (target_lower - old_lower) * self.smoothing
                self._arm_depths[lower] = lower_absolute - self._arm_depths[upper]
        else:
            for name, target in depth_targets.items():
                delta = _wrap_degrees(target - self._arm_depths[name])
                self._arm_depths[name] = _wrap_degrees(self._arm_depths[name] + delta * self.smoothing)
        for name, target in leg_depth_targets.items():
            delta = _wrap_degrees(target - self._leg_depths[name])
            self._leg_depths[name] = _wrap_degrees(self._leg_depths[name] + delta * self.smoothing)
        yaw_delta = _wrap_degrees(self._body_yaw_target - self._body_yaw)
        self._body_yaw = _wrap_degrees(self._body_yaw + yaw_delta * self.turn_smoothing)
        arm_rotations = {}
        for side in ("Left", "Right"):
            upper, lower = side + "UpperArm", side + "LowerArm"
            upper_rotation = arm_quaternion(side, self._angles[upper], self._arm_depths[upper])
            # Both segment directions are expressed in the torso frame. Convert the
            # lower segment back through its actual parent; Euler subtraction fails
            # when the upper arm also rotates in depth.
            lower_in_torso = arm_quaternion(
                side, self._angles[upper] + self._angles[lower],
                self._arm_depths[upper] + self._arm_depths[lower],
            )
            arm_rotations[upper] = upper_rotation
            arm_rotations[lower] = quaternion_multiply(quaternion_inverse(upper_rotation), lower_in_torso)
        rotations = dict(
            (
                name,
                quaternion_y(self._body_yaw)
                if name == "Hips"
                else arm_rotations[name]
                if name in arm_rotations
                else leg_quaternion(self._angles[name], self._leg_depths[name])
                if name in self._leg_depths
                else quaternion_z(self._angles[name]),
            )
            for name in BODY_BONES
        )
        if self.conservative_body_turn and abs(self._body_yaw) > 1e-9:
            yaw = rotations["Hips"]
            inverse_yaw = quaternion_inverse(yaw)
            # Keep existing image-plane lean in camera space, with yaw carried by
            # the torso/head. No legacy full-3D limb or leg solver is enabled.
            for name in ("Spine", "Chest", "UpperChest", "Neck", "Head",
                         "LeftUpperLeg", "LeftLowerLeg", "RightUpperLeg", "RightLowerLeg"):
                rotations[name] = quaternion_multiply(quaternion_multiply(inverse_yaw, rotations[name]), yaw)
            for side in ("Left", "Right"):
                # Retain the entire camera-space forearm frame, not just its axis:
                # the established planar wrist bend also depends on this frame.
                upper = side + "UpperArm"
                rotations[upper] = quaternion_multiply(inverse_yaw, rotations[upper])
        return tuple(rotations.items())


class FingerSolver:
    """Map 2D/3D landmarks to hand roll, finger flexion and thumb opposition."""

    def __init__(
        self,
        mirror: bool = True,
        confidence: float = 0.25,
        smoothing: float = 0.35,
        missing_hold_frames: int = 5,
        hand3d_drive: bool = False,
        hand_orientation_only: bool = False,
    ):
        self.mirror = mirror
        self.confidence = confidence
        self.smoothing = smoothing
        self.missing_hold_frames = missing_hold_frames
        self.hand3d_drive = hand3d_drive
        self.hand_orientation_only = hand_orientation_only
        self._orientation_misses = {side: 0 for side in ("Left", "Right")}
        self._angles = {name: 0.0 for name in HAND_OUTPUT_BONES}
        self._misses = {name: missing_hold_frames + 1 for name in HAND_OUTPUT_BONES}
        self._twists = {side: 0.0 for side in ("Left", "Right")}
        self._thumb_oppositions = {side: 0.0 for side in ("Left", "Right")}
        self._thumb_misses = {side: missing_hold_frames + 1 for side in ("Left", "Right")}
        self._thumb_extensions = {side: 0.0 for side in ("Left", "Right")}
        self._surface = {side: None for side in ("Left", "Right")}
        self._surface_candidate = {side: None for side in ("Left", "Right")}
        self._surface_evidence = {side: 0 for side in ("Left", "Right")}
        self._last_hand3d_sequence: Optional[int] = None
        self._assigned_sides: Tuple[str, ...] = ()
        self._diagnostics: Dict[str, object] = {}

    @property
    def assigned_sides(self) -> Tuple[str, ...]:
        return self._assigned_sides

    @property
    def diagnostics(self) -> Dict[str, object]:
        return dict(self._diagnostics)

    def _anchor(self, hand: Detection) -> Tuple[float, float]:
        wrist = hand.keypoints[0]
        if wrist[2] >= self.confidence:
            return wrist[0], wrist[1]
        return hand.box[0] + hand.box[2] * 0.5, hand.box[1] + hand.box[3] * 0.5

    def _fallback_assign(self, hands: Sequence[Detection]) -> Dict[str, Detection]:
        ordered = sorted(hands, key=lambda hand: self._anchor(hand)[0])
        if len(ordered) == 1:
            x = self._anchor(ordered[0])[0]
            side = "Left" if (x >= 0.5) == self.mirror else "Right"
            return {side: ordered[0]}
        if self.mirror:
            return {"Right": ordered[0], "Left": ordered[-1]}
        return {"Left": ordered[0], "Right": ordered[-1]}

    def _assign_hands(
        self, hands: Sequence[Detection], body: Optional[Detection]
    ) -> Dict[str, Detection]:
        hands = tuple(hands[:2])
        if not hands:
            return {}
        wrists = {}
        if body is not None and len(body.keypoints) >= 11:
            for side, index in (("Left", 9), ("Right", 10)):
                point = body.keypoints[index]
                if point[2] >= self.confidence:
                    wrists[side] = point
        if len(wrists) == 2:
            if len(hands) == 1:
                anchor = self._anchor(hands[0])
                side = min(wrists, key=lambda name: _distance(anchor, wrists[name]))
                return {side: hands[0]}
            first, second = hands
            first_anchor, second_anchor = self._anchor(first), self._anchor(second)
            direct = _distance(first_anchor, wrists["Left"]) + _distance(second_anchor, wrists["Right"])
            crossed = _distance(first_anchor, wrists["Right"]) + _distance(second_anchor, wrists["Left"])
            return (
                {"Left": first, "Right": second}
                if direct <= crossed
                else {"Left": second, "Right": first}
            )
        if len(wrists) == 1:
            side, wrist = next(iter(wrists.items()))
            nearest = min(hands, key=lambda hand: _distance(self._anchor(hand), wrist))
            if _distance(self._anchor(nearest), wrist) <= 0.25:
                assigned = {side: nearest}
                if len(hands) == 2:
                    assigned["Right" if side == "Left" else "Left"] = next(
                        hand for hand in hands if hand is not nearest
                    )
                return assigned
        return self._fallback_assign(hands)

    def _joint_curl(
        self,
        points: Sequence[Sequence[float]],
        parent: int,
        joint: int,
        child: int,
        deadzone: float,
    ) -> Optional[float]:
        if any(points[index][2] < self.confidence for index in (parent, joint, child)):
            return None
        before = points[parent][0] - points[joint][0], points[parent][1] - points[joint][1]
        after = points[child][0] - points[joint][0], points[child][1] - points[joint][1]
        before_length = math.hypot(*before)
        after_length = math.hypot(*after)
        if min(before_length, after_length) < 0.001:
            return None
        cosine = _clamp(
            (before[0] * after[0] + before[1] * after[1]) / (before_length * after_length),
            -1.0,
            1.0,
        )
        bend = 180.0 - math.degrees(math.acos(cosine))
        return _clamp((bend - deadzone) * 1.15, 0.0, 95.0)

    def _reach_curl(
        self, points: Sequence[Sequence[float]], chain: Sequence[int]
    ) -> Tuple[Optional[float], Optional[float]]:
        indices = (0,) + tuple(chain)
        if any(points[index][2] < self.confidence for index in indices):
            return None, None
        path = sum(_distance(points[first], points[second]) for first, second in zip(indices, indices[1:]))
        if path < 0.004:
            return None, None
        reach = _distance(points[0], points[chain[-1]]) / path
        # shortcut: projected reach cannot recover depth, but it remains useful when palm-facing joints overlap.
        curl = _clamp((0.90 - reach) / 0.45 * 95.0, 0.0, 95.0)
        return reach, curl

    def _world_reach_curl(
        self, points: Sequence[Sequence[float]], chain: Sequence[int]
    ) -> Tuple[Optional[float], Optional[float]]:
        if len(points) != 21:
            return None, None
        indices = (0,) + tuple(chain)
        path = sum(math.dist(points[first][:3], points[second][:3]) for first, second in zip(indices, indices[1:]))
        if path < 0.001:
            return None, None
        reach = math.dist(points[0][:3], points[chain[-1]][:3]) / path
        return reach, _clamp((0.72 - reach) / 0.30 * 95.0, 0.0, 95.0)

    def _thumb_opposition(self, points: Sequence[Sequence[float]], world: bool = False) -> Optional[float]:
        if len(points) != 21:
            return None
        if not world and any(points[index][2] < self.confidence for index in (4, 5, 17)):
            return None
        dimensions = 3 if world else 2
        span = math.dist(points[5][:dimensions], points[17][:dimensions])
        if span < (0.001 if world else 0.004):
            return None
        distance = math.dist(points[4][:dimensions], points[5][:dimensions]) / span
        # Opposition closes a nearly straight thumb across the knuckles without shortening its wrist reach.
        return 45.0 * _smoothstep((1.0 - distance) / 0.65)

    def _thumb_extension(self, points: Sequence[Sequence[float]], world: bool = False) -> Optional[float]:
        """Only a straight thumb clearly outside the other fingers overrides a fist."""
        if len(points) != 21:
            return None
        required = (0, 1, 2, 3, 4, 5, 9, 17)
        if not world and any(points[i][2] < self.confidence for i in required):
            return None
        dimensions = 3 if world else 2
        if not all(math.isfinite(v) for i in required for v in points[i][:dimensions]):
            return None
        distance = lambda a, b: math.dist(points[a][:dimensions], points[b][:dimensions])
        span = distance(5, 17)
        palm = distance(0, 9)
        if min(span, palm) < (0.001 if world else 0.004) or not 0.25 < span / palm < 2.5:
            return None
        path = sum(distance(a, b) for a, b in ((1, 2), (2, 3), (3, 4)))
        if not 0.5 < path / span < 2.5:
            return None
        visible = [i for i in range(5, 21) if world or points[i][2] >= self.confidence]
        separation = min(distance(4, i) for i in visible) / span
        straightness = distance(1, 4) / path
        return _smoothstep((separation - 0.55) / 0.45) * _smoothstep((straightness - 0.82) / 0.15)

    @staticmethod
    def _fist_strength(side: str, targets: Dict[str, Optional[float]]) -> Optional[float]:
        curls = []
        for finger in ("Index", "Middle", "Ring", "Little"):
            values = [targets[side + finger + segment] for segment in ("Intermediate", "Distal")]
            valid = [value for value in values if value is not None]
            if valid:
                curls.append(_smoothstep((max(valid) - 30.0) / 40.0))
        # Three fingers must support a fist; one noisy finger or a pinch cannot close the thumb.
        return sorted(curls)[-3] if len(curls) >= 3 else None

    @staticmethod
    def _world_thumb_bend(points: Sequence[Sequence[float]], indices: Sequence[int], deadzone: float) -> Optional[float]:
        parent, joint, child = indices
        before = _unit(_vector(points[joint], points[parent]))
        after = _unit(_vector(points[joint], points[child]))
        if before is None or after is None:
            return None
        bend = 180.0 - math.degrees(math.acos(_clamp(_dot(before, after), -1.0, 1.0)))
        return _clamp((bend - deadzone) * 1.15, 0.0, 95.0)

    def _finger_rotation(self, name: str) -> Quaternion:
        sign = 1.0 if name.startswith("Left") else -1.0
        if "Thumb" not in name:
            return quaternion_z(sign * self._angles[name])
        proximal = name.endswith("Proximal")
        # shortcut: flex axes measured from model.prefab thumb phalanges, refit for a different rig.
        x, z = (0.02528, sign * 0.02937) if proximal else (0.01474, sign * 0.01874)
        length = math.hypot(x, z)
        weight = 0.40 if proximal else 0.65 if name.endswith("Intermediate") else 0.75
        radians = math.radians(self._angles[name] * weight) * 0.5
        curl = (x / length * math.sin(radians), 0.0, z / length * math.sin(radians), math.cos(radians))
        if not proximal:
            return curl
        side = "Left" if sign > 0 else "Right"
        return quaternion_multiply(quaternion_y(-sign * self._thumb_oppositions[side]), curl)

    @staticmethod
    def _hand_twist(side: str, hand: dict) -> Optional[float]:
        points = hand.get("world_landmarks", ())
        normal = hand.get("palm_normal", ())
        if len(points) != 21 or len(normal) != 3:
            return None

        def unit(vector: Sequence[float]) -> Optional[Tuple[float, float, float]]:
            length = math.sqrt(sum(float(value) ** 2 for value in vector))
            if length < 1e-6:
                return None
            return tuple(float(value) / length for value in vector)

        axis = unit(tuple(points[9][index] - points[0][index] for index in range(3)))
        oriented_normal = tuple(float(value) * (-1.0 if side == "Left" else 1.0) for value in normal)
        palm = unit(oriented_normal)
        if axis is None or palm is None:
            return None

        def project(vector: Sequence[float]) -> Optional[Tuple[float, float, float]]:
            along = sum(vector[index] * axis[index] for index in range(3))
            return unit(tuple(vector[index] - along * axis[index] for index in range(3)))

        reference = project((0.0, 1.0, 0.0))
        if reference is None:
            reference = project((0.0, 0.0, -1.0))
        palm = project(palm)
        if reference is None or palm is None:
            return None
        cross = (
            reference[1] * palm[2] - reference[2] * palm[1],
            reference[2] * palm[0] - reference[0] * palm[2],
            reference[0] * palm[1] - reference[1] * palm[0],
        )
        signed = math.degrees(math.atan2(
            sum(axis[index] * cross[index] for index in range(3)),
            sum(reference[index] * palm[index] for index in range(3)),
        ))
        return _wrap_degrees(-signed)

    def _select_hand3d(
        self, hands: Sequence[dict], assigned: Dict[str, Detection],
        strict: bool = False, aspect: float = 1.0,
    ) -> Dict[str, dict]:
        candidates = []
        for hand in hands:
            side = hand.get("handedness")
            score = hand.get("score", 0.0)
            if (not isinstance(score, (int, float)) or not math.isfinite(score)
                    or score < (.5 if strict else .8)
                    or (not strict and side not in ("Left", "Right"))):
                continue
            candidates.append(hand)

        # MediaPipe handedness depends on whether the camera image is mirrored. Match
        # RGB hands to the board detector's anatomical assignment by wrist position
        # first, then use handedness only when no 2D match is available.
        selected: Dict[str, dict] = {}
        used = set()
        pairs = []
        for side, detected in assigned.items():
            anchor = self._anchor(detected)
            for index, hand in enumerate(candidates):
                landmarks = hand.get("landmarks", ())
                if len(landmarks) == 21 and len(landmarks[0]) >= 2:
                    distance = math.hypot((anchor[0] - landmarks[0][0]) * aspect,
                                          anchor[1] - landmarks[0][1])
                    if strict:
                        # A handedness category is not an anatomical identity on
                        # unmirrored video. Require a visible NPU/RGB wrist match.
                        if distance > .10 or detected.keypoints[0][2] < self.confidence:
                            continue
                        a, b = detected.keypoints[9], landmarks[9]
                        if a[2] < self.confidence or math.hypot((a[0] - b[0]) * aspect, a[1] - b[1]) > .12:
                            continue
                        other = [self._anchor(h) for s, h in assigned.items() if s != side]
                        if any(math.hypot((p[0] - landmarks[0][0]) * aspect,
                                          p[1] - landmarks[0][1]) < distance + .025 for p in other):
                            continue
                    pairs.append((distance, side, index))
        for distance, side, index in sorted(pairs):
            if distance > 0.25 or side in selected or index in used:
                continue
            selected[side] = candidates[index]
            used.add(index)

        if strict:
            return selected
        for index, hand in enumerate(candidates):
            if index in used:
                continue
            side = hand["handedness"]
            if side not in selected or hand["score"] > selected[side].get("score", 0.0):
                selected[side] = hand
        return selected

    def _wrist_twist(self, hand: dict, hand_base: Quaternion) -> Optional[float]:
        """Align the actual rig's +Z index/thumb edge in the current parent frame."""
        points = hand.get("world_landmarks", ())
        if len(points) != 21 or any(not math.isfinite(float(v)) for p in points for v in p[:3]):
            return None
        along, radial = _vector(points[0], points[9]), _vector(points[17], points[5])
        if math.sqrt(_dot(along, along)) < .015 or math.sqrt(_dot(radial, radial)) < .005:
            return None
        along = _unit(along)
        radial = _unit(radial)
        cross = _cross(along, radial)
        if math.sqrt(_dot(cross, cross)) < .2:
            return None
        # Transform vectors, not cross-product normals: reflection changes the
        # latter's chirality. Both real rig hands have index/thumb on local +Z.
        radial = ((-1 if self.mirror else 1) * radial[0], -radial[1], -radial[2])
        local = quaternion_rotate(quaternion_inverse(hand_base), radial)
        if math.hypot(local[1], local[2]) < .35:
            return None
        return _wrap_degrees(math.degrees(math.atan2(-local[1], local[2])))

    def _update_surface(self, hands: Dict[str, dict], sequence: Optional[int]) -> None:
        if sequence is None or sequence == self._last_hand3d_sequence:
            return
        self._last_hand3d_sequence = sequence
        for side, hand in hands.items():
            normal = hand.get("palm_normal", ())
            if len(normal) != 3:
                continue
            oriented_z = float(normal[2]) if side == "Right" else -float(normal[2])
            candidate = "back" if oriented_z > 0.35 else "palm" if oriented_z < -0.35 else None
            if candidate is None:
                continue
            if candidate == self._surface_candidate[side]:
                self._surface_evidence[side] += 1
            else:
                self._surface_candidate[side] = candidate
                self._surface_evidence[side] = 1
            if self._surface_evidence[side] >= 2:
                self._surface[side] = candidate

    def _line_angle(
        self, first: Sequence[float], second: Sequence[float], min_length: float
    ) -> Optional[float]:
        if first[2] < self.confidence or second[2] < self.confidence:
            return None
        dx = second[0] - first[0]
        if self.mirror:
            dx = -dx
        dy = -(second[1] - first[1])
        if math.hypot(dx, dy) < min_length:
            return None
        return math.degrees(math.atan2(dy, dx))

    def _hand_roll(
        self, side: str, hand: Detection, body: Optional[Detection]
    ) -> Optional[float]:
        if body is None:
            return None
        elbow, wrist = (7, 9) if side == "Left" else (8, 10)
        if _distance(self._anchor(hand), body.keypoints[wrist]) > 0.20:
            return None
        forearm = self._line_angle(body.keypoints[elbow], body.keypoints[wrist], 0.02)
        palm = self._line_angle(hand.keypoints[0], hand.keypoints[9], 0.003)
        if forearm is None or palm is None:
            return None
        return _clamp(_wrap_degrees(palm - forearm), -85.0, 85.0)

    def solve(
        self,
        hands: Sequence[Detection],
        body: Optional[Detection],
        hands3d: Sequence[dict] = (),
        hand3d_sequence: Optional[int] = None,
        frame_width: int = 1,
        frame_height: int = 1,
        body_bones: Sequence[BonePose] = (),
    ) -> Tuple[BonePose, ...]:
        # Association uses normalized image positions (center x=.5 and RGB wrists).
        # Only the angle/distance geometry below uses equal units for x and y.
        assigned = self._assign_hands(hands, body)
        selected3d = self._select_hand3d(hands3d, assigned) if self.hand3d_drive else {}
        aspect = frame_width / frame_height if frame_width > 0 and frame_height > 0 else 1.0
        orientation3d = self._select_hand3d(hands3d, assigned, strict=True, aspect=aspect) if self.hand_orientation_only else {}
        body = _aspect_correct_detection(body, frame_width, frame_height)
        assigned = {side: _aspect_correct_detection(hand, frame_width, frame_height)
                    for side, hand in assigned.items()}
        fresh3d = hand3d_sequence is None or hand3d_sequence != self._last_hand3d_sequence
        self._update_surface(selected3d, hand3d_sequence)
        self._assigned_sides = tuple(side for side in ("Left", "Right") if side in assigned)
        targets: Dict[str, Optional[float]] = {name: None for name in HAND_OUTPUT_BONES}
        thumb_targets: Dict[str, Optional[float]] = {side: None for side in ("Left", "Right")}
        diagnostics: Dict[str, object] = {"assigned_sides": list(self._assigned_sides)}
        for side, hand in assigned.items():
            points = hand.keypoints
            thumb_targets[side] = self._thumb_opposition(points)
            roll = self._hand_roll(side, hand, body)
            targets[side + "Hand"] = roll
            side_diagnostics = {
                "score": hand.score,
                "hand_roll_target": roll,
                "fingers": {},
            }
            for finger, chain in FINGER_CHAINS:
                root_deadzone = 25.0 if finger == "Thumb" else 18.0
                reach, reach_curl = self._reach_curl(points, chain)
                finger_diagnostics = {"reach_ratio": reach, "reach_curl": reach_curl}
                for segment, indices, deadzone in (
                    ("Proximal", (0, chain[0], chain[1]), root_deadzone),
                    ("Intermediate", (chain[0], chain[1], chain[2]), 8.0),
                    ("Distal", (chain[1], chain[2], chain[3]), 8.0),
                ):
                    name = side + finger + segment
                    joint_curl = self._joint_curl(points, *indices, deadzone)
                    reach_weight = {"Proximal": 0.65, "Intermediate": 1.0, "Distal": 0.75}[segment]
                    reach_target = reach_curl * reach_weight if reach_curl is not None else None
                    if joint_curl is None:
                        target = reach_target
                    elif reach_target is None:
                        target = joint_curl
                    else:
                        target = max(joint_curl, reach_target)
                    targets[name] = target
                    finger_diagnostics[segment.lower() + "_joint_curl"] = joint_curl
                    finger_diagnostics[segment.lower() + "_target"] = target
                side_diagnostics["fingers"][finger] = finger_diagnostics
            side_diagnostics["fingers"]["Thumb"]["opposition_target"] = thumb_targets[side]
            diagnostics[side] = side_diagnostics

        for side, hand in selected3d.items():
            points = hand.get("world_landmarks", ())
            opposition = self._thumb_opposition(points, world=True)
            if opposition is not None:
                thumb_targets[side] = opposition
            twist_target = self._hand_twist(side, hand)
            hand3d_diagnostics = {
                "score": hand["score"],
                "palm_normal": hand.get("palm_normal"),
                "surface": self._surface[side],
                "twist_target": twist_target,
                "fingers": {},
            }
            for finger, chain in FINGER_CHAINS:
                reach, reach_curl = self._world_reach_curl(points, chain)
                hand3d_diagnostics["fingers"][finger] = {
                    "reach_ratio": reach,
                    "reach_curl": reach_curl,
                }
                if finger == "Thumb":
                    hand3d_diagnostics["fingers"][finger]["opposition_target"] = opposition
                if reach_curl is None:
                    continue
                for segment, weight in (("Proximal", 0.65), ("Intermediate", 1.0), ("Distal", 0.75)):
                    name = side + finger + segment
                    fallback = reach_curl * weight
                    if finger == "Thumb":
                        indices = {"Proximal": (0, 1, 2), "Intermediate": (1, 2, 3), "Distal": (2, 3, 4)}[segment]
                        bend = self._world_thumb_bend(points, indices, 25.0 if segment == "Proximal" else 8.0)
                        targets[name] = fallback if bend is None else max(bend, fallback)
                        hand3d_diagnostics["fingers"][finger][segment.lower() + "_joint_curl"] = bend
                    else:
                        targets[name] = fallback if targets[name] is None else max(targets[name], fallback)
            if twist_target is not None:
                delta = _wrap_degrees(twist_target - self._twists[side])
                self._twists[side] = _wrap_degrees(self._twists[side] + delta * self.smoothing)
            diagnostics.setdefault(side, {})["hand3d"] = hand3d_diagnostics

        for side, opposition in thumb_targets.items():
            fist = self._fist_strength(side, targets)
            extension_evidence = []
            visual_evidence = []
            if side in assigned:
                visual_evidence.append(self._thumb_extension(assigned[side].keypoints))
            if side in selected3d:
                hand = selected3d[side]
                extension_evidence.append(self._thumb_extension(hand.get("world_landmarks", ()), world=True))
                # Handedness score does not measure landmark accuracy: also check the visible silhouette.
                aspect = frame_width / frame_height if frame_width > 0 and frame_height > 0 else 1.0
                projected = tuple((p[0] * aspect, p[1], 1.0) for p in hand.get("landmarks", ()))
                visual_evidence.append(self._thumb_extension(projected))
            visible = [value for value in visual_evidence if value is not None]
            evidence = visible + [value for value in extension_evidence if value is not None]
            extension = min(evidence) if visible else 0.0
            if side in assigned or fresh3d or side not in selected3d:
                previous = self._thumb_extensions[side]
                rate = 0.30 if extension > previous else 0.65
                self._thumb_extensions[side] += (extension - previous) * rate
            extended = _smoothstep((self._thumb_extensions[side] - 0.4) / 0.4)
            closure = (fist or 0.0) * (1.0 - extended)
            if closure > 0.0:
                opposition = max(opposition or 0.0, 45.0 * closure)
            if side in diagnostics:
                diagnostics[side]["thumb_grasp"] = {
                    "fist_strength": fist,
                    "extension_evidence": extension,
                    "extension_filtered": self._thumb_extensions[side],
                    "closure": closure,
                }
            if opposition is None:
                self._thumb_misses[side] += 1
                opposition = self._thumb_oppositions[side] if self._thumb_misses[side] <= self.missing_hold_frames else 0.0
            else:
                self._thumb_misses[side] = 0
                # Blend all three joints into the calibrated wrap together, even when the thumb is occluded.
                for segment, closed_angle in (("Proximal", 60.0), ("Intermediate", 70.0), ("Distal", 60.0)):
                    name = side + "Thumb" + segment
                    if closure > 0.0:
                        measured = targets[name] if targets[name] is not None else 0.0
                        targets[name] = measured + (closed_angle - measured) * closure
            self._thumb_oppositions[side] += (opposition - self._thumb_oppositions[side]) * self.smoothing

        for name, target in targets.items():
            if target is None:
                self._misses[name] += 1
                target = self._angles[name] if self._misses[name] <= self.missing_hold_frames else 0.0
            else:
                self._misses[name] = 0
            self._angles[name] += (target - self._angles[name]) * self.smoothing

        if self.hand_orientation_only:
            rotations = dict(body_bones)
            for side in ("Left", "Right"):
                parent = (0.0, 0.0, 0.0, 1.0)
                for bone in ("Hips", "Spine", "Chest", "UpperChest", side + "Shoulder",
                             side + "UpperArm", side + "LowerArm"):
                    parent = quaternion_multiply(parent, rotations.get(bone, (0.0, 0.0, 0.0, 1.0)))
                base = quaternion_multiply(parent, quaternion_z(self._angles[side + "Hand"]))
                hand = orientation3d.get(side)
                target = self._wrist_twist(hand, base) if hand is not None else None
                self._orientation_misses[side] = 0 if target is not None else self._orientation_misses[side] + 1
                if target is None and self._orientation_misses[side] <= self.missing_hold_frames:
                    target = self._twists[side]
                diagnostics.setdefault(side, {})["wrist3d"] = {
                    "matched": hand is not None, "valid": self._orientation_misses[side] == 0,
                    "twist_target": target, "misses": self._orientation_misses[side],
                }
                target = 0.0 if target is None else target
                self._twists[side] = _angle_blend(self._twists[side], target, self.smoothing)

        for side in ("Left", "Right"):
            if side in diagnostics:
                diagnostics[side]["hand_roll_output"] = self._angles[side + "Hand"]
        diagnostics["output_angles"] = dict(self._angles)
        diagnostics["output_twists"] = dict(self._twists)
        diagnostics["output_thumb_oppositions"] = dict(self._thumb_oppositions)
        self._diagnostics = diagnostics

        return tuple(
            (
                name,
                (
                    quaternion_multiply(
                        quaternion_z(self._angles[name]),
                        quaternion_x(
                            self._twists[name[:-4]] *
                            (-1 if name.startswith("Left") and not self.hand_orientation_only else 1)
                        ),
                    )
                    if name in HAND_BONES
                    else self._finger_rotation(name)
                ),
            )
            for name in HAND_OUTPUT_BONES
        )


def _distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


class FaceSolver:
    """Calibrate each facial region independently and drive the model's real morphs."""

    FILTER_FRAMES = 3
    INVALID_HOLD_FRAMES = 5
    PROFILE_YAW_THRESHOLD = 0.35

    def __init__(self, calibration_frames: int = 30, smoothing: float = 0.4):
        if calibration_frames < 1:
            raise ValueError("calibration_frames must be positive")
        self.calibration_frames = calibration_frames
        self.smoothing = smoothing
        # left/right eye, mouth aperture, mouth width, left/right brow.
        self._samples = [[] for _ in range(6)]
        self._baseline = [None] * 6
        self._filters = [deque(maxlen=self.FILTER_FRAMES) for _ in range(6)]
        self._values = {"Blink_L": 0.0, "Blink_R": 0.0, "A": 0.0, "Joy": 0.0,
                        "BrowUp": 0.0, "BrowDown": 0.0}
        self._active = {name: False for name in self._values}
        self._pending = {name: 0 for name in self._values}
        self._invalid_frames = {name: 0 for name in self._values}
        self._diagnostics: Dict[str, object] = {}

    @property
    def calibration_progress(self) -> Tuple[int, int]:
        return min(min(len(samples) for samples in self._samples[:3]), self.calibration_frames), self.calibration_frames

    @staticmethod
    def _separation(first: Sequence[float], second: Sequence[float], axis: Sequence[float]) -> float:
        # Distance perpendicular to the local eye/lip line; horizontal jitter is
        # not eyelid or mouth opening. This is invariant to head roll.
        return abs((second[0] - first[0]) * axis[1] - (second[1] - first[1]) * axis[0])

    @property
    def diagnostics(self) -> Dict[str, object]:
        return dict(self._diagnostics)

    @staticmethod
    def _eye(points: Sequence[Sequence[float]], start: int, min_width: float) -> Optional[float]:
        p = points[start : start + 6]
        width = _distance(p[0], p[3])
        if width < min_width or any(point[2] < 0.2 for point in p):
            return None
        axis = ((p[3][0] - p[0][0]) / width, (p[3][1] - p[0][1]) / width)
        value = (FaceSolver._separation(p[1], p[5], axis) + FaceSolver._separation(p[2], p[4], axis)) / (2.0 * width)
        return value if math.isfinite(value) and value <= 1.25 else None

    @staticmethod
    def _scaled_points(
        face: Detection, frame_width: int, frame_height: int
    ) -> Tuple[Tuple[float, float, float], ...]:
        return tuple((point[0] * frame_width, point[1] * frame_height, point[2]) for point in face.keypoints)

    def _measure(
        self, face: Detection, points: Sequence[Sequence[float]], frame_width: int
    ) -> Tuple[Optional[float], ...]:
        face_width = face.box[2] * frame_width
        min_eye_width = max(face_width * 0.04, 1.5)
        right_eye = self._eye(points, 19, min_eye_width)
        left_eye = self._eye(points, 25, min_eye_width)
        # Outer lip corners remain separated when the inner lips close/collapse.
        mouth_width = _distance(points[31], points[37])
        mouth_open = smile_width = None
        if mouth_width >= max(face_width * 0.08, 3.0) and min(points[i][2] for i in (31, 37)) >= 0.2:
            smile_width = mouth_width / max(face_width, 1e-6)
            if min(points[i][2] for i in (44, 45, 46, 48, 49, 50)) >= 0.2:
                axis = tuple((points[37][i] - points[31][i]) / mouth_width for i in (0, 1))
                value = sum(self._separation(points[a], points[b], axis)
                            for a, b in ((44, 50), (45, 49), (46, 48))) / (3.0 * mouth_width)
                if math.isfinite(value) and value <= 2.0:
                    mouth_open = value
        return left_eye, right_eye, mouth_open, smile_width

    @staticmethod
    def _yaw_proxy(face: Detection, points: Sequence[Sequence[float]]) -> Optional[float]:
        needed = (13,) + tuple(range(19, 31))
        if any(face.keypoints[index][2] < 0.2 for index in needed):
            return None
        right_eye = points[19:25]
        left_eye = points[25:31]
        right_center = tuple(sum(point[axis] for point in right_eye) / 6.0 for axis in (0, 1))
        left_center = tuple(sum(point[axis] for point in left_eye) / 6.0 for axis in (0, 1))
        eye_distance = _distance(left_center, right_center)
        if eye_distance < 1.5:
            return None
        eye_mid_x = (left_center[0] + right_center[0]) / 2.0
        return (points[13][0] - eye_mid_x) / eye_distance

    @staticmethod
    def _brow_eye_ratio(
        face: Detection,
        points: Sequence[Sequence[float]],
        brow_start: int,
        eye_start: int,
    ) -> Optional[float]:
        needed = tuple(range(brow_start, brow_start + 5)) + tuple(range(eye_start, eye_start + 6))
        if any(face.keypoints[index][2] < 0.2 for index in needed):
            return None
        eye_width = _distance(points[eye_start], points[eye_start + 3])
        if eye_width < 1.5:
            return None
        axis = tuple((points[eye_start + 3][i] - points[eye_start][i]) / eye_width for i in (0, 1))
        brow = tuple(sum(points[k][i] for k in range(brow_start, brow_start + 5)) / 5.0 for i in (0, 1))
        eye = tuple(sum(points[k][i] for k in range(eye_start, eye_start + 6)) / 6.0 for i in (0, 1))
        return FaceSolver._separation(brow, eye, axis) / eye_width

    def _update_calibration(self, measured: Sequence[Optional[float]]) -> None:
        limits = ((0.04, 1.0), (0.04, 1.0), (0.0, 0.35), (0.05, 1.0), (0.05, 2.0), (0.05, 2.0))
        for i, (samples, value, (low, high)) in enumerate(zip(self._samples, measured, limits)):
            if value is not None and len(samples) < self.calibration_frames and low <= value <= high:
                samples.append(value)
            if self._baseline[i] is None and len(samples) >= self.calibration_frames:
                self._baseline[i] = statistics.median(samples)
                self._filters[i].extend([self._baseline[i]] * self.FILTER_FRAMES)

    def _filtered(self, measured: Sequence[Optional[float]]) -> Tuple[Optional[float], ...]:
        for values, value in zip(self._filters, measured):
            if value is not None:
                values.append(value)
        return tuple(statistics.median(values) if value is not None else None
                     for value, values in zip(measured, self._filters))

    def _with_hysteresis(self, name: str, value: float, turn_on: float, turn_off: float) -> float:
        wanted = value > turn_off if self._active[name] else value >= turn_on
        if wanted == self._active[name]:
            self._pending[name] = 0
        else:
            self._pending[name] += 1
            if self._pending[name] >= 2:
                self._active[name] = wanted
                self._pending[name] = 0
        return value if self._active[name] else 0.0

    def solve(
        self, face: Optional[Detection], frame_width: int = 1920, frame_height: int = 1080
    ) -> Dict[str, float]:
        points = self._scaled_points(face, frame_width, frame_height) if face is not None else ()
        raw_measured = self._measure(face, points, frame_width) if face is not None else (None,) * 4
        yaw_proxy = self._yaw_proxy(face, points) if face is not None else None
        left_brow = self._brow_eye_ratio(face, points, 5, 25) if face is not None else None
        right_brow = self._brow_eye_ratio(face, points, 0, 19) if face is not None else None
        # A nose-offset proxy is not a reliable reason to erase valid eyelids/lips.
        # Keep it in diagnostics; confidence/geometry validity is local to each region.
        measured = raw_measured + (left_brow, right_brow)
        self._update_calibration(measured)
        filtered = self._filtered(measured)
        targets = {name: None for name in self._values}
        targets["Joy"] = 0.0  # The global Joy clip also closes the eyes; do not infer emotion from mouth width.
        for i, name in enumerate(("Blink_L", "Blink_R")):
            base, value = self._baseline[i], filtered[i]
            if base is not None and value is not None:
                blink = _clamp((base * 0.85 - value) / max(base * 0.45, 1e-6), 0.0, 1.0)
                targets[name] = self._with_hysteresis(name, blink, 0.45, 0.12)
        base_mouth, mouth = self._baseline[2], filtered[2]
        if base_mouth is not None and mouth is not None:
            mouth_closed = max(base_mouth + 0.06, 0.08)
            mouth_open = max(mouth_closed + 0.14, base_mouth + 0.30)
            value = _clamp((mouth - mouth_closed) / (mouth_open - mouth_closed), 0.0, 1.0)
            targets["A"] = self._with_hysteresis("A", value, 0.35, 0.15)
        brow_deltas = [filtered[i] - self._baseline[i] for i in (4, 5)
                       if filtered[i] is not None and self._baseline[i] is not None]
        if brow_deltas:
            # The supplied rig has bilateral brow morphs, no independent brow bones.
            delta = statistics.mean(brow_deltas)
            for name, sign in (("BrowUp", 1.0), ("BrowDown", -1.0)):
                value = _clamp((sign * delta - 0.08) / 0.25, 0.0, 1.0)
                targets[name] = self._with_hysteresis(name, value, 0.35, 0.15)
        for name, target in targets.items():
            if target is None:
                self._invalid_frames[name] += 1
                # A pending transition requires consecutive valid measurements.
                self._pending[name] = 0
                target = self._values[name] if self._invalid_frames[name] <= self.INVALID_HOLD_FRAMES else 0.0
                if self._invalid_frames[name] > self.INVALID_HOLD_FRAMES:
                    self._active[name] = False
            else:
                self._invalid_frames[name] = 0
            targets[name] = target
            self._values[name] += (target - self._values[name]) * self.smoothing
        needed = tuple(range(19, 31)) + tuple(range(31, 51))
        ratios = raw_measured
        baseline = self._baseline
        self._diagnostics = {
            "face_present": face is not None,
            "face_score": face.score if face is not None else None,
            "box_x": face.box[0] if face is not None else None,
            "box_y": face.box[1] if face is not None else None,
            "box_w": face.box[2] if face is not None else None,
            "box_h": face.box[3] if face is not None else None,
            "min_required_conf": min(face.keypoints[index][2] for index in needed) if face is not None else None,
            "measurement_valid": any(value is not None for value in raw_measured[:3]),
            "left_eye_valid": raw_measured[0] is not None,
            "right_eye_valid": raw_measured[1] is not None,
            "mouth_valid": raw_measured[2] is not None,
            "left_eye_calibration": len(self._samples[0]),
            "right_eye_calibration": len(self._samples[1]),
            "mouth_calibration": len(self._samples[2]),
            "left_eye_ratio": ratios[0],
            "right_eye_ratio": ratios[1],
            "mouth_open_ratio": ratios[2],
            "smile_width_ratio": ratios[3],
            "yaw_proxy": yaw_proxy,
            "profile_suppressed": False,
            "profile_detected": yaw_proxy is not None and abs(yaw_proxy) >= self.PROFILE_YAW_THRESHOLD,
            "left_brow_eye_ratio": left_brow,
            "right_brow_eye_ratio": right_brow,
            "base_left_eye": baseline[0],
            "base_right_eye": baseline[1],
            "base_mouth_open": baseline[2],
            "base_smile_width": baseline[3],
            "calibration_done": self.calibration_progress[0],
            "calibration_total": self.calibration_progress[1],
            "target_blink_l": targets["Blink_L"],
            "target_blink_r": targets["Blink_R"],
            "target_a": targets["A"],
            "target_joy": targets["Joy"],
            "output_blink_l": self._values["Blink_L"],
            "output_blink_r": self._values["Blink_R"],
            "output_a": self._values["A"],
            "output_joy": self._values["Joy"],
            "output_brow_up": self._values["BrowUp"],
            "output_brow_down": self._values["BrowDown"],
        }
        return dict(self._values)


FACE_LOG_FIELDS = (
    "receive_time_ns", "source_ip", "sequence", "capture_time_ns", "emit_time_ns", "source_id",
    "face_keypoints_json",
    "face_present", "face_score", "box_x", "box_y", "box_w", "box_h", "min_required_conf",
    "measurement_valid", "left_eye_ratio", "right_eye_ratio", "mouth_open_ratio", "smile_width_ratio",
    "left_eye_valid", "right_eye_valid", "mouth_valid",
    "left_eye_calibration", "right_eye_calibration", "mouth_calibration", "profile_detected",
    "yaw_proxy", "profile_suppressed", "left_brow_eye_ratio", "right_brow_eye_ratio",
    "base_left_eye", "base_right_eye", "base_mouth_open", "base_smile_width",
    "calibration_done", "calibration_total", "target_blink_l", "target_blink_r", "target_a", "target_joy",
    "output_blink_l", "output_blink_r", "output_a", "output_joy",
    "output_brow_up", "output_brow_down",
)


class FaceCsvLogger:
    def __init__(self, path: str):
        output = Path(path).expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        self.path = output
        self._file = output.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=FACE_LOG_FIELDS)
        self._writer.writeheader()
        self._file.flush()

    def write(self, frame: MotionFrame, source_ip: str, diagnostics: Dict[str, object]) -> None:
        row = {
            "receive_time_ns": time.time_ns(),
            "source_ip": source_ip,
            "sequence": frame.sequence,
            "capture_time_ns": frame.capture_time_ns,
            "emit_time_ns": frame.emit_time_ns,
            "source_id": frame.source_id,
            "face_keypoints_json": json.dumps(frame.face.keypoints, separators=(",", ":")) if frame.face else "",
        }
        row.update(diagnostics)
        self._writer.writerow(row)
        self._file.flush()

    def close(self) -> None:
        self._file.close()


class FingerJsonlLogger:
    """Write raw FNG1 landmarks and derived V2 features for short tuning captures."""

    def __init__(self, path: str):
        output = Path(path).expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        self.path = output
        self._file = output.open("w", encoding="utf-8")

    def write(
        self,
        motion_frame: MotionFrame,
        finger_frame: Optional[FingerFrame],
        source_ip: str,
        diagnostics: Dict[str, object],
    ) -> None:
        hands = finger_frame.hands if finger_frame else ()
        record = {
            "receive_time_ns": time.time_ns(),
            "source_ip": source_ip,
            "motion_sequence": motion_frame.sequence,
            "finger_sequence": finger_frame.sequence if finger_frame else None,
            "finger_lag": (
                (motion_frame.sequence - finger_frame.sequence) & 0xFFFFFFFF
                if finger_frame else None
            ),
            "capture_time_ns": finger_frame.capture_time_ns if finger_frame else None,
            "frame_width": motion_frame.frame_width,
            "frame_height": motion_frame.frame_height,
            "body_keypoints": motion_frame.body.keypoints if motion_frame.body else None,
            "body_arm_points": {
                "left": [motion_frame.body.keypoints[index] for index in (7, 9)],
                "right": [motion_frame.body.keypoints[index] for index in (8, 10)],
            } if motion_frame.body else None,
            "hands": [
                {"score": hand.score, "box": hand.box, "keypoints": hand.keypoints}
                for hand in hands
            ],
            "solver": diagnostics,
        }
        self._file.write(json.dumps(record, separators=(",", ":")) + "\n")
        self._file.flush()

    def close(self) -> None:
        self._file.close()


def select_finger_frame(
    frames: Iterable[FingerFrame], target_sequence: int, source_id: int, max_lag: int
) -> Optional[FingerFrame]:
    """Choose an exact or recent same-camera FNG1 frame, including uint32 wraparound."""
    candidates = []
    for frame in frames:
        if frame.source_id != source_id:
            continue
        lag = (target_sequence - frame.sequence) & 0xFFFFFFFF
        if lag <= max_lag:
            candidates.append((lag, frame))
    return min(candidates, key=lambda item: item[0])[1] if candidates else None


def poll_finger_frame(
    receiver: socket.socket,
    cache: Dict[Tuple[int, int], FingerFrame],
    target_sequence: int,
    source_id: int,
    wait_seconds: float,
    max_lag: int,
) -> Tuple[Optional[FingerFrame], int, int]:
    """Wait briefly for same-frame FNG1, then fall back to a bounded older frame."""
    accepted = rejected = 0
    deadline = time.monotonic() + wait_seconds
    exact_key = source_id, target_sequence
    first_poll = True
    while exact_key not in cache:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0 and not first_poll:
            break
        readable, _, _ = select.select((receiver,), (), (), max(0.0, remaining))
        first_poll = False
        if not readable:
            break
        while True:
            try:
                packet, _source = receiver.recvfrom(2048)
            except BlockingIOError:
                break
            try:
                frame = unpack_finger_frame(packet)
            except ValueError:
                rejected += 1
                continue
            accepted += 1
            cache[(frame.source_id, frame.sequence)] = frame
            while len(cache) > 64:
                cache.pop(next(iter(cache)))

    selected = select_finger_frame(cache.values(), target_sequence, source_id, max_lag)
    if selected is not None:
        cache.pop((selected.source_id, selected.sequence), None)
    return selected, accepted, rejected


class VmcSender:
    def __init__(self, host: str, port: int):
        self.destination = host, port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.started = time.monotonic()
        self.last_ok = 0.0
        self.last_mode = ""
        self.last_mode_at = 0.0

    def _send(self, address: str, *values: object) -> None:
        self.socket.sendto(osc_message(address, *values), self.destination)

    def send_mode(self, mode: str, force: bool = False) -> None:
        now = time.monotonic()
        if force or mode != self.last_mode or now - self.last_mode_at >= 1.0:
            self._send("/Avatar/Mode", mode)
            self.last_mode = mode
            self.last_mode_at = now

    def send(self, bones: Iterable[BonePose], blends: Dict[str, float], mode: str = "") -> None:
        now = time.monotonic()
        if now - self.last_ok >= 1.0:
            self._send("/VMC/Ext/OK", 1)
            self.last_ok = now
        if mode:
            self.send_mode(mode)
        for name, rotation in bones:
            self._send("/VMC/Ext/Bone/Pos", name, 0.0, 0.0, 0.0, *rotation)
        for name, value in blends.items():
            # Keep both eyes open regardless of FACE eyelid measurements.
            self._send("/VMC/Ext/Blend/Val", name, 0.0 if name in ("Blink_L", "Blink_R") else float(value))
        self._send("/VMC/Ext/Blend/Apply")
        self._send("/VMC/Ext/T", float(now - self.started))

    def send_gesture(self, name: str, active: int) -> None:
        """Publish the gesture signal bit; Unity chooses trigger or sustained use."""
        self._send("/Avatar/Gesture", name, int(active))


NEUTRAL_BLENDS = {"Blink_L": 0.0, "Blink_R": 0.0, "A": 0.0, "Joy": 0.0,
                  "BrowUp": 0.0, "BrowDown": 0.0}


def neutral_bones(arm_forward: float) -> Tuple[BonePose, ...]:
    """A relaxed standing pose.

    Interaction mode sends this every frame instead of the solved capture pose,
    so the avatar holds still and only moves when the gesture fires.
    """
    return tuple(
        (
            name,
            arm_quaternion("Left", IDLE_ANGLES[name], arm_forward)
            if name == "LeftUpperArm"
            else arm_quaternion("Right", IDLE_ANGLES[name], arm_forward)
            if name == "RightUpperArm"
            else quaternion_z(IDLE_ANGLES.get(name, 0.0)),
        )
        for name in BODY_BONES
    )


def poll_gesture_frames(receiver: socket.socket) -> Tuple[Optional[GestureFrame], int, int]:
    """Drain the GST1 socket; return the newest frame plus accept/reject counts.

    GST1 arrives at the camera rate while the motion loop runs at its own pace,
    so every datagram is consumed and only the newest state is kept.
    """
    newest: Optional[GestureFrame] = None
    accepted = rejected = 0
    while True:
        try:
            packet, _source = receiver.recvfrom(2048)
        except (BlockingIOError, InterruptedError):
            break
        except OSError:
            break
        try:
            newest = unpack_gesture_frame(packet)
            accepted += 1
        except ValueError:
            rejected += 1
    return newest, accepted, rejected


def poll_motion_frames(
    receiver: socket.socket,
) -> Tuple[Optional[MotionFrame], Optional[Tuple[str, int]], int, int, int]:
    """Drain queued DMV packets and keep the newest valid primary-person frame."""
    newest: Optional[MotionFrame] = None
    newest_source: Optional[Tuple[str, int]] = None
    accepted = rejected = ignored = 0
    while True:
        try:
            packet, source = receiver.recvfrom(2048)
        except (BlockingIOError, InterruptedError, OSError):
            break
        try:
            frame = unpack_frame(packet)
        except ValueError:
            rejected += 1
            continue
        if not is_primary_frame(frame):
            ignored += 1
            continue
        accepted += 1
        if newest is None or frame.capture_time_ns >= newest.capture_time_ns:
            newest, newest_source = frame, source
    return newest, newest_source, accepted, rejected, ignored


def board_gesture_edges(
    frame: Optional[GestureFrame],
    last: Dict[str, int],
) -> List[Tuple[str, int, Optional[GestureRecord]]]:
    """Edge events for every gesture whose signal bit changed since ``last``.

    ``last`` is updated in place.  ``None`` means the board stream went stale,
    which releases every gesture that was still high so a dropped link cannot
    leave the avatar stuck mid-reaction.
    """
    current: Dict[str, int] = {}
    records: Dict[str, GestureRecord] = {}
    if frame is not None:
        for record in frame.records:
            current[record.name] = record.active
            records[record.name] = record

    edges: List[Tuple[str, int, Optional[GestureRecord]]] = []
    for name in sorted(set(current) | set(last)):
        active = current.get(name, 0)
        if active != last.get(name, 0):
            edges.append((name, active, records.get(name)))
            last[name] = active
    return edges


def board_gesture_summary(last: Dict[str, int]) -> str:
    """Compact 'which board gestures are high' text for the status line."""
    active = sorted(name for name, bit in last.items() if bit)
    return ",".join(active) if active else "off"


def board_gesture_tuning(frame: Optional[GestureFrame]) -> Tuple[str, str]:
    """Live dwell progress of the busiest board gesture, plus its outside metric.

    Tuning aid. progress climbs 0.00 -> 1.00 while the hands dwell inside that
    gesture's region, and metric is how far OUTSIDE the region the hand sits
    (0.00 = inside, in units of head or shoulder width). Reading them shows
    whether a gesture that never fired was in the wrong place or just lost:

        prog stays 0.00, out > 0.10   -> the region is too small / misplaced
        prog climbs then resets       -> the hand detection keeps dropping out
        prog climbs slowly            -> frame rate or confidence limits

    The gesture name is included because the board now publishes three of them.
    """
    if frame is None or not frame.records:
        return "-", "-"
    record = max(frame.records, key=lambda item: item.progress)
    progress = "{}:{:.2f}".format(record.name, record.progress)
    metric = "{:.2f}".format(record.metric) if record.metric_valid else "-"
    return progress, metric


def run_bridge(args: argparse.Namespace) -> None:
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind((args.bind, args.port))
    receiver.setblocking(False)
    sender = VmcSender(args.vmc_host, args.vmc_port)
    current_mode = args.mode
    mode_control = (
        ModeControlReceiver(args.bind, args.mode_control_port, args.mode_control_host, current_mode)
        if args.mode_control_port else None
    )

    def apply_mode(mode):
        nonlocal current_mode
        if mode != current_mode:
            current_mode = mode
            sender.send_mode(mode, force=True)
            print("[MODE] applied {} (source=voice/button control)".format(mode), flush=True)
        return current_mode

    body_solver = BodySolver(
        mirror=args.mirror,
        confidence=args.confidence,
        smoothing=args.smoothing,
        arm_forward=args.arm_forward,
        arm_depth_scale=args.arm_depth_scale,
        arm_depth_calibration_frames=args.arm_depth_calibration_frames,
        min_segment_length=args.min_segment_length,
        body_turn=not args.no_body_turn,
        arm_depth_only=args.rgb_arm_depth_only,
        conservative_body_turn=args.rgb_body_turn,
    )
    finger_solver = FingerSolver(
        mirror=args.mirror,
        confidence=args.finger_confidence,
        smoothing=args.finger_smoothing,
        missing_hold_frames=args.finger_hold_frames,
        hand3d_drive=args.rgb_hand_drive,
        hand_orientation_only=args.rgb_hand_orientation_only,
    )
    neutral_hands = tuple((name, (0.0, 0.0, 0.0, 1.0)) for name in HAND_OUTPUT_BONES)
    face_solver = FaceSolver(calibration_frames=args.calibration_frames, smoothing=args.smoothing)
    face_logger = FaceCsvLogger(args.face_log) if args.face_log else None
    finger_logger = FingerJsonlLogger(args.finger_log) if args.finger_log else None
    gesture = (
        HeadPatTrigger(
            name=args.gesture_name,
            dwell_seconds=args.gesture_dwell,
            region_above=args.gesture_above,
            region_half_width=args.gesture_width,
            min_confidence=args.gesture_confidence,
        )
        if args.gesture
        else None
    )
    gesture_logger = HeadPatJsonlLogger(args.gesture_log) if args.gesture_log else None
    gesture_state = None
    gesture_last = 0
    # Board-side gesture signals arrive independently of the motion stream.
    board_gesture_receiver = None
    if args.gesture_source in ("board", "both"):
        board_gesture_receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        board_gesture_receiver.bind((args.bind, args.gesture_port))
        board_gesture_receiver.setblocking(False)
    board_gesture_frame = None
    board_gesture_last: Dict[str, int] = {}
    board_gesture_at = 0.0
    board_gesture_accepted = board_gesture_rejected = 0
    board_gesture_stale_reported = False
    finger_receiver = None
    finger_cache: Dict[Tuple[int, int], FingerFrame] = {}
    finger_accepted = finger_rejected = 0
    if not args.no_fingers:
        finger_receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        finger_receiver.bind((args.bind, args.finger_port))
        finger_receiver.setblocking(False)
    lifter = None
    rgb_receiver = None
    if args.rgb_pose_model:
        from rgb_pose_receiver import RgbPoseReceiver

        rgb_receiver = RgbPoseReceiver(
            args.bind,
            args.rgb_port,
            args.rgb_pose_model,
            args.rgb_hand_model,
            args.rgb_hand_log,
        )
        rgb_receiver.start()
        print("MediaPipe RGB 3D enabled: {} on UDP {}".format(args.rgb_pose_model, args.rgb_port))
        if args.rgb_hand_model:
            print("MediaPipe hand RGB enabled ({}): {} -> {}".format(
                "orientation only" if args.rgb_hand_orientation_only else "legacy full drive" if args.rgb_hand_drive else "log only",
                args.rgb_hand_model, args.rgb_hand_log or "no log"
            ))
    if args.pose3d_model:
        from pose3d_lifter import VideoPose3DLifter

        lifter = VideoPose3DLifter(args.pose3d_model, confidence=args.confidence)
        print("VideoPose3D enabled: {}".format(args.pose3d_model))
    accepted = rejected = 0
    ignored_tracks = 0
    report_at = time.monotonic() + 1.0
    finger_status = "disabled" if finger_receiver is None else "{}:{}".format(args.bind, args.finger_port)
    gesture_status = (
        "{}:{}".format(args.bind, args.gesture_port)
        if board_gesture_receiver is not None
        else "disabled"
    )
    print(
        "DMV1/DMV2 {}:{} + FNG1 {} + GST1 {} -> VMC {}:{} (Ctrl+C to stop)".format(
            args.bind, args.port, finger_status, gesture_status, args.vmc_host, args.vmc_port
        ),
        flush=True,
    )
    print(
        "模式: {} — {}".format(
            current_mode,
            "数字人跟随你的动作"
            if args.mode == "follow"
            else "数字人保持站立不跟随，只对板端交互动作做出回应",
        ),
        flush=True,
    )
    if mode_control is not None:
        print("[MODE] control UDP {}:{} (allowed host: {})".format(
            args.bind, args.mode_control_port, args.mode_control_host or "trusted LAN"
        ), flush=True)
    readers = [receiver]
    if mode_control is not None:
        readers.append(mode_control.socket)
    if board_gesture_receiver is not None:
        readers.append(board_gesture_receiver)
    while True:
        readable, _, _ = select.select(readers, (), (), 0.1)
        if mode_control is not None and mode_control.socket in readable:
            mode_control.poll(apply_mode)
        # Mode sync and stale-gesture release must also run with no camera frames.
        sender.send_mode(current_mode)
        if board_gesture_receiver is not None:
            board_now = time.monotonic()
            newest, accepted_delta, rejected_delta = poll_gesture_frames(board_gesture_receiver)
            board_gesture_accepted += accepted_delta
            board_gesture_rejected += rejected_delta
            if newest is not None:
                board_gesture_frame = newest
                board_gesture_at = board_now
                board_gesture_stale_reported = False
            elif board_gesture_frame is not None and board_now - board_gesture_at > 1.0:
                board_gesture_frame = None
                if not board_gesture_stale_reported:
                    board_gesture_stale_reported = True
                    print("[GESTURE] board GST1 stream stale; releasing gesture signal", flush=True)
            for name, active, record in board_gesture_edges(board_gesture_frame, board_gesture_last):
                sender.send_gesture(name, active)
                if args.gesture_verbose and record is not None:
                    detail = "board_active={} board_progress={:.2f}".format(active, record.progress)
                    if record.metric_valid:
                        detail += " board_outside={:.2f}".format(record.metric)
                else:
                    detail = "board_active={}".format(active)
                print("[GESTURE] {} {} {} (source=board)".format(
                    name, "ON" if active else "OFF", detail
                ), flush=True)
        if receiver not in readable:
            continue
        frame, source, accepted_delta, rejected_delta, ignored_delta = poll_motion_frames(receiver)
        accepted += accepted_delta
        rejected += rejected_delta
        ignored_tracks += ignored_delta
        if frame is None or source is None:
            continue
        finger_frame = None
        if finger_receiver is not None:
            finger_frame, newly_accepted, newly_rejected = poll_finger_frame(
                finger_receiver,
                finger_cache,
                frame.sequence,
                frame.source_id,
                args.finger_sync_wait,
                args.finger_max_lag,
            )
            finger_accepted += newly_accepted
            finger_rejected += newly_rejected
        pose_sequence, pose_capture_ns, pose3d = (
            rgb_receiver.latest_frame(args.rgb_pose_max_age) if rgb_receiver else (-1, 0, None)
        )
        hand3d_sequence, hands3d = (
            rgb_receiver.latest_hands(args.rgb_pose_max_age) if rgb_receiver else (-1, ())
        )
        turn_pose3d = None
        if rgb_receiver and (args.rgb_hand_orientation_only or args.rgb_arm_depth_only):
            pose_observation, hand_observation = rgb_receiver.latest_observations(args.rgb_pose_max_age)
            pose3d = None
            hands3d = ()
            if args.rgb_arm_depth_only and rgb_observation_matches(
                pose_observation, frame, source[0], args.rgb_max_skew_ms
            ):
                pose3d = matched_arm_depth_pose(frame.body, pose_observation,
                                               frame.frame_width, frame.frame_height, args.confidence)
                pose_sequence, pose_capture_ns = pose_observation["sequence"], pose_observation["capture_ns"]
                if args.rgb_body_turn:
                    turn_pose3d = matched_body_turn_pose(frame.body, pose_observation,
                                                       frame.frame_width, frame.frame_height, args.confidence)
            if rgb_observation_matches(hand_observation, frame, source[0], args.rgb_max_skew_ms):
                hand3d_sequence, hands3d = hand_observation["sequence"], hand_observation["hands"]
        if pose3d is None and lifter:
            pose3d = lifter.update(frame.body, frame.frame_width, frame.frame_height)
            if pose3d is not None:
                pose_sequence, pose_capture_ns = frame.sequence, frame.capture_time_ns
        body_bones = body_solver.solve(
            frame.body, pose3d, pose_sequence, pose_capture_ns,
            frame_width=frame.frame_width, frame_height=frame.frame_height,
            turn_pose3d=turn_pose3d,
        )
        finger_bones = finger_solver.solve(
            finger_frame.hands if finger_frame and frame.body is not None else (),
            frame.body,
            hands3d,
            hand3d_sequence,
            frame_width=frame.frame_width,
            frame_height=frame.frame_height,
            body_bones=body_bones,
        )
        # Keep calibration and smoothing current while interaction animations own the avatar.
        solved_bones = body_bones + finger_bones
        solved_blends = face_solver.solve(frame.face, frame.frame_width, frame.frame_height)
        if current_mode == "interact":
            # Interaction mode: nothing follows the performer. Send a fixed
            # relaxed pose so the avatar holds still, and let the gesture
            # animation be the only thing that moves it.
            bones = neutral_bones(args.arm_forward) + neutral_hands
            blends = NEUTRAL_BLENDS
        else:
            bones = solved_bones
            blends = solved_blends
        sender.send(bones, blends, current_mode)
        if gesture is not None:
            gesture_state = gesture.update(frame, finger_frame)
            if gesture_state.active != gesture_last:
                # Edge only: Unity consumes this as a one-shot trigger today and
                # as a sustained bool later without any change here.
                sender.send_gesture(gesture.name, gesture_state.active)
                print(
                    "[GESTURE] {} {} {}".format(
                        gesture.name,
                        "ON" if gesture_state.active else "OFF",
                        gesture_state.status()
                        if args.gesture_verbose
                        else "pat_active={}".format(gesture_state.active),
                    ),
                    flush=True,
                )
                gesture_last = gesture_state.active
            if gesture_logger is not None:
                gesture_logger.write(frame.sequence, source[0], gesture_state)
        if face_logger:
            face_logger.write(frame, source[0], face_solver.diagnostics)
        if finger_logger:
            finger_logger.write(frame, finger_frame, source[0], finger_solver.diagnostics)
        now = time.monotonic()
        if now >= report_at:
            if not args.quiet:
                done, total = face_solver.calibration_progress
                arm_done, arm_total = body_solver.depth_calibration_progress
                left_depth, right_depth = body_solver.arm_depths
                body_yaw, body_yaw_target, turn_confidence, turn_misses = body_solver.body_turn_status
                left_conf = min(frame.body.keypoints[index][2] for index in (7, 9)) if frame.body else 0.0
                right_conf = min(frame.body.keypoints[index][2] for index in (8, 10)) if frame.body else 0.0
                finger_lag = (
                    (frame.sequence - finger_frame.sequence) & 0xFFFFFFFF if finger_frame is not None else -1
                )
                board_prog, board_out = board_gesture_tuning(board_gesture_frame)
                pose_lag = (
                    (frame.capture_time_ns - pose_capture_ns) / 1_000_000.0 if pose_capture_ns > 0 else -1
                )
                wrist_status = "/".join(
                    side + ":" + ("OK" if finger_solver.diagnostics.get(side, {}).get("wrist3d", {}).get("valid") else "hold/fallback")
                    for side in ("Left", "Right")
                ) if args.rgb_hand_orientation_only else "off"
                print(
                    "src={} seq={} track={} accepted={} rejected={} ignored_tracks={} body={} pose3d={} rgb_skew_ms={:.1f} "
                    "body_yaw={:+.1f}->{:+.1f} turn_conf={:.2f} turn_miss={} arm_depth=L{:+.1f}/R{:+.1f} "
                    "arm_conf=L{:.2f}/R{:.2f} arm_cal={}/{} rgb={}/{} rgb_err={} face={} face_cal={}/{} "
                    "hand3d={}/{} hand3d_err={} wrist3d={} hands={} sides={} finger_lag={} finger_rx={}/{} {} "
                    "board_pat={} board_prog={} board_out={} board_rx={}/{}".format(
                        source[0], frame.sequence, frame.track_id, accepted, rejected, ignored_tracks,
                        frame.body is not None,
                        pose3d is not None, pose_lag,
                        body_yaw, body_yaw_target, turn_confidence, turn_misses,
                        left_depth, right_depth, left_conf, right_conf, arm_done, arm_total,
                        rgb_receiver.inferred if rgb_receiver else 0,
                        rgb_receiver.received if rgb_receiver else 0,
                        rgb_receiver.errors if rgb_receiver else 0,
                        frame.face is not None, done, total,
                        rgb_receiver.hand_inferred if rgb_receiver else 0,
                        rgb_receiver.received if rgb_receiver else 0,
                        rgb_receiver.hand_errors if rgb_receiver else 0,
                        wrist_status,
                        len(finger_frame.hands) if finger_frame else 0,
                        "/".join(finger_solver.assigned_sides) or "-",
                        finger_lag, finger_accepted, finger_rejected,
                        gesture_state.status() if gesture_state is not None else "pat_active=off",
                        board_gesture_summary(board_gesture_last),
                        board_prog, board_out,
                        board_gesture_accepted, board_gesture_rejected,
                    )
                )
            report_at = now + 1.0


def run_demo(args: argparse.Namespace) -> None:
    sender = VmcSender(args.vmc_host, args.vmc_port)
    print("Sending VMC demo to {}:{} (Ctrl+C to stop)".format(args.vmc_host, args.vmc_port))
    while True:
        t = time.monotonic() - sender.started
        if args.demo_seconds > 0.0 and t >= args.demo_seconds:
            return
        wave = 35.0 * math.sin(t * 2.0)
        blink = max(0.0, math.sin(t * 3.0) - 0.8) * 5.0
        bones = tuple(
            (
                name,
                arm_quaternion("Left", IDLE_ANGLES[name] + wave, args.arm_forward)
                if name == "LeftUpperArm"
                else arm_quaternion("Right", IDLE_ANGLES[name], args.arm_forward)
                if name == "RightUpperArm"
                else quaternion_z(IDLE_ANGLES.get(name, 0.0)),
            )
            for name in BODY_BONES
        )
        sender.send(bones, {"Blink_L": blink, "Blink_R": blink, "A": 0.0, "Joy": 0.0})
        time.sleep(1.0 / 30.0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0", help="DMV1 listening address")
    parser.add_argument("--port", type=int, default=39540, help="DMV1 listening port")
    parser.add_argument("--finger-port", type=int, default=39542, help="FNG1 listening port")
    parser.add_argument("--no-fingers", action="store_true", help="disable FNG1 input and finger bones")
    parser.add_argument(
        "--finger-sync-wait", type=float, default=0.006,
        help="seconds to wait for the same-sequence FNG1 packet",
    )
    parser.add_argument(
        "--finger-max-lag", type=int, default=0,
        help="older FNG1 sequence lag accepted when the exact packet is late; 0 requires same-frame data",
    )
    parser.add_argument(
        "--finger-confidence", type=float, default=0.25,
        help="minimum hand keypoint confidence",
    )
    parser.add_argument(
        "--finger-smoothing", type=float, default=0.35,
        help="finger curl EMA response in (0, 1]",
    )
    parser.add_argument(
        "--finger-hold-frames", type=int, default=5,
        help="frames to hold a missing finger joint before relaxing it",
    )
    parser.add_argument(
        "--mode", choices=("follow", "interact"), default="follow",
        help="follow: the avatar mirrors the performer. "
             "interact: the avatar holds still and only reacts to the gesture",
    )
    parser.add_argument("--mode-control-port", type=int, default=39544,
                        help="board mode-control UDP port; 0 disables remote switching")
    parser.add_argument("--mode-control-host", default="",
                        help="optional allowed board IPv4 address; default accepts the trusted LAN")
    parser.add_argument(
        "--gesture", action="store_true",
        help="detect the head-pat proxy gesture and publish a signal bit to Unity",
    )
    parser.add_argument("--gesture-name", default="touch_head", help="gesture name sent to Unity")
    parser.add_argument(
        "--gesture-dwell", type=float, default=1.0,
        help="seconds the hand must stay inside the head region",
    )
    parser.add_argument(
        "--gesture-above", type=float, default=1.3,
        help="head-region extent above the nose, in head widths",
    )
    parser.add_argument(
        "--gesture-width", type=float, default=0.8,
        help="head-region half width, in head widths",
    )
    parser.add_argument(
        "--gesture-confidence", type=float, default=0.3,
        help="minimum nose and hand confidence for the gesture detector",
    )
    parser.add_argument(
        "--gesture-log", default="",
        help="write per-frame gesture calibration data to JSONL",
    )
    parser.add_argument(
        "--gesture-port", type=int, default=39543,
        help="GST1 port carrying the board's PS-side gesture signals",
    )
    parser.add_argument(
        "--gesture-source", choices=("board", "pc", "both"), default="board",
        help="board: use PS gesture signals (default); pc: detect locally; both: A/B test",
    )
    parser.add_argument(
        "--quiet", action="store_true",
        help="suppress the per-second status line; only gesture edges are printed",
    )
    parser.add_argument(
        "--gesture-verbose", action="store_true",
        help="include the full pat_* diagnostics on the gesture edge line",
    )
    parser.add_argument("--vmc-host", default="127.0.0.1", help="Unity/EVMC4U address")
    parser.add_argument("--vmc-port", type=int, default=39539, help="Unity/EVMC4U VMC port")
    parser.add_argument("--confidence", type=float, default=0.25, help="minimum body keypoint confidence")
    parser.add_argument("--smoothing", type=float, default=0.35, help="EMA response in (0, 1]")
    parser.add_argument(
        "--arm-forward", type=float, default=10.0,
        help="fallback shoulder depth in degrees when 3D lifting is unavailable",
    )
    parser.add_argument(
        "--pose3d-model", default="",
        help="VideoPose3D COCO checkpoint; enables temporal arm-depth estimation",
    )
    parser.add_argument(
        "--rgb-pose-model", default="",
        help="MediaPipe Pose Landmarker bundle; uses RGB pixels for arm depth",
    )
    parser.add_argument(
        "--rgb-hand-model", default="",
        help="MediaPipe Hand Landmarker bundle; enables passive 3D hand diagnostics",
    )
    parser.add_argument(
        "--rgb-hand-log", default="",
        help="write passive MediaPipe 3D hand diagnostics to JSONL",
    )
    parser.add_argument(
        "--rgb-hand-drive", action="store_true",
        help="use MediaPipe 3D palm orientation and curl fallback for hand bones",
    )
    parser.add_argument("--rgb-hand-orientation-only", action="store_true",
                        help="RGB wrist twist only; retain NPU finger curls and wrist direction")
    parser.add_argument("--rgb-arm-depth-only", action="store_true",
                        help="RGB arm depth only; retain all NPU projected directions and torso/leg mapping")
    parser.add_argument("--rgb-body-turn", action="store_true",
                        help="add RGB torso yaw to --rgb-arm-depth-only; preserve camera-space limb targets")
    parser.add_argument("--rgb-max-skew-ms", type=float, default=150.0,
                        help="maximum capture-time mismatch for conservative RGB enhancements")
    parser.add_argument("--rgb-port", type=int, default=39541, help="board JPEG fragment UDP port")
    parser.add_argument(
        "--rgb-pose-max-age", type=float, default=0.5,
        help="maximum age in seconds of the newest RGB 3D estimate",
    )
    parser.add_argument(
        "--no-body-turn", action="store_true",
        help="keep the avatar facing forward while retaining RGB arm depth",
    )
    parser.add_argument(
        "--arm-depth-scale", type=float, default=1.0,
        help="3D arm-depth gain; use -1 to reverse front/back",
    )
    parser.add_argument(
        "--arm-depth-calibration-frames", type=int, default=30,
        help="neutral-pose frames used to remove per-camera depth bias",
    )
    parser.add_argument(
        "--min-segment-length", type=float, default=0.02,
        help="ignore shorter normalized limb segments as occlusion noise",
    )
    parser.add_argument("--calibration-frames", type=int, default=30, help="neutral face frames to collect")
    parser.add_argument("--face-log", default="", help="write per-frame face diagnostics to this CSV file")
    parser.add_argument("--finger-log", default="", help="write raw hand landmarks and V2 features to JSONL")
    parser.add_argument("--no-mirror", dest="mirror", action="store_false", help="disable camera mirror correction")
    parser.add_argument("--demo", action="store_true", help="send a synthetic wave directly to Unity")
    parser.add_argument("--demo-seconds", type=float, default=0.0, help="stop demo after this many seconds")
    parser.set_defaults(mirror=True)
    return parser


def main() -> None:
    # 输出被重定向（管道/重定向到文件）时 Python 会切成块缓冲，看起来就像
    # "程序卡住不输出"。强制行缓冲，保证每条日志立刻出现。
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except (AttributeError, ValueError):
        pass
    args = build_parser().parse_args()
    print("[MAPPING wrist-depth-20261022] source={} wrist_only={} arm_depth_only={}".format(
        Path(__file__).resolve(), args.rgb_hand_orientation_only, args.rgb_arm_depth_only), flush=True)
    print("[MAPPING body-turn-20261022] enabled={} (conservative torso yaw)".format(args.rgb_body_turn), flush=True)
    if not 20 <= args.rgb_max_skew_ms <= 500:
        raise SystemExit("--rgb-max-skew-ms must be in [20, 500]")
    if args.rgb_hand_orientation_only and args.rgb_hand_drive:
        raise SystemExit("choose orientation-only or legacy full hand drive, not both")
    if args.rgb_hand_orientation_only and not args.rgb_hand_model:
        raise SystemExit("--rgb-hand-orientation-only requires --rgb-hand-model")
    if args.rgb_arm_depth_only and not args.rgb_pose_model:
        raise SystemExit("--rgb-arm-depth-only requires --rgb-pose-model")
    if args.rgb_body_turn and (not args.rgb_arm_depth_only or args.no_body_turn):
        raise SystemExit("--rgb-body-turn requires --rgb-arm-depth-only and cannot use --no-body-turn")
    if (args.rgb_arm_depth_only or args.rgb_hand_orientation_only) and args.pose3d_model:
        raise SystemExit("conservative RGB modes cannot be mixed with the legacy VideoPose3D lifter")
    if not 0.0 < args.smoothing <= 1.0:
        raise SystemExit("--smoothing must be in (0, 1]")
    if not 0.0 <= args.arm_forward <= 30.0:
        raise SystemExit("--arm-forward must be in [0, 30]")
    if not -3.0 <= args.arm_depth_scale <= 3.0 or args.arm_depth_scale == 0.0:
        raise SystemExit("--arm-depth-scale must be in [-3, 3] and non-zero")
    if not 0 <= args.arm_depth_calibration_frames <= 300:
        raise SystemExit("--arm-depth-calibration-frames must be in [0, 300]")
    if not 0.0 <= args.min_segment_length <= 0.2:
        raise SystemExit("--min-segment-length must be in [0, 0.2]")
    if not 0.05 <= args.rgb_pose_max_age <= 2.0:
        raise SystemExit("--rgb-pose-max-age must be in [0.05, 2]")
    if args.rgb_hand_log and not args.rgb_hand_model:
        raise SystemExit("--rgb-hand-log requires --rgb-hand-model")
    if args.rgb_hand_model and not args.rgb_pose_model:
        raise SystemExit("--rgb-hand-model requires --rgb-pose-model")
    if args.rgb_hand_drive and not args.rgb_hand_model:
        raise SystemExit("--rgb-hand-drive requires --rgb-hand-model")
    if not 1 <= args.finger_port <= 65535 or args.finger_port == args.port:
        raise SystemExit("--finger-port must be valid and differ from --port")
    if not 0.0 <= args.finger_sync_wait <= 0.05:
        raise SystemExit("--finger-sync-wait must be in [0, 0.05]")
    if not 0 <= args.finger_max_lag <= 10:
        raise SystemExit("--finger-max-lag must be in [0, 10]")
    if not 0.0 <= args.finger_confidence <= 1.0:
        raise SystemExit("--finger-confidence must be in [0, 1]")
    if not 0.0 < args.finger_smoothing <= 1.0:
        raise SystemExit("--finger-smoothing must be in (0, 1]")
    if not 0 <= args.finger_hold_frames <= 30:
        raise SystemExit("--finger-hold-frames must be in [0, 30]")
    if not 0 <= args.mode_control_port <= 65535:
        raise SystemExit("--mode-control-port must be in [0, 65535]")
    if args.mode_control_port and args.mode_control_port in (
        args.port, args.finger_port, args.rgb_port, args.gesture_port
    ):
        raise SystemExit("--mode-control-port must differ from all data ports")
    if args.mode_control_host:
        try:
            socket.inet_pton(socket.AF_INET, args.mode_control_host)
        except OSError:
            raise SystemExit("--mode-control-host must be a numeric IPv4 address")
    if args.mode_control_port and args.gesture_source in ("pc", "both"):
        args.gesture = True
    if args.gesture_log and not args.gesture:
        raise SystemExit("--gesture-log requires --gesture")
    if args.gesture_source in ("board", "both"):
        if not 1 <= args.gesture_port <= 65535:
            raise SystemExit("--gesture-port must be valid")
        if args.gesture_port in (args.port, args.finger_port, args.rgb_port):
            raise SystemExit("--gesture-port must differ from the motion/finger/rgb ports")
    if args.mode == "interact" and not args.gesture:
        if args.gesture_source in ("pc", "both"):
            print("交互模式必须开启手势判定，已自动启用 --gesture。")
            args.gesture = True
        else:
            print("交互模式：板端 PS 负责手势判定（GST1 {}）。".format(args.gesture_port))
    if args.gesture_dwell <= 0.0:
        raise SystemExit("--gesture-dwell must be positive")
    if args.gesture_width <= 0.0:
        raise SystemExit("--gesture-width must be positive")
    if args.gesture_above < 0.0:
        raise SystemExit("--gesture-above must be non-negative")
    if args.demo:
        run_demo(args)
    else:
        run_bridge(args)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nstopped")
