"""Receive board JPEG fragments and estimate RGB-based 3D pose with MediaPipe."""

from __future__ import annotations

import json
import math
import select
import socket
import struct
import threading
import time
from pathlib import Path
from typing import Dict, Optional, Sequence, Tuple


MAGIC = b"DMI1"
VERSION = 1
_HEADER = struct.Struct("<4sBBHIIHHQ")
HEADER_BYTES = _HEADER.size
MAX_FRAME_BYTES = 1_000_000
Pose3D = Tuple[Tuple[float, ...], ...]


class JpegFrameReassembler:
    """Reassemble bounded, unordered UDP fragments into complete JPEG frames."""

    def __init__(self, timeout: float = 0.5, max_frames: int = 4) -> None:
        self.timeout = timeout
        self.max_frames = max_frames
        self._frames: Dict[Tuple[int, int], dict] = {}

    def push(self, packet: bytes, now: Optional[float] = None) -> Optional[Tuple[int, int, bytes]]:
        if len(packet) <= HEADER_BYTES:
            return None
        magic, version, source_id, header_bytes, sequence, frame_bytes, part, count, capture_ns = (
            _HEADER.unpack_from(packet)
        )
        payload = packet[HEADER_BYTES:]
        if (
            magic != MAGIC
            or version != VERSION
            or header_bytes != HEADER_BYTES
            or not 0 < frame_bytes <= MAX_FRAME_BYTES
            or not 0 < count <= 0xFFFF
            or part >= count
            or not payload
        ):
            return None

        now = time.monotonic() if now is None else now
        for key, pending in tuple(self._frames.items()):
            if now - pending["updated"] > self.timeout:
                del self._frames[key]
        key = source_id, sequence
        pending = self._frames.get(key)
        if pending is None:
            if len(self._frames) >= self.max_frames:
                oldest = min(self._frames, key=lambda item: self._frames[item]["updated"])
                del self._frames[oldest]
            pending = {
                "capture_ns": capture_ns,
                "frame_bytes": frame_bytes,
                "parts": [None] * count,
                "updated": now,
            }
            self._frames[key] = pending
        elif (
            pending["frame_bytes"] != frame_bytes
            or len(pending["parts"]) != count
            or pending["capture_ns"] != capture_ns
        ):
            del self._frames[key]
            return None

        pending["parts"][part] = payload
        pending["updated"] = now
        if any(value is None for value in pending["parts"]):
            return None
        jpeg = b"".join(pending["parts"])
        del self._frames[key]
        if len(jpeg) != frame_bytes or not jpeg.startswith(b"\xff\xd8"):
            return None
        return sequence, capture_ns, jpeg


def world_landmarks_to_h36m(points: Sequence[Sequence[float]]) -> Pose3D:
    """Map MediaPipe's 33 world landmarks to the H36M-17 layout used by BodySolver."""
    if len(points) != 33:
        raise ValueError("MediaPipe pose must contain 33 landmarks")

    def point(index: int) -> Tuple[float, ...]:
        # MediaPipe supplies visibility as axis 3. VideoPose3D callers still use xyz.
        return tuple(float(axis) for axis in points[index][: min(4, len(points[index]))])

    def midpoint(first: int, second: int) -> Tuple[float, ...]:
        xyz = tuple((points[first][axis] + points[second][axis]) * 0.5 for axis in range(3))
        if len(points[first]) < 4 or len(points[second]) < 4:
            return xyz
        return xyz + (min(float(points[first][3]), float(points[second][3])),)

    pelvis = midpoint(23, 24)
    thorax = midpoint(11, 12)
    spine = tuple((pelvis[axis] + thorax[axis]) * 0.5 for axis in range(3))
    return (
        pelvis,
        point(24), point(26), point(28),
        point(23), point(25), point(27),
        spine, thorax, thorax, point(0),
        point(11), point(13), point(15),
        point(12), point(14), point(16),
    )


def palm_normal(points: Sequence[Sequence[float]]) -> Tuple[float, float, float]:
    """Return the unit palm normal from wrist, index MCP and pinky MCP."""
    if len(points) != 21:
        raise ValueError("MediaPipe hand must contain 21 landmarks")
    wrist, index, pinky = points[0], points[5], points[17]
    first = tuple(index[axis] - wrist[axis] for axis in range(3))
    second = tuple(pinky[axis] - wrist[axis] for axis in range(3))
    cross = (
        first[1] * second[2] - first[2] * second[1],
        first[2] * second[0] - first[0] * second[2],
        first[0] * second[1] - first[1] * second[0],
    )
    length = math.sqrt(sum(axis * axis for axis in cross))
    if length < 1e-6:
        return 0.0, 0.0, 0.0
    return tuple(axis / length for axis in cross)


class MediaPipePose3D:
    def __init__(self, model_path: str) -> None:
        import mediapipe as mp

        path = Path(model_path)
        if not path.is_file():
            raise FileNotFoundError("MediaPipe pose model not found: {}".format(path))
        options = mp.tasks.vision.PoseLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=str(path)),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=0.5,
            min_pose_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self._mp = mp
        self._landmarker = mp.tasks.vision.PoseLandmarker.create_from_options(options)
        self._timestamp_ms = -1
        self.image_landmarks = ()

    def estimate(self, jpeg: bytes, capture_ns: int) -> Optional[Pose3D]:
        import cv2
        import numpy as np

        bgr = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            return None
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        timestamp_ms = max(self._timestamp_ms + 1, capture_ns // 1_000_000)
        self._timestamp_ms = timestamp_ms
        image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=rgb)
        result = self._landmarker.detect_for_video(image, timestamp_ms)
        self.image_landmarks = ()
        if not result.pose_world_landmarks:
            return None
        self.image_landmarks = world_landmarks_to_h36m(tuple(
            (p.x, p.y, p.z, min(float(p.visibility), float(p.presence)))
            for p in result.pose_landmarks[0]
        ))
        points = tuple(
            (
                landmark.x,
                landmark.y,
                landmark.z,
                min(
                    float(getattr(landmark, "visibility", 1.0)),
                    float(getattr(landmark, "presence", 1.0)),
                ),
            )
            for landmark in result.pose_world_landmarks[0]
        )
        return world_landmarks_to_h36m(points)


class MediaPipeHand3D:
    """Estimate passive 3D hand landmarks without changing avatar output."""

    def __init__(self, model_path: str) -> None:
        import mediapipe as mp

        path = Path(model_path)
        if not path.is_file():
            raise FileNotFoundError("MediaPipe hand model not found: {}".format(path))
        options = mp.tasks.vision.HandLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=str(path)),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_hands=2,
            min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self._mp = mp
        self._landmarker = mp.tasks.vision.HandLandmarker.create_from_options(options)
        self._timestamp_ms = -1

    def estimate(self, jpeg: bytes, capture_ns: int) -> list[dict]:
        import cv2
        import numpy as np

        bgr = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            return []
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        timestamp_ms = max(self._timestamp_ms + 1, capture_ns // 1_000_000)
        self._timestamp_ms = timestamp_ms
        image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=rgb)
        result = self._landmarker.detect_for_video(image, timestamp_ms)
        hands = []
        for index, landmarks in enumerate(result.hand_landmarks):
            world = tuple(
                (float(point.x), float(point.y), float(point.z))
                for point in result.hand_world_landmarks[index]
            )
            image_points = tuple(
                (float(point.x), float(point.y), float(point.z)) for point in landmarks
            )
            category = result.handedness[index][0]
            hands.append(
                {
                    # shortcut: keep raw handedness until camera mirroring is calibrated.
                    "handedness": category.category_name,
                    "score": float(category.score),
                    "palm_normal": palm_normal(world),
                    "landmarks": image_points,
                    "world_landmarks": world,
                }
            )
        return hands


class Hand3DJsonlLogger:
    def __init__(self, path: str) -> None:
        output = Path(path).expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        self.path = output
        self._file = output.open("w", encoding="utf-8")

    def write(self, sequence: int, capture_ns: int, hands: Sequence[dict],
              pose: Optional[Pose3D] = None, image_points: Sequence = ()) -> None:
        record = {
            "receive_time_ns": time.time_ns(),
            "sequence": sequence,
            "capture_time_ns": capture_ns,
            "hands": hands,
            "pose_world_h36m": pose,
            "pose_image_h36m": image_points,
        }
        self._file.write(json.dumps(record, separators=(",", ":")) + "\n")
        self._file.flush()


class RgbPoseReceiver(threading.Thread):
    """Background image receiver; the motion loop reads only the newest 3D pose."""

    def __init__(
        self,
        bind: str,
        port: int,
        model_path: str,
        hand_model_path: str = "",
        hand_log_path: str = "",
    ) -> None:
        super().__init__(name="rgb-pose-receiver", daemon=True)
        self.bind = bind
        self.port = port
        self.estimator = MediaPipePose3D(model_path)
        self.hand_estimator = MediaPipeHand3D(hand_model_path) if hand_model_path else None
        self.hand_logger = Hand3DJsonlLogger(hand_log_path) if hand_log_path else None
        self.reassembler = JpegFrameReassembler()
        self._lock = threading.Lock()
        self._pose: Optional[Pose3D] = None
        self._pose_sequence = -1
        self._pose_capture_ns = 0
        self._updated = 0.0
        self._hands: Tuple[dict, ...] = ()
        self._hand_sequence = -1
        self._hand_updated = 0.0
        self.received = 0
        self.inferred = 0
        self.errors = 0
        self.hand_inferred = 0
        self.hand_errors = 0
        self._pose_observation = None
        self._hand_observation = None
        # Bind synchronously so an occupied RGB port fails visibly at startup,
        # rather than leaving the motion bridge running without a receiver.
        self._receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._receiver.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1_048_576)
        self._receiver.bind((self.bind, self.port))
        self._receiver.settimeout(.5)

    def latest_observations(self, max_age: float = .5):
        """Return image association and capture metadata atomically with each result."""
        with self._lock:
            now = time.monotonic()
            return (self._pose_observation if now - self._updated <= max_age else None,
                    self._hand_observation if now - self._hand_updated <= max_age else None)

    def latest(self, max_age: float = 0.5) -> Optional[Pose3D]:
        with self._lock:
            if time.monotonic() - self._updated > max_age:
                return None
            return self._pose

    def latest_frame(self, max_age: float = 0.5) -> Tuple[int, int, Optional[Pose3D]]:
        with self._lock:
            if time.monotonic() - self._updated > max_age:
                return -1, 0, None
            return self._pose_sequence, self._pose_capture_ns, self._pose

    def latest_hands(self, max_age: float = 0.5) -> Tuple[int, Tuple[dict, ...]]:
        with self._lock:
            if time.monotonic() - self._hand_updated > max_age:
                return -1, ()
            return self._hand_sequence, self._hands

    def run(self) -> None:
        receiver = self._receiver
        while True:
            try:
                packet, _source = receiver.recvfrom(2048)
                complete = self.reassembler.push(packet)
                # Inference must consume the newest complete image, not build up
                # a queue of old JPEGs behind the much faster NPU motion stream.
                newest = (complete, packet[5], _source[0]) if complete is not None else None
                for _ in range(4096):
                    if not select.select([receiver], [], [], 0)[0]:
                        break
                    packet, _source = receiver.recvfrom(2048)
                    complete = self.reassembler.push(packet)
                    if complete is not None and (newest is None or complete[1] >= newest[0][1]):
                        newest = complete, packet[5], _source[0]
                if newest is None:
                    continue
                complete, source_id, source_host = newest
                sequence, capture_ns, jpeg = complete
                metadata = dict(sequence=sequence, capture_ns=capture_ns,
                                source_id=source_id, source_host=source_host)
                self.received += 1
                pose = self.estimator.estimate(jpeg, capture_ns)
                if pose is not None:
                    with self._lock:
                        self._pose = pose
                        self._pose_sequence = sequence
                        self._pose_capture_ns = capture_ns
                        self._updated = time.monotonic()
                        self._pose_observation = dict(metadata, points=pose,
                                                      image_points=self.estimator.image_landmarks)
                    self.inferred += 1
                if self.hand_estimator is not None:
                    try:
                        hands = self.hand_estimator.estimate(jpeg, capture_ns)
                        with self._lock:
                            self._hands = tuple(hands)
                            self._hand_sequence = sequence
                            self._hand_updated = time.monotonic()
                            self._hand_observation = dict(metadata, hands=tuple(hands))
                        if hands:
                            self.hand_inferred += 1
                        if self.hand_logger is not None:
                            self.hand_logger.write(sequence, capture_ns, hands, pose,
                                                   self.estimator.image_landmarks)
                    except Exception as error:
                        self.hand_errors += 1
                        if self.hand_errors == 1 or self.hand_errors % 30 == 0:
                            print("RGB hand error ({} total): {}".format(self.hand_errors, error))
            except socket.timeout:
                continue
            except Exception as error:
                self.errors += 1
                if self.errors == 1 or self.errors % 30 == 0:
                    print("RGB pose error ({} total): {}".format(self.errors, error))
