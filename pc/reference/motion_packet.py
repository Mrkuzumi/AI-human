"""Reference codec for DMV2 multi-person packets with DMV1 compatibility."""

from __future__ import annotations

import math
import struct
import zlib
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple


MAGIC = b"DMV2"
VERSION = 2
LEGACY_MAGIC = b"DMV1"
LEGACY_VERSION = 1
FLAG_BODY = 1 << 0
FLAG_FACE = 1 << 1
FLAG_NORMALIZED = 1 << 2
POSE_COUNT = 17
FACE_COUNT = 51

_HEADER_V1 = struct.Struct("<4sBBHIIQQHHBBBB")
_HEADER = struct.Struct("<4sBBHIIQQHHBBBBI")
_DETECTION = struct.Struct("<fffff")
_POINT = struct.Struct("<fff")
_CRC = struct.Struct("<I")

HEADER_BYTES = _HEADER.size
LEGACY_HEADER_BYTES = _HEADER_V1.size
EMPTY_PACKET_BYTES = HEADER_BYTES + _CRC.size
FULL_PACKET_BYTES = HEADER_BYTES + _DETECTION.size * 2 + (POSE_COUNT + FACE_COUNT) * _POINT.size + _CRC.size

Point = Tuple[float, float, float]
Box = Tuple[float, float, float, float]


@dataclass(frozen=True)
class Detection:
    score: float
    box: Box
    keypoints: Tuple[Point, ...]


@dataclass(frozen=True)
class MotionFrame:
    sequence: int
    capture_time_ns: int
    emit_time_ns: int
    frame_width: int = 1920
    frame_height: int = 1080
    source_id: int = 0
    track_id: int = 1
    body: Optional[Detection] = None
    face: Optional[Detection] = None


def _unit_float(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value) or not 0.0 <= value <= 1.0:
        raise ValueError("{} must be finite and in [0, 1]".format(name))
    return value


def _validate_detection(detection: Detection, count: int, name: str) -> None:
    _unit_float(detection.score, name + ".score")
    if len(detection.box) != 4:
        raise ValueError(name + ".box must contain x, y, w, h")
    for index, value in enumerate(detection.box):
        _unit_float(value, "{}.box[{}]".format(name, index))
    if len(detection.keypoints) != count:
        raise ValueError("{} must contain {} keypoints".format(name, count))
    for point_index, point in enumerate(detection.keypoints):
        if len(point) != 3:
            raise ValueError("{}.keypoints[{}] must contain x, y, confidence".format(name, point_index))
        for value_index, value in enumerate(point):
            _unit_float(value, "{}.keypoints[{}][{}]".format(name, point_index, value_index))


def _append_detection(payload: bytearray, detection: Detection) -> None:
    payload.extend(_DETECTION.pack(detection.score, *detection.box))
    for point in detection.keypoints:
        payload.extend(_POINT.pack(*point))


def pack_frame(frame: MotionFrame) -> bytes:
    """Serialize one frame. Raises ValueError instead of emitting an invalid packet."""
    if not 0 <= frame.sequence <= 0xFFFFFFFF:
        raise ValueError("sequence must fit uint32")
    if not 0 <= frame.capture_time_ns <= 0xFFFFFFFFFFFFFFFF:
        raise ValueError("capture_time_ns must fit uint64")
    if not frame.capture_time_ns <= frame.emit_time_ns <= 0xFFFFFFFFFFFFFFFF:
        raise ValueError("emit_time_ns must be uint64 and not precede capture_time_ns")
    if not 1 <= frame.frame_width <= 0xFFFF or not 1 <= frame.frame_height <= 0xFFFF:
        raise ValueError("frame dimensions must fit positive uint16 values")
    if not 0 <= frame.source_id <= 0xFF:
        raise ValueError("source_id must fit uint8")
    if not 1 <= frame.track_id <= 0xFFFFFFFF:
        raise ValueError("track_id must fit positive uint32")

    flags = FLAG_NORMALIZED
    pose_count = 0
    face_count = 0
    blocks = bytearray()

    if frame.body is not None:
        _validate_detection(frame.body, POSE_COUNT, "body")
        flags |= FLAG_BODY
        pose_count = POSE_COUNT
        _append_detection(blocks, frame.body)
    if frame.face is not None:
        _validate_detection(frame.face, FACE_COUNT, "face")
        flags |= FLAG_FACE
        face_count = FACE_COUNT
        _append_detection(blocks, frame.face)

    packet_bytes = HEADER_BYTES + len(blocks) + _CRC.size
    payload = bytearray(
        _HEADER.pack(
            MAGIC,
            VERSION,
            flags,
            HEADER_BYTES,
            packet_bytes,
            frame.sequence,
            frame.capture_time_ns,
            frame.emit_time_ns,
            frame.frame_width,
            frame.frame_height,
            pose_count,
            face_count,
            frame.source_id,
            0,
            frame.track_id,
        )
    )
    payload.extend(blocks)
    payload.extend(_CRC.pack(zlib.crc32(payload) & 0xFFFFFFFF))
    return bytes(payload)


def _read_detection(data: bytes, offset: int, count: int, name: str) -> Tuple[Detection, int]:
    score, x, y, width, height = _DETECTION.unpack_from(data, offset)
    offset += _DETECTION.size
    points = []
    for _ in range(count):
        points.append(_POINT.unpack_from(data, offset))
        offset += _POINT.size
    detection = Detection(score, (x, y, width, height), tuple(points))
    _validate_detection(detection, count, name)
    return detection, offset


def unpack_frame(data: bytes) -> MotionFrame:
    """Parse and strictly validate one complete UDP datagram."""
    if len(data) < LEGACY_HEADER_BYTES + _CRC.size:
        raise ValueError("packet is shorter than the motion header and CRC")

    magic, version, _flags, header_bytes = struct.unpack_from("<4sBBH", data)
    if (magic, version, header_bytes) == (MAGIC, VERSION, HEADER_BYTES):
        header = _HEADER
    elif (magic, version, header_bytes) == (LEGACY_MAGIC, LEGACY_VERSION, LEGACY_HEADER_BYTES):
        header = _HEADER_V1
    else:
        raise ValueError("unsupported packet magic or version")

    values = header.unpack_from(data)
    (
        magic,
        version,
        flags,
        header_bytes,
        packet_bytes,
        sequence,
        capture_time_ns,
        emit_time_ns,
        frame_width,
        frame_height,
        pose_count,
        face_count,
        source_id,
        _reserved,
    ) = values[:14]
    track_id = values[14] if version == VERSION else 1

    if packet_bytes != len(data):
        raise ValueError("packet length does not match its header")
    if flags & ~(FLAG_BODY | FLAG_FACE | FLAG_NORMALIZED):
        raise ValueError("packet contains unknown flags")
    if not flags & FLAG_NORMALIZED:
        raise ValueError("v1 coordinates must be normalized")
    if pose_count not in (0, POSE_COUNT) or face_count not in (0, FACE_COUNT):
        raise ValueError("invalid v1 keypoint count")
    if bool(flags & FLAG_BODY) != bool(pose_count) or bool(flags & FLAG_FACE) != bool(face_count):
        raise ValueError("presence flags and keypoint counts disagree")
    if not 1 <= track_id <= 0xFFFFFFFF:
        raise ValueError("track_id must fit positive uint32")

    expected_bytes = (
        header_bytes
        + (_DETECTION.size + pose_count * _POINT.size if pose_count else 0)
        + (_DETECTION.size + face_count * _POINT.size if face_count else 0)
        + _CRC.size
    )
    if len(data) != expected_bytes:
        raise ValueError("packet length does not match its detection blocks")

    expected_crc = _CRC.unpack_from(data, len(data) - _CRC.size)[0]
    actual_crc = zlib.crc32(data[:-_CRC.size]) & 0xFFFFFFFF
    if actual_crc != expected_crc:
        raise ValueError("CRC32 mismatch")

    offset = header_bytes
    body = None
    face = None
    if pose_count:
        body, offset = _read_detection(data, offset, pose_count, "body")
    if face_count:
        face, offset = _read_detection(data, offset, face_count, "face")
    if offset != len(data) - _CRC.size:
        raise ValueError("unexpected bytes before CRC32")

    return MotionFrame(
        sequence=sequence,
        capture_time_ns=capture_time_ns,
        emit_time_ns=emit_time_ns,
        frame_width=frame_width,
        frame_height=frame_height,
        source_id=source_id,
        track_id=track_id,
        body=body,
        face=face,
    )

