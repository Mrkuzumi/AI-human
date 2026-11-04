"""Strict codec for the board-to-PC FNG1 hand-landmark packet."""

from __future__ import annotations

import math
import struct
import zlib
from dataclasses import dataclass
from typing import Tuple

from motion_packet import Detection


MAGIC = b"FNG1"
VERSION = 1
FLAG_NORMALIZED = 1 << 2
POINT_COUNT = 21
MAX_HANDS = 2

_HEADER = struct.Struct("<4sBBHIIQQHHBBBB")
_HAND = struct.Struct("<68f")
_CRC = struct.Struct("<I")

HEADER_BYTES = _HEADER.size
EMPTY_PACKET_BYTES = HEADER_BYTES + _CRC.size
HAND_BYTES = _HAND.size


@dataclass(frozen=True)
class FingerFrame:
    sequence: int
    capture_time_ns: int
    emit_time_ns: int
    frame_width: int
    frame_height: int
    source_id: int
    hands: Tuple[Detection, ...]


def _unit_float(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value) or not 0.0 <= value <= 1.0:
        raise ValueError("{} must be finite and in [0, 1]".format(name))
    return value


def _validate_hand(hand: Detection, index: int) -> None:
    name = "hands[{}]".format(index)
    _unit_float(hand.score, name + ".score")
    if len(hand.box) != 4:
        raise ValueError(name + ".box must contain x, y, w, h")
    for value_index, value in enumerate(hand.box):
        _unit_float(value, "{}.box[{}]".format(name, value_index))
    if len(hand.keypoints) != POINT_COUNT:
        raise ValueError("{} must contain {} keypoints".format(name, POINT_COUNT))
    for point_index, point in enumerate(hand.keypoints):
        if len(point) != 3:
            raise ValueError("{}.keypoints[{}] must contain x, y, confidence".format(name, point_index))
        for value_index, value in enumerate(point):
            _unit_float(value, "{}.keypoints[{}][{}]".format(name, point_index, value_index))


def pack_finger_frame(frame: FingerFrame) -> bytes:
    """Serialize FNG1 for tests and diagnostics using the board wire format."""
    if not 0 <= frame.sequence <= 0xFFFFFFFF:
        raise ValueError("sequence must fit uint32")
    if not 0 <= frame.capture_time_ns <= frame.emit_time_ns <= 0xFFFFFFFFFFFFFFFF:
        raise ValueError("timestamps must fit uint64 and emit must not precede capture")
    if not 1 <= frame.frame_width <= 0xFFFF or not 1 <= frame.frame_height <= 0xFFFF:
        raise ValueError("frame dimensions must fit positive uint16 values")
    if not 0 <= frame.source_id <= 0xFF:
        raise ValueError("source_id must fit uint8")
    if len(frame.hands) > MAX_HANDS:
        raise ValueError("FNG1 supports at most two hands")

    blocks = bytearray()
    for index, hand in enumerate(frame.hands):
        _validate_hand(hand, index)
        values = [hand.score, *hand.box]
        for point in hand.keypoints:
            values.extend(point)
        blocks.extend(_HAND.pack(*values))

    packet_bytes = EMPTY_PACKET_BYTES + len(blocks)
    payload = bytearray(
        _HEADER.pack(
            MAGIC,
            VERSION,
            FLAG_NORMALIZED,
            HEADER_BYTES,
            packet_bytes,
            frame.sequence,
            frame.capture_time_ns,
            frame.emit_time_ns,
            frame.frame_width,
            frame.frame_height,
            len(frame.hands),
            POINT_COUNT,
            frame.source_id,
            0,
        )
    )
    payload.extend(blocks)
    payload.extend(_CRC.pack(zlib.crc32(payload) & 0xFFFFFFFF))
    return bytes(payload)


def unpack_finger_frame(data: bytes) -> FingerFrame:
    """Parse and strictly validate one complete FNG1 UDP datagram."""
    if len(data) < EMPTY_PACKET_BYTES:
        raise ValueError("packet is shorter than the FNG1 header and CRC")

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
        hand_count,
        point_count,
        source_id,
        reserved,
    ) = _HEADER.unpack_from(data)

    if (magic, version, flags) != (MAGIC, VERSION, FLAG_NORMALIZED):
        raise ValueError("unsupported FNG1 magic, version, or flags")
    if header_bytes != HEADER_BYTES or packet_bytes != len(data):
        raise ValueError("packet length does not match its header")
    if hand_count > MAX_HANDS or point_count != POINT_COUNT or reserved != 0:
        raise ValueError("invalid FNG1 hand count, keypoint count, or reserved byte")
    if frame_width == 0 or frame_height == 0:
        raise ValueError("invalid frame dimensions")
    if emit_time_ns < capture_time_ns:
        raise ValueError("emit timestamp precedes capture timestamp")
    if len(data) != EMPTY_PACKET_BYTES + hand_count * HAND_BYTES:
        raise ValueError("packet length does not match its hand blocks")

    expected_crc = _CRC.unpack_from(data, len(data) - _CRC.size)[0]
    actual_crc = zlib.crc32(data[:-_CRC.size]) & 0xFFFFFFFF
    if expected_crc != actual_crc:
        raise ValueError("CRC32 mismatch")

    hands = []
    offset = HEADER_BYTES
    for index in range(hand_count):
        values = _HAND.unpack_from(data, offset)
        offset += HAND_BYTES
        hand = Detection(
            score=values[0],
            box=tuple(values[1:5]),
            keypoints=tuple(tuple(values[5 + point * 3 : 8 + point * 3]) for point in range(POINT_COUNT)),
        )
        _validate_hand(hand, index)
        hands.append(hand)

    return FingerFrame(
        sequence=sequence,
        capture_time_ns=capture_time_ns,
        emit_time_ns=emit_time_ns,
        frame_width=frame_width,
        frame_height=frame_height,
        source_id=source_id,
        hands=tuple(hands),
    )
