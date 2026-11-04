"""Reference codec for GST1 gesture-signal packets.

The board evaluates gestures on its PS side and publishes the state of every
gesture it knows about once per frame.  This module is the PC end of that
contract: it validates the datagram strictly and maps gesture ids back to the
names Unity expects on ``/Avatar/Gesture``.

Adding a gesture is additive: reserve an id in
``three_*_board/include/gesture_udp_sender.hpp``, add it to :data:`GESTURE_IDS`
here, and the bridge will forward it without any further change.
"""

from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass
from typing import Dict, Tuple

MAGIC = b"GST1"
VERSION = 1
HEADER_BYTES = 48
MAX_RECORDS = 32
FLAG_METRIC_VALID = 1 << 0

_HEADER = struct.Struct("<4sBBHIIQQHHBBHII")
_RECORD = struct.Struct("<HBBff")
_CRC = struct.Struct("<I")

RECORD_BYTES = _RECORD.size
EMPTY_PACKET_BYTES = HEADER_BYTES + _CRC.size

# Wire contract with the board; keep in sync with gesture_udp_sender.hpp.
GESTURE_IDS: Dict[int, str] = {
    0: "touch_head",
    1: "touch_face",
    2: "hug",
}
NAME_TO_ID: Dict[str, int] = {name: gesture_id for gesture_id, name in GESTURE_IDS.items()}


@dataclass(frozen=True)
class GestureRecord:
    id: int
    name: str
    active: int
    progress: float
    metric: float
    metric_valid: bool


@dataclass(frozen=True)
class GestureFrame:
    sequence: int
    capture_time_ns: int
    emit_time_ns: int
    frame_width: int
    frame_height: int
    source_id: int
    records: Tuple[GestureRecord, ...]

    def active_ids(self) -> Tuple[int, ...]:
        return tuple(record.id for record in self.records if record.active)


def gesture_name(gesture_id: int) -> str:
    """Name for a gesture id; unknown ids stay visible as ``Gesture<id>``."""
    return GESTURE_IDS.get(gesture_id, "Gesture{}".format(gesture_id))


def pack_frame(frame: GestureFrame) -> bytes:
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
    if len(frame.records) > MAX_RECORDS:
        raise ValueError("record count exceeds the protocol limit")

    packet_bytes = HEADER_BYTES + len(frame.records) * RECORD_BYTES + _CRC.size
    payload = bytearray(
        _HEADER.pack(
            MAGIC,
            VERSION,
            0,
            HEADER_BYTES,
            packet_bytes,
            frame.sequence,
            frame.capture_time_ns,
            frame.emit_time_ns,
            frame.frame_width,
            frame.frame_height,
            frame.source_id,
            0,
            len(frame.records),
            0,
            0,
        )
    )
    for record in frame.records:
        if not 0 <= record.id <= 0xFFFF:
            raise ValueError("gesture id must fit uint16")
        flags = FLAG_METRIC_VALID if record.metric_valid else 0
        payload.extend(
            _RECORD.pack(record.id, 1 if record.active else 0, flags, record.progress, record.metric)
        )
    payload.extend(_CRC.pack(zlib.crc32(payload) & 0xFFFFFFFF))
    return bytes(payload)


def unpack_frame(data: bytes) -> GestureFrame:
    """Parse and strictly validate one complete UDP datagram."""
    if len(data) < EMPTY_PACKET_BYTES:
        raise ValueError("packet is shorter than the gesture header and CRC")

    magic, version, _flags, header_bytes = struct.unpack_from("<4sBBH", data)
    if (magic, version, header_bytes) != (MAGIC, VERSION, HEADER_BYTES):
        raise ValueError("unsupported packet magic or version")

    (
        _magic,
        _version,
        flags,
        header_bytes,
        packet_bytes,
        sequence,
        capture_time_ns,
        emit_time_ns,
        frame_width,
        frame_height,
        source_id,
        _reserved,
        count,
        _reserved2,
        _reserved3,
    ) = _HEADER.unpack_from(data)

    if packet_bytes != len(data):
        raise ValueError("packet length does not match its header")
    if flags:
        raise ValueError("packet contains unknown flags")
    if not 0 <= count <= MAX_RECORDS:
        raise ValueError("invalid gesture record count")
    expected_bytes = HEADER_BYTES + count * RECORD_BYTES + _CRC.size
    if len(data) != expected_bytes:
        raise ValueError("packet length does not match its record count")

    expected_crc = _CRC.unpack_from(data, len(data) - _CRC.size)[0]
    actual_crc = zlib.crc32(data[: -_CRC.size]) & 0xFFFFFFFF
    if actual_crc != expected_crc:
        raise ValueError("CRC32 mismatch")

    records = []
    offset = header_bytes
    for _ in range(count):
        gesture_id, active, record_flags, progress, metric = _RECORD.unpack_from(data, offset)
        offset += RECORD_BYTES
        if record_flags & ~FLAG_METRIC_VALID:
            raise ValueError("unknown gesture record flags")
        if active not in (0, 1):
            raise ValueError("gesture active flag must be 0 or 1")
        records.append(
            GestureRecord(
                id=gesture_id,
                name=gesture_name(gesture_id),
                active=active,
                progress=progress,
                metric=metric,
                metric_valid=bool(record_flags & FLAG_METRIC_VALID),
            )
        )
    if offset != len(data) - _CRC.size:
        raise ValueError("unexpected bytes before CRC32")

    return GestureFrame(
        sequence=sequence,
        capture_time_ns=capture_time_ns,
        emit_time_ns=emit_time_ns,
        frame_width=frame_width,
        frame_height=frame_height,
        source_id=source_id,
        records=tuple(records),
    )
