"""Standalone DMV1/DMV2 UDP diagnostic receiver for the upper computer."""

import argparse
import socket
import struct
import time
import zlib


HEADER_V1 = struct.Struct("<4sBBHIIQQHHBBBB")
HEADER_V2 = struct.Struct("<4sBBHIIQQHHBBBBI")


def decode(data):
    if len(data) < 44:
        raise ValueError("packet too short")
    magic, version, _flags, header_bytes = struct.unpack_from("<4sBBH", data)
    if (magic, version, header_bytes) == (b"DMV1", 1, HEADER_V1.size):
        header = HEADER_V1
    elif (magic, version, header_bytes) == (b"DMV2", 2, HEADER_V2.size):
        header = HEADER_V2
    else:
        raise ValueError("bad magic, version, or header size")
    fields = header.unpack_from(data)
    magic, version, flags, header_bytes, packet_bytes = fields[:5]
    sequence, capture_ns, emit_ns = fields[5:8]
    pose_count, face_count, source_id = fields[10:13]
    track_id = fields[14] if version == 2 else 1
    if packet_bytes != len(data):
        raise ValueError("packet length mismatch")
    if pose_count not in (0, 17) or face_count not in (0, 51):
        raise ValueError("bad keypoint count")
    if bool(flags & 1) != bool(pose_count) or bool(flags & 2) != bool(face_count):
        raise ValueError("flags/count mismatch")
    expected = header_bytes + (20 + pose_count * 12 if pose_count else 0) + (20 + face_count * 12 if face_count else 0) + 4
    if len(data) != expected:
        raise ValueError("detection block length mismatch")
    packet_crc = struct.unpack_from("<I", data, len(data) - 4)[0]
    if packet_crc != zlib.crc32(data[:-4]) & 0xFFFFFFFF:
        raise ValueError("CRC32 mismatch")
    return sequence, capture_ns, emit_ns, pose_count, face_count, source_id, track_id


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=39540)
    args = parser.parse_args()
    received = rejected = missed = 0
    last_sequence = None
    report_at = time.monotonic()

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
        udp.bind((args.bind, args.port))
        udp.settimeout(1.0)
        print("listening on {}:{}".format(args.bind, args.port), flush=True)
        try:
            while True:
                try:
                    data, address = udp.recvfrom(2048)
                except socket.timeout:
                    continue
                try:
                    sequence, capture_ns, emit_ns, pose_count, face_count, source_id, track_id = decode(data)
                except ValueError as error:
                    rejected += 1
                    print("rejected {}: {}".format(address[0], error), flush=True)
                    continue

                received += 1
                if last_sequence is not None:
                    delta = (sequence - last_sequence) & 0xFFFFFFFF
                    if 0 < delta < 0x80000000:
                        missed += max(0, delta - 1)
                last_sequence = sequence

                now = time.monotonic()
                elapsed = now - report_at
                if elapsed >= 1.0:
                    print(
                        "src={} source_id={} track_id={} seq={} packets/s={:.1f} missed={} rejected={} pose={} face={} board={:.1f}ms bytes={}".format(
                            address[0], source_id, track_id, sequence, received / elapsed, missed, rejected,
                            pose_count, face_count, (emit_ns - capture_ns) / 1_000_000.0, len(data)
                        ),
                        flush=True,
                    )
                    received = 0
                    report_at = now
        except KeyboardInterrupt:
            print("stopped")


if __name__ == "__main__":
    main()
