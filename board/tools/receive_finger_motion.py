"""FNG1 receiver: normalized hand boxes and 21 (x, y, confidence) points per hand."""
import argparse
import json
import math
import socket
import struct
import time
import zlib

HEADER = struct.Struct("<4sBBHIIQQHHBBBB")
HAND = struct.Struct("<68f")


def decode(data):
    if len(data) < 44:
        raise ValueError("packet too short")
    fields = HEADER.unpack_from(data)
    magic, version, flags, header_bytes, packet_bytes, sequence, capture_ns, emit_ns, width, height, count, points, source_id, reserved = fields
    if (magic, version, flags, header_bytes, points, reserved) != (b"FNG1", 1, 4, 40, 21, 0):
        raise ValueError("unsupported FNG1 header")
    if count > 2 or packet_bytes != len(data) or len(data) != 44 + count * HAND.size:
        raise ValueError("hand count or packet length mismatch")
    if width == 0 or height == 0:
        raise ValueError("invalid frame dimensions")
    if struct.unpack_from("<I", data, len(data) - 4)[0] != zlib.crc32(data[:-4]) & 0xFFFFFFFF:
        raise ValueError("CRC32 mismatch")
    hands = []
    for i in range(count):
        values = HAND.unpack_from(data, HEADER.size + i * HAND.size)
        if any(not math.isfinite(v) or not 0 <= v <= 1 for v in values):
            raise ValueError("nonfinite or unnormalized hand values")
        hands.append({"score": values[0], "box": list(values[1:5]),
                      "keypoints": [list(values[5 + k * 3:8 + k * 3]) for k in range(21)]})
    return {"sequence": sequence, "capture_ns": capture_ns, "emit_ns": emit_ns,
            "width": width, "height": height, "source_id": source_id, "hands": hands}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=39542)
    parser.add_argument("--jsonl", help="optional path to save every decoded frame (including zero hands)")
    args = parser.parse_args()
    received = rejected = missed = 0
    last_sequence = None
    report_at = time.monotonic()
    output = open(args.jsonl, "w", encoding="utf-8") if args.jsonl else None
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
            udp.bind((args.bind, args.port))
            udp.settimeout(1.0)
            print("FNG1 listening on {}:{}".format(args.bind, args.port), flush=True)
            while True:
                try:
                    data, address = udp.recvfrom(2048)
                except socket.timeout:
                    continue
                try:
                    frame = decode(data)
                except ValueError as error:
                    rejected += 1
                    print("rejected {}: {}".format(address[0], error), flush=True)
                    continue
                if output:
                    output.write(json.dumps(frame, separators=(",", ":")) + "\n")
                sequence = frame["sequence"]
                if last_sequence is not None:
                    delta = (sequence - last_sequence) & 0xFFFFFFFF
                    if 0 < delta < 0x80000000:
                        missed += max(0, delta - 1)
                last_sequence = sequence
                received += 1
                now = time.monotonic()
                if now - report_at >= 1:
                    print("src={} seq={} fps={:.1f} hands={} points/hand=21 missed={} rejected={} board={:.1f}ms scores={}".format(
                        address[0], sequence, received / (now - report_at), len(frame["hands"]), missed, rejected,
                        (frame["emit_ns"] - frame["capture_ns"]) / 1e6,
                        [round(h["score"], 3) for h in frame["hands"]]), flush=True)
                    if output:
                        output.flush()
                    received = 0
                    report_at = now
    except KeyboardInterrupt:
        pass
    finally:
        if output:
            output.close()


if __name__ == "__main__":
    main()
