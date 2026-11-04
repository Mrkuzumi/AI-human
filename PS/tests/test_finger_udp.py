"""Run with the host-built smoke command (e.g. docker exec CONTAINER /path/to/smoke)."""
import struct
import subprocess
import sys
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from receive_finger_motion import decode
from receive_motion import decode as decode_motion

result = subprocess.run(sys.argv[1:], check=True, capture_output=True, text=True)
packets = [line.split() for line in result.stdout.splitlines() if line.startswith(("FNG1 ", "DMV1 "))]
assert len(packets) == 6
for count in range(3):
        assert packets[count * 2][0] == "FNG1" and packets[count * 2 + 1][0] == "DMV1"
        data = bytes.fromhex(packets[count * 2][1])
        frame = decode(data)
        body = decode_motion(bytes.fromhex(packets[count * 2 + 1][1]))
        assert frame["sequence"] == body[0] == count
        assert frame["capture_ns"] == body[1] == 123456789
        assert frame["emit_ns"] == body[2] == 123456999
        assert (frame["width"], frame["height"]) == (1920, 1080)
        assert len(frame["hands"]) == count
        if count:
            assert len(frame["hands"][0]["keypoints"]) == 21
            assert frame["hands"][0]["keypoints"][20][0] == 1.0
        corrupt = bytearray(data)
        corrupt[8] ^= 1
        try:
            decode(corrupt)
            raise AssertionError("corruption accepted")
        except ValueError:
            pass
for header_index in (4, 5, 6, 36, 37, 39):
        malformed = bytearray(data)
        malformed[header_index] = 255
        struct.pack_into("<I", malformed, len(malformed) - 4, zlib.crc32(malformed[:-4]) & 0xFFFFFFFF)
        try:
            decode(malformed)
            raise AssertionError("malformed header accepted")
        except ValueError:
            pass
for truncated in (b"", data[:39], data[:-1]):
        try:
            decode(truncated)
            raise AssertionError("truncation accepted")
        except ValueError:
            pass
print("PASS: C++/Python FNG1 0/1/2 hands, shared DMV1 sequence/timestamps, invalid counts, CRC and header validation")
