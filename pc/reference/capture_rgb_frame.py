"""Capture one complete DMI1 JPEG for board-to-PC image-path diagnostics."""

from __future__ import annotations

import argparse
import socket
from pathlib import Path

from rgb_pose_receiver import JpegFrameReassembler


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=39541)
    args = parser.parse_args()

    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind((args.bind, args.port))
    receiver.settimeout(5.0)
    reassembler = JpegFrameReassembler()
    while True:
        complete = reassembler.push(receiver.recvfrom(2048)[0])
        if complete is None:
            continue
        sequence, _capture_ns, jpeg = complete
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_bytes(jpeg)
        print("saved sequence={} bytes={} path={}".format(sequence, len(jpeg), args.output.resolve()))
        return


if __name__ == "__main__":
    main()
