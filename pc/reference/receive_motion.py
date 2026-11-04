"""Receive and validate digital-man motion packet v1 over UDP."""

import argparse
import socket
import time

from motion_packet import unpack_frame


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0", help="local IPv4 address")
    parser.add_argument("--port", type=int, default=39540)
    args = parser.parse_args()

    received = 0
    rejected = 0
    missed = 0
    last_sequence = None
    report_started = time.monotonic()

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
                    frame = unpack_frame(data)
                except ValueError as error:
                    rejected += 1
                    print("rejected {}: {}".format(address[0], error), flush=True)
                    continue

                received += 1
                if last_sequence is not None:
                    delta = (frame.sequence - last_sequence) & 0xFFFFFFFF
                    if 0 < delta < 0x80000000:
                        missed += max(0, delta - 1)
                last_sequence = frame.sequence

                now = time.monotonic()
                elapsed = now - report_started
                if elapsed >= 1.0:
                    board_ms = (frame.emit_time_ns - frame.capture_time_ns) / 1_000_000.0
                    print(
                        "src={} seq={} fps={:.1f} missed={} rejected={} body={} face={} board={:.1f}ms bytes={}".format(
                            address[0],
                            frame.sequence,
                            received / elapsed,
                            missed,
                            rejected,
                            "yes" if frame.body else "no",
                            "yes" if frame.face else "no",
                            board_ms,
                            len(data),
                        ),
                        flush=True,
                    )
                    received = 0
                    report_started = now
        except KeyboardInterrupt:
            print("stopped")


if __name__ == "__main__":
    main()

