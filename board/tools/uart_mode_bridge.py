#!/usr/bin/env python3
"""Forward TJC @MODE=... CRLF events to the PC; Python 3 standard library only."""

import argparse
import fcntl
import json
import os
import select
import signal
import socket
import termios
import time
import uuid


class ModeLineParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        modes = []
        for byte in data:
            if byte == ord("@"):
                self.buffer.clear()
                self.buffer.append(byte)
            elif self.buffer:
                self.buffer.append(byte)
                if byte == 10:
                    if self.buffer == b"@MODE=follow\r\n":
                        modes.append("follow")
                    elif self.buffer == b"@MODE=interact\r\n":
                        modes.append("interact")
                    self.buffer.clear()
                elif len(self.buffer) >= 64:
                    self.buffer.clear()
        return modes


class ModeLink:
    RETRY_SECONDS = 0.5
    ACK_TIMEOUT = 2.0

    def __init__(self, host, port):
        self.destination = socket.gethostbyname(host), port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("0.0.0.0", 0))
        self.socket.setblocking(False)
        self.session = uuid.uuid4().hex
        self.sequence = 0
        self.mode = None
        self.actual_mode = None
        self.last_reply = None
        self.query_sequence = 0
        self.next_query = 0.0
        self.packet = b""
        self.next_send = 0.0
        self.requested_at = 0.0
        self.last_ack = None
        self.status = "idle"

    def request(self, mode, now=None):
        if mode not in ("follow", "interact"):
            raise ValueError("invalid mode")
        now = time.monotonic() if now is None else now
        # Each serial event is a new action; tick() alone retransmits it.
        self.mode = mode
        self.actual_mode = None
        self.sequence += 1
        if self.sequence > 0x7FFFFFFF:
            self.session, self.sequence = uuid.uuid4().hex, 1
        self.packet = json.dumps(dict(v=1, kind="set", session=self.session,
                                      seq=self.sequence, mode=mode), separators=(",", ":")).encode("ascii")
        self.last_ack = None
        self.requested_at = now
        self.status = "pending"
        print("[MODE] requested {} (seq={})".format(mode, self.sequence), flush=True)
        self.next_send = 0.0

    def tick(self, now=None):
        now = time.monotonic() if now is None else now
        if self.mode is None:
            if now >= self.next_query:
                self.query_sequence = self.query_sequence % 0x7FFFFFFF + 1
                query = dict(v=1, kind="get", session=self.session, seq=self.query_sequence)
                try:
                    self.socket.sendto(json.dumps(query, separators=(",", ":")).encode("ascii"), self.destination)
                except OSError:
                    pass
                self.next_query = now + self.RETRY_SECONDS
            if self.last_reply is not None and now - self.last_reply > self.ACK_TIMEOUT:
                self.actual_mode = None
            return
        if now >= self.next_send:
            try:
                self.socket.sendto(self.packet, self.destination)
            except OSError as exc:
                if self.status != "offline":
                    print("[MODE] UDP unavailable: {}".format(exc), flush=True)
            self.next_send = now + self.RETRY_SECONDS
        since = self.last_ack if self.last_ack is not None else self.requested_at
        if now - since > self.ACK_TIMEOUT and self.status != "offline":
            self.status = "offline"
            print("[MODE] PC not confirmed; retrying {}".format(self.mode), flush=True)
        if self.last_reply is not None and now - self.last_reply > self.ACK_TIMEOUT:
            self.actual_mode = None

    def poll(self, now=None):
        now = time.monotonic() if now is None else now
        for _ in range(32):
            try:
                packet, peer = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            except (ConnectionResetError, ConnectionRefusedError):
                continue
            if peer != self.destination or len(packet) > 256:
                continue
            try:
                ack = json.loads(packet)
            except (ValueError, UnicodeError):
                continue
            if (isinstance(ack, dict) and ack.get("kind") == "state" and self.mode is None
                    and type(ack.get("v")) is int and ack["v"] == 1
                    and ack.get("session") == self.session
                    and type(ack.get("seq")) is int and ack["seq"] == self.query_sequence
                    and ack.get("mode") in ("follow", "interact")):
                self.actual_mode = ack["mode"]
                self.last_reply = now
                continue
            if (
                not isinstance(ack, dict) or type(ack.get("v")) is not int or ack["v"] != 1
                or ack.get("kind") != "ack" or ack.get("session") != self.session
                or type(ack.get("seq")) is not int or ack["seq"] != self.sequence
                or self.mode is None or ack.get("mode") not in ("follow", "interact")
            ):
                continue
            if ack["mode"] != self.mode:
                self.mode = ack["mode"]
                # Keep the heartbeat in sync, so a restarted PC restores the
                # last confirmed mode. Same-sequence packets never reapply it.
                self.packet = json.dumps(dict(v=1, kind="set", session=self.session,
                                              seq=self.sequence, mode=self.mode), separators=(",", ":")).encode("ascii")
                print("[MODE] PC active {} (newer voice/button action)".format(self.mode), flush=True)
            if self.status != "confirmed":
                print("[MODE] PC bridge confirmed {} (seq={})".format(self.mode, self.sequence), flush=True)
            self.status = "confirmed"
            self.last_ack = now
            self.actual_mode = ack["mode"]
            self.last_reply = now

    def close(self):
        self.socket.close()


def open_serial(path, baud):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        attrs = termios.tcgetattr(fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
        attrs[4] = attrs[5] = getattr(termios, "B" + str(baud))
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        return fd
    except BaseException:
        os.close(fd)
        raise


def write_page(fd, mode):
    command = b"page page3\xff\xff\xff" if mode == "interact" else b"page page2\xff\xff\xff"
    while command:
        if not select.select((), (fd,), (), 0.5)[1]:
            raise OSError("screen page write timed out")
        sent = os.write(fd, command)
        if sent <= 0:
            raise OSError("screen page write failed")
        command = command[sent:]


def run(args):
    link = ModeLink(args.host, args.port)
    parser = ModeLineParser()
    serial_fd = None
    reopen_at = 0.0
    serial_error = None
    displayed_mode = None
    print("[UART] {} baud={} 8N1 -> {}:{}; waiting for a button".format(
        args.serial, args.baud, *link.destination), flush=True)
    try:
        while True:
            now = time.monotonic()
            if serial_fd is None and now >= reopen_at:
                try:
                    serial_fd = open_serial(args.serial, args.baud)
                    parser = ModeLineParser()
                    displayed_mode = None
                    serial_error = None
                    print("[UART] connected {}".format(args.serial), flush=True)
                except (OSError, termios.error) as exc:
                    message = str(exc)
                    if message != serial_error:
                        print("[UART] {}: {}; retry in 2 seconds".format(args.serial, message), flush=True)
                    serial_error = message
                    reopen_at = now + 2.0
            readers = [link.socket]
            if serial_fd is not None:
                readers.append(serial_fd)
            readable, _, _ = select.select(readers, (), (), 0.1)
            if serial_fd is not None and serial_fd in readable:
                try:
                    data = os.read(serial_fd, 4096)
                    if not data:
                        raise OSError("serial device disconnected")
                    for mode in parser.feed(data):
                        link.request(mode)
                except BlockingIOError:
                    pass
                except (OSError, termios.error) as exc:
                    print("[UART] {}: {}; reconnecting".format(args.serial, exc), flush=True)
                    os.close(serial_fd)
                    serial_fd = None
                    reopen_at = time.monotonic() + 2.0
            link.poll()
            link.tick()
            if serial_fd is not None and link.actual_mode is not None and link.actual_mode != displayed_mode:
                try:
                    write_page(serial_fd, link.actual_mode)
                    displayed_mode = link.actual_mode
                    print("[UART] screen page -> {}".format(displayed_mode), flush=True)
                except (OSError, termios.error) as exc:
                    print("[UART] screen page write: {}; reconnecting".format(exc), flush=True)
                    os.close(serial_fd)
                    serial_fd = None
                    reopen_at = time.monotonic() + 2.0
    finally:
        if serial_fd is not None:
            os.close(serial_fd)
        link.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", required=True, help="screen serial device; prefer /dev/serial/by-id/...")
    parser.add_argument("--host", required=True, help="PC running motion_bridge.py")
    parser.add_argument("--port", type=int, default=39544)
    parser.add_argument("--baud", type=int, choices=(9600, 19200, 38400, 57600, 115200), default=115200)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be in [1, 65535]")
    signal.signal(signal.SIGTERM, stop)
    try:
        run(args)
    except KeyboardInterrupt:
        pass


def stop(_signum, _frame):
    raise SystemExit(0)


if __name__ == "__main__":
    main()
