"""Small acknowledged UDP control channel, separate from camera data (port 39544)."""

import json
import re
import socket
import uuid


MODES = ("follow", "interact")


class ModeControlReceiver:
    def __init__(self, bind, port, allowed_host="", initial_mode="follow"):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.bind((bind, port))
            self.socket.setblocking(False)
        except BaseException:
            self.socket.close()
            raise
        self.allowed_host = allowed_host
        self.sequences = {}
        self.mode = initial_mode
        self.server_id = uuid.uuid4().hex
        self.revision = 0

    def poll(self, apply_mode):
        # Bound work so a busy control socket cannot starve pose processing.
        for _ in range(32):
            try:
                packet, peer = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            except ConnectionResetError:
                # Windows can report an ICMP error after the board disconnects.
                continue
            local = peer[0] == "127.0.0.1"
            if len(packet) > 256 or (self.allowed_host and peer[0] != self.allowed_host and not local):
                continue
            try:
                request = json.loads(packet)
            except (ValueError, UnicodeError):
                continue
            if not isinstance(request, dict):
                continue
            session, sequence, mode = (request.get(k) for k in ("session", "seq", "mode"))
            if (
                type(request.get("v")) is not int or request["v"] != 1
                or request.get("kind") not in ("set", "get")
                or not isinstance(session, str) or not re.fullmatch(r"[0-9a-f]{32}", session)
                or type(sequence) is not int or not 1 <= sequence <= 0x7FFFFFFF
            ):
                continue
            if request["kind"] == "get":
                state = dict(v=1, kind="state", session=session, seq=sequence, mode=self.mode,
                             server=self.server_id, revision=self.revision)
                try:
                    self.socket.sendto(json.dumps(state, separators=(",", ":")).encode("ascii"), peer)
                except OSError:
                    pass
                continue
            if mode not in MODES:
                continue
            key = peer, session
            previous_sequence, _ = self.sequences.get(key, (0, ""))
            if sequence < previous_sequence:
                continue
            # A retransmission acknowledges current state; it must not undo a
            # newer voice/button action from a different sender.
            if sequence > previous_sequence:
                applied = apply_mode(mode)
                if applied != self.mode:
                    self.revision += 1
                self.mode = applied
            else:
                applied = self.mode
            self.sequences[key] = sequence, mode
            while len(self.sequences) > 16:
                self.sequences.pop(next(iter(self.sequences)))
            ack = dict(v=1, kind="ack", session=session, seq=sequence, mode=applied)
            try:
                self.socket.sendto(json.dumps(ack, separators=(",", ":")).encode("ascii"), peer)
            except OSError:
                # The board retries the same idempotent request after a lost ACK.
                pass

    def close(self):
        self.socket.close()
