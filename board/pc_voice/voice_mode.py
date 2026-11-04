"""Voice commands use the existing local authoritative mode channel, not a second mode."""
import json
import re
import select
import socket
import threading
import time
import uuid


def command_mode(text):
    # Exact utterances (punctuation/whitespace ignored) avoid quotations, negations
    # and normal chat sentences accidentally changing the operating mode.
    words = re.sub(r"[\W_]+", "", text)
    return {"跟着我的动作": "follow", "跟我聊天": "interact"}.get(words)


class VoiceModeLink:
    def __init__(self, port, changed):
        self.destination = ("127.0.0.1", port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", 0))
        self.socket.setblocking(False)
        self.session = uuid.uuid4().hex
        self.sequence = 0
        self.pending = None
        self.queries = set()
        self.last_reply = 0.0
        self.last_query = 0
        self.mode = None
        self.changed = changed
        self.lock = threading.RLock()
        self.closed = threading.Event()
        self.worker = threading.Thread(target=self.run, name="voice-mode", daemon=True)
        self.worker.start()

    def request(self, mode):
        if mode not in ("follow", "interact"):
            raise ValueError("invalid mode")
        with self.lock:
            self.sequence += 1
            self.pending = dict(v=1, kind="set", session=self.session, seq=self.sequence, mode=mode)
            # Fail closed while a command is awaiting confirmation. ASR keeps listening.
            self.update(None)

    def update(self, mode):
        with self.lock:
            if self.mode != mode:
                self.mode = mode
                self.changed(mode)

    def run(self):
        next_send = 0.0
        while not self.closed.is_set():
            now = time.monotonic()
            if now >= next_send:
                with self.lock:
                    if self.pending:
                        request = dict(self.pending)
                    else:
                        self.sequence += 1
                        request = dict(v=1, kind="get", session=self.session, seq=self.sequence)
                        self.queries.add(self.sequence)
                        self.queries = set(sorted(self.queries)[-16:])
                try:
                    self.socket.sendto(json.dumps(request).encode("ascii"), self.destination)
                except OSError:
                    pass
                next_send = now + .2
            try:
                readable, _, _ = select.select([self.socket], [], [], .05)
                if readable:
                    packet, peer = self.socket.recvfrom(4096)
                    response = json.loads(packet)
                    if (peer != self.destination or not isinstance(response, dict)
                            or type(response.get("v")) is not int or response["v"] != 1
                            or response.get("session") != self.session
                            or type(response.get("seq")) is not int
                            or response.get("mode") not in ("follow", "interact")):
                        continue
                    with self.lock:
                        if self.pending:
                            if response.get("kind") != "ack" or response["seq"] != self.pending["seq"]:
                                continue
                            self.pending = None
                            self.queries.clear()
                            self.last_query = response["seq"]
                        else:
                            if (response.get("kind") != "state" or response["seq"] not in self.queries
                                    or response["seq"] <= self.last_query):
                                continue
                            self.last_query = response["seq"]
                        self.last_reply = now
                        self.update(response["mode"])
            except (OSError, ValueError, UnicodeError):
                pass
            if now - self.last_reply > 1.5:
                self.update(None)

    def close(self):
        self.closed.set()
        self.worker.join(timeout=2)
        self.socket.close()
