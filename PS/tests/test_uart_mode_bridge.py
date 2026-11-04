"""Linux parser/serial checks and PTY -> board helper -> real PC bridge -> OSC."""

import json
import os
import pty
import re
import select
import shutil
import socket
import subprocess
import sys
import tempfile
import termios
import time
import unittest
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "tools"))
from uart_mode_bridge import ModeLineParser, ModeLink, open_serial, write_page

PC = PACKAGE.parent / "AI_humen" / "AI_humen" / "pc"
sys.path.insert(0, str(PC / "tests"))
from test_mode_control import decode_osc, free_port


class SerialParserTests(unittest.TestCase):
    def test_split_and_back_to_back_events(self):
        parser = ModeLineParser()
        self.assertEqual(parser.feed(b"@MO"), [])
        self.assertEqual(parser.feed(b"DE=follow\r"), [])
        self.assertEqual(parser.feed(b"\n@MODE=interact\r\n"), ["follow", "interact"])

    def test_native_return_frames_noise_and_overflow_resynchronize(self):
        parser = ModeLineParser()
        noise = b"\x88\xff\xff\xff\x65\x01\x03\x00\xff\xff\xff"
        self.assertEqual(parser.feed(noise + b"@MODE=toggle\r\n@" + b"x" * 10000), [])
        self.assertLess(len(parser.buffer), 64)
        self.assertEqual(parser.feed(b"@junk@MODE=interact\r\n"), ["interact"])

    def test_requires_exact_case_and_crlf(self):
        parser = ModeLineParser()
        for packet in (b"MODE=follow\r\n", b"@MODE=follow\n", b"@mode=follow\r\n",
                       b"@MODE=follow \r\n", b"@MODE=interact\x00\r\n"):
            self.assertEqual(parser.feed(packet), [])

    def test_serial_is_raw_115200_and_has_no_echo(self):
        master, slave = pty.openpty()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        fd = open_serial(os.ttyname(slave), 115200)
        self.addCleanup(os.close, fd)
        attrs = termios.tcgetattr(fd)
        self.assertEqual(attrs[4:6], [termios.B115200, termios.B115200])
        self.assertEqual(attrs[3] & (termios.ICANON | termios.ECHO), 0)
        os.write(master, b"@MODE=follow\r\n")
        self.assertTrue(select.select([fd], [], [], 1)[0])
        self.assertEqual(os.read(fd, 128), b"@MODE=follow\r\n")

    def test_screen_page_commands_have_native_terminators(self):
        master, slave = pty.openpty()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        fd = open_serial(os.ttyname(slave), 115200)
        self.addCleanup(os.close, fd)
        for mode, expected in (("interact", b"page page3\xff\xff\xff"),
                               ("follow", b"page page2\xff\xff\xff")):
            write_page(fd, mode)
            self.assertTrue(select.select([master], [], [], 1)[0])
            self.assertEqual(os.read(master, 128), expected)


class ModeLinkTests(unittest.TestCase):
    def setUp(self):
        self.pc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pc.bind(("127.0.0.1", 0))
        self.pc.settimeout(0.1)
        self.addCleanup(self.pc.close)
        self.link = ModeLink(*self.pc.getsockname())
        self.addCleanup(self.link.close)

    def ack(self, req, destination):
        req["kind"] = "ack"
        self.pc.sendto(json.dumps(req).encode("ascii"), destination)

    def test_no_startup_override_retry_and_matching_ack(self):
        self.link.tick(now=1)
        query, peer = self.pc.recvfrom(256)
        request = json.loads(query)
        self.assertEqual(request["kind"], "get")
        request.update(kind="state", mode="follow")
        self.pc.sendto(json.dumps(request).encode("ascii"), peer)
        self.link.poll(now=1.1)
        self.assertIsNone(self.link.mode)
        self.assertEqual(self.link.actual_mode, "follow")
        self.link.request("interact", now=2)
        self.link.tick(now=2)
        packet, peer = self.pc.recvfrom(256)
        req = json.loads(packet)
        self.ack(dict(req, seq=req["seq"] + 1), peer)
        self.link.poll(now=2.1)
        self.assertEqual(self.link.status, "pending")
        self.link.tick(now=4.1)
        retry, _ = self.pc.recvfrom(256)
        self.assertEqual(retry, packet)
        self.assertEqual(self.link.status, "offline")
        self.ack(req, peer)
        self.link.poll(now=4.2)
        self.assertEqual(self.link.status, "confirmed")
        self.link.tick(now=6.3)
        self.assertEqual(self.link.status, "offline")

    def test_repeated_clicks_are_new_actions_and_old_ack_cannot_confirm_new_mode(self):
        self.link.request("follow", now=1)
        self.link.tick(now=1)
        packet, peer = self.pc.recvfrom(256)
        self.link.request("follow", now=1.1)
        self.assertEqual(self.link.sequence, 2)
        self.link.request("interact", now=1.2)
        self.assertEqual(self.link.sequence, 3)
        self.ack(json.loads(packet), peer)
        self.link.poll(now=1.3)
        self.assertEqual(self.link.status, "pending")


class SerialToPcTests(unittest.TestCase):
    def setUp(self):
        self.processes = []
        self.fds = []
        self.addCleanup(self.cleanup)
        self.temp = tempfile.TemporaryDirectory(prefix="uart-mode-")
        self.addCleanup(self.temp.cleanup)
        self.serial_path = Path(self.temp.name) / "screen"
        self.sink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sink.bind(("127.0.0.1", 0))
        self.sink.settimeout(0.1)
        self.addCleanup(self.sink.close)
        self.control_port, self.motion_port, self.gesture_port = free_port(), free_port(), free_port()
        self.pc = self.start_pc()
        self.master = self.new_serial()
        self.board = subprocess.Popen([
            sys.executable, "-B", "-u", str(PACKAGE / "tools" / "uart_mode_bridge.py"),
            "--serial", str(self.serial_path), "--host", "127.0.0.1", "--port", str(self.control_port),
        ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.processes.append(self.board)
        self.wait_line(self.board, "[UART] connected")

    def cleanup(self):
        for process in reversed(self.processes):
            if process.poll() is None:
                process.terminate()
            try:
                process.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate()
        for fd in self.fds:
            os.close(fd)

    def new_serial(self):
        master, slave = pty.openpty()
        self.fds.extend((master, slave))
        target = Path(self.temp.name) / "replacement"
        target.symlink_to(os.ttyname(slave))
        os.replace(str(target), str(self.serial_path))
        return master

    def start_pc(self):
        process = subprocess.Popen([
            sys.executable, "-B", "-u", str(PC / "reference" / "motion_bridge.py"),
            "--bind", "127.0.0.1", "--port", str(self.motion_port), "--no-fingers",
            "--gesture-port", str(self.gesture_port), "--mode-control-port", str(self.control_port),
            "--vmc-host", "127.0.0.1", "--vmc-port", str(self.sink.getsockname()[1]), "--quiet",
        ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.processes.append(process)
        self.wait_line(process, "[MODE] control UDP")
        return process

    def wait_line(self, process, needle, timeout=5):
        deadline = time.monotonic() + timeout
        # Read bytes without buffered readline prefetch, so select sees each log line.
        line = bytearray()
        while time.monotonic() < deadline:
            if not select.select([process.stdout], [], [], 0.1)[0]:
                continue
            chunk = os.read(process.stdout.fileno(), 1)
            if not chunk:
                self.fail("process ended while waiting for " + needle)
            line.extend(chunk)
            if chunk == b"\n":
                if needle in line.decode("utf-8", errors="replace"):
                    return
                line.clear()
        self.fail("missing log: " + needle)

    def wait_mode(self, mode, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                address, values = decode_osc(self.sink.recv(2048))
            except socket.timeout:
                continue
            if address == "/Avatar/Mode" and values == [mode]:
                return
        self.fail("missing OSC mode " + mode)

    def wait_page(self, mode, timeout=5):
        expected = b"page page3\xff\xff\xff" if mode == "interact" else b"page page2\xff\xff\xff"
        deadline = time.monotonic() + timeout
        received = bytearray()
        while time.monotonic() < deadline:
            if select.select([self.master], [], [], 0.1)[0]:
                received.extend(os.read(self.master, 128))
                if expected in received:
                    return
        self.fail("missing screen command " + repr(expected))

    def test_voice_mode_drives_page_and_reconnect_restores_it(self):
        self.wait_page("follow")
        for sequence, mode in ((1, "interact"), (2, "follow")):
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as voice:
                voice.settimeout(2)
                voice.sendto(json.dumps(dict(v=1, kind="set", session="b" * 32,
                                            seq=sequence, mode=mode)).encode(),
                             ("127.0.0.1", self.control_port))
                self.assertEqual(json.loads(voice.recv(256))["mode"], mode)
            self.wait_page(mode)
        old_master = self.master
        os.close(old_master)
        self.fds.remove(old_master)
        self.master = self.new_serial()
        self.wait_page("follow")

    def test_serial_split_duplicate_pc_restart_and_usb_reconnect(self):
        os.write(self.master, b"\x88\xff\xff\xff@MODE=inter")
        os.write(self.master, b"act\r\n")
        self.wait_line(self.board, "PC bridge confirmed interact")
        self.wait_mode("interact")
        os.write(self.master, b"@MODE=interact\r\n@MODE=follow\r\n")
        self.wait_line(self.board, "PC bridge confirmed follow")
        self.wait_mode("follow")
        os.write(self.master, b"@MODE=interact\r\n")
        self.wait_line(self.board, "PC bridge confirmed interact")
        self.wait_mode("interact")

        # A voice sender overrides the last serial event; its old heartbeat
        # learns the current mode rather than undoing the voice command.
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as voice:
            voice.settimeout(2)
            voice.sendto(json.dumps(dict(v=1, kind="set", session="b" * 32,
                                        seq=1, mode="follow")).encode(), ("127.0.0.1", self.control_port))
            self.assertEqual(json.loads(voice.recv(256))["mode"], "follow")
        self.wait_mode("follow")
        self.wait_line(self.board, "PC active follow")

        self.pc.terminate()
        self.pc.communicate(timeout=5)
        self.wait_line(self.board, "PC not confirmed")
        self.pc = self.start_pc()
        self.wait_line(self.board, "PC bridge confirmed follow")
        self.wait_mode("follow")
        # Pressing the same interaction button as before is still a new action.
        os.write(self.master, b"@MODE=interact\r\n")
        self.wait_line(self.board, "PC bridge confirmed interact")
        self.wait_mode("interact")

        old_master = self.master
        os.close(old_master)
        self.fds.remove(old_master)
        self.master = self.new_serial()
        self.wait_line(self.board, "reconnecting")
        self.wait_line(self.board, "[UART] connected")
        os.write(self.master, b"@MODE=follow\r\n")
        self.wait_line(self.board, "PC bridge confirmed follow")
        self.wait_mode("follow")


class LauncherTests(unittest.TestCase):
    def test_nonroot_requests_sudo_and_preserves_defaults_and_overrides(self):
        with tempfile.TemporaryDirectory(prefix="uart-sudo-") as directory:
            root = Path(directory)
            root.chmod(0o755)
            (root / "scripts").mkdir()
            (root / "bin").mkdir()
            script = root / "scripts" / "run_board.sh"
            shutil.copyfile(str(PACKAGE / "scripts" / "run_board.sh"), str(script))
            sudo = root / "bin" / "sudo"
            sudo.write_text("#!/bin/sh\nprintf '%s\\n' \"$@\"\nexit 42\n")
            sudo.chmod(0o755)
            env = dict(os.environ, PATH=str(root / "bin") + ":" + os.environ["PATH"])
            env.pop("UART_DEVICE", None)
            env.pop("MODE_HOST", None)
            drop_root = (lambda: os.setuid(65534)) if os.geteuid() == 0 else None
            for args, expected in (([], [
                "configs/ZG/yolov8-pose+yolov8-face+yolov8-finger+hdmi-192.168.126.100.yaml",
                "/dev/ttyACM0", "192.168.126.100",
            ]), (["custom.yaml", "/dev/ttyACM7", "192.168.126.101"], [
                "custom.yaml", "/dev/ttyACM7", "192.168.126.101",
            ])):
                result = subprocess.run(["bash", str(script)] + args, env=env,
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        preexec_fn=drop_root, timeout=5)
                output = result.stdout.decode(errors="replace")
                self.assertEqual(result.returncode, 42, output)
                self.assertEqual(output.splitlines()[1:], [
                    "--preserve-env=VOICE_ENABLED,VOICE_HOST,VOICE_PORT,AUDIO_DEVICE,VOICE_UPLOAD_TOKEN",
                    "--", "bash", str(script)] + expected)

    def test_runtime_exit_preserves_status_and_cleans_up_uart_helper(self):
        with tempfile.TemporaryDirectory(prefix="uart-launcher-") as directory:
            root = Path(directory)
            (root / "scripts").mkdir()
            (root / "tools").mkdir()
            (root / "bin").mkdir()
            shutil.copyfile(str(PACKAGE / "scripts" / "run_board.sh"), str(root / "scripts" / "run_board.sh"))
            shutil.copyfile(str(PACKAGE / "tools" / "uart_mode_bridge.py"), str(root / "tools" / "uart_mode_bridge.py"))
            for model in ("pose", "face", "finger"):
                for suffix in ("json", "raw"):
                    path = root / "imodel" / "ZG" / ("yolov8-{}_ZG.{}".format(model, suffix))
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.touch()
                names = root / "names" / ("yolov8-{}.names".format(model))
                names.parent.mkdir(exist_ok=True)
                names.touch()
            (root / "config.yaml").touch()
            for name, content in (("uname", "#!/bin/sh\nprintf 'aarch64\\n'\n"),
                                  ("ldd", "#!/bin/sh\nexit 0\n")):
                path = root / "bin" / name
                path.write_text(content)
                path.chmod(0o755)
            executable = root / "sdicamera+yolov8-pose+yolov8-face+yolov8-finger+hdmi"
            executable.write_text("#!/bin/sh\nsleep 1\nexit 7\n")
            master, slave = pty.openpty()
            try:
                env = dict(os.environ, PATH=str(root / "bin") + ":" + os.environ["PATH"])
                result = subprocess.run([
                    "bash", str(root / "scripts" / "run_board.sh"), "config.yaml",
                    os.ttyname(slave), "127.0.0.1",
                ], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=10)
                output = result.stdout.decode(errors="replace")
                self.assertEqual(result.returncode, 7, output)
                match = re.search(r"UART control PID: (\d+)", output)
                self.assertIsNotNone(match, output)
                with self.assertRaises(ProcessLookupError):
                    os.kill(int(match.group(1)), 0)
            finally:
                os.close(master)
                os.close(slave)


if __name__ == "__main__":
    unittest.main()
