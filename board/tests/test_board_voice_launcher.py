"""Linux launcher integration: real helper, TCP server and fake ALSA subprocess."""

import json
import os
from pathlib import Path
import re
import shutil
import socketserver
import subprocess
import sys
import tempfile
import threading
import unittest

PACKAGE = Path(__file__).resolve().parents[1]


class UploadHandler(socketserver.StreamRequestHandler):
    def handle(self):
        self.request.settimeout(5)
        if self.rfile.readline() != b"test-token\n":
            return
        self.wfile.write("[已连接] test\n".encode("utf-8"))
        self.wfile.flush()
        audio = self.rfile.read(6400)
        if len(audio) == 6400:
            self.wfile.write("[最终] launcher speech test\n".encode("utf-8"))
            self.wfile.flush()
        while self.rfile.read(6400):
            pass


@unittest.skipIf(os.name == "nt", "requires Linux bash and process signals")
class VoiceLauncherTests(unittest.TestCase):
    def test_runtime_status_voice_output_and_no_orphan_recorder(self):
        server = socketserver.ThreadingTCPServer(("127.0.0.1", 0), UploadHandler)
        server.daemon_threads = True
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        with tempfile.TemporaryDirectory(prefix="voice-launcher-") as directory:
            root = Path(directory)
            for name in ("scripts", "tools", "configs", "bin"):
                (root / name).mkdir()
            for relative in ("scripts/run_board.sh", "tools/board_voice_stream.py"):
                shutil.copyfile(str(PACKAGE / relative), str(root / relative))
            for model in ("pose", "face", "finger"):
                for suffix in ("json", "raw"):
                    path = root / "imodel" / "ZG" / ("yolov8-{}_ZG.{}".format(model, suffix))
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.touch()
                path = root / "names" / ("yolov8-{}.names".format(model))
                path.parent.mkdir(exist_ok=True)
                path.touch()
            (root / "config.yaml").touch()
            (root / "configs/voice.json").write_text(json.dumps(dict(
                enabled=True, server_port=server.server_address[1],
                upload_token="test-token", audio_device="plughw:0,0", reconnect_seconds=0.1)))
            for name, script in (
                ("uname", "#!/bin/sh\nprintf 'aarch64\\n'\n"),
                ("ldd", "#!/bin/sh\nexit 0\n"),
                ("arecord", "#!{}\nimport os,sys,time\nfrom pathlib import Path\n"
                 "Path(os.environ['VOICE_TEST_PID_FILE']).write_text(str(os.getpid()))\n"
                 "sys.stdout.buffer.write(b'\\0'*12800)\nsys.stdout.buffer.flush()\ntime.sleep(30)\n".format(sys.executable)),
            ):
                path = root / "bin" / name
                path.write_text(script)
                path.chmod(0o755)
            runtime = root / "sdicamera+yolov8-pose+yolov8-face+yolov8-finger+hdmi"
            runtime.write_text("#!/bin/sh\nprintf 'TEST_VIDEO_RUNNING\\n'\nsleep 2\nexit 7\n")
            pid_file = root / "recorder.pid"
            env = dict(os.environ, PATH=str(root / "bin") + ":" + os.environ["PATH"],
                       VOICE_ENABLED="1", VOICE_TEST_PID_FILE=str(pid_file))
            for name in ("VOICE_PORT", "AUDIO_DEVICE", "VOICE_UPLOAD_TOKEN", "VOICE_HOST"):
                env.pop(name, None)
            result = subprocess.run([
                "bash", str(root / "scripts/run_board.sh"), "config.yaml", "", "127.0.0.1",
            ], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=10)
            output = result.stdout.decode("utf-8", errors="replace")
            self.assertEqual(result.returncode, 7, output)
            self.assertIn("TEST_VIDEO_RUNNING", output)
            self.assertIn("[最终] launcher speech test", output)
            match = re.search(r"Voice upload PID: (\d+)", output)
            self.assertIsNotNone(match, output)
            self.assertTrue(pid_file.exists(), output)
            for pid in (int(match.group(1)), int(pid_file.read_text())):
                with self.assertRaises(ProcessLookupError):
                    os.kill(pid, 0)


if __name__ == "__main__":
    unittest.main()
