"""Verify teammate profile without starting board inference or cloud ASR."""
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
YAML_NAME = "yolov8-pose+yolov8-face+yolov8-finger+hdmi-192.168.126.10.yaml"
SCRIPT_NAME = "run_board_192.168.126.10.sh"


class TeammateNetworkTests(unittest.TestCase):
    def test_yaml_only_changes_four_udp_hosts_and_files_use_lf(self):
        source = (PACKAGE / "configs/ZG" / YAML_NAME.replace(".10.yaml", ".100.yaml")).read_text(encoding="utf-8")
        target_path = PACKAGE / "configs/ZG" / YAML_NAME
        target = target_path.read_text(encoding="utf-8")
        self.assertEqual(target, source.replace("192.168.126.100", "192.168.126.10"))
        self.assertEqual(target.count("host: 192.168.126.10"), 4)
        self.assertNotIn(b"\r", target_path.read_bytes())
        self.assertNotIn(b"\r", (PACKAGE / "scripts" / SCRIPT_NAME).read_bytes())

    @unittest.skipIf(os.name == "nt", "Linux bash execution")
    def test_wrapper_routes_voice_uart_and_yaml_despite_stale_environment(self):
        with tempfile.TemporaryDirectory(prefix="teammate space ") as directory:
            root = Path(directory)
            (root / "scripts").mkdir()
            script = root / "scripts" / SCRIPT_NAME
            shutil.copyfile(str(PACKAGE / "scripts" / SCRIPT_NAME), str(script))
            capture = "import json,os,sys; print(json.dumps(dict(args=sys.argv[1:],voice=os.environ.get('VOICE_HOST'),enabled=os.environ.get('VOICE_ENABLED'))))"
            (root / "scripts/run_board.sh").write_text(
                "#!/usr/bin/env bash\nexec " + shlex.quote(sys.executable) + " -c " + shlex.quote(capture) + ' "$@"\n')
            for uart in (None, "", "/dev/ttyACM7"):
                with self.subTest(uart=uart):
                    env = dict(os.environ, VOICE_HOST="192.168.126.100", MODE_HOST="192.168.126.91", VOICE_ENABLED="0")
                    env.pop("UART_DEVICE", None)
                    if uart is not None:
                        env["UART_DEVICE"] = uart
                    result = subprocess.run(["bash", str(script)], cwd=str(root.parent), env=env,
                                            text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=5)
                    self.assertEqual(result.returncode, 0, result.stdout)
                    captured = json.loads(result.stdout)
                    self.assertEqual(captured["voice"], "192.168.126.10")
                    self.assertEqual(captured["args"], [
                        str(root / "configs/ZG" / YAML_NAME),
                        "/dev/ttyACM0" if uart is None else uart,
                        "192.168.126.10",
                    ])
                    self.assertEqual(captured["enabled"], "0")


if __name__ == "__main__":
    unittest.main()
