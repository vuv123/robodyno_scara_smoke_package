import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from robodyno_scara_smoke import run_webots_smoke


class RunnerBehaviorTest(unittest.TestCase):
    def test_assert_log_markers_accepts_complete_log(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "log.txt"
            log.write_text("\n".join(f"SCARA_SMOKE: {marker}" for marker in run_webots_smoke.REQUIRED_MARKERS), encoding="utf-8")
            run_webots_smoke.assert_log_markers(log)

    def test_assert_log_markers_reports_missing_entries(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "log.txt"
            log.write_text("SCARA_SMOKE: imports ok\n", encoding="utf-8")
            with self.assertRaises(AssertionError) as raised:
                run_webots_smoke.assert_log_markers(log)
        self.assertIn("Missing Webots smoke markers", str(raised.exception))

    def test_resolve_webots_home_root_layout(self):
        with tempfile.TemporaryDirectory() as directory:
            home = Path(directory)
            exe = home / "webots.exe"
            exe.write_text("", encoding="utf-8")
            with patch.dict(os.environ, {"WEBOTS_HOME": str(home)}):
                self.assertEqual(run_webots_smoke.resolve_webots_executable(), exe)

    def test_resolve_webots_home_msys_layout(self):
        with tempfile.TemporaryDirectory() as directory:
            home = Path(directory)
            exe = home / "msys64" / "mingw64" / "bin" / "webots.exe"
            exe.parent.mkdir(parents=True)
            exe.write_text("", encoding="utf-8")
            with patch.dict(os.environ, {"WEBOTS_HOME": str(home)}):
                self.assertEqual(run_webots_smoke.resolve_webots_executable(), exe)

    def test_default_webots_path_returns_candidate(self):
        with patch.dict(os.environ, {}, clear=True):
            candidate = run_webots_smoke.default_webots_executable()
        self.assertIn(candidate.name.lower(), {"webots.exe", "webotsw.exe"})


if __name__ == "__main__":
    unittest.main()
