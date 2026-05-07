import ast
import os
import math
import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from robodyno_scara_smoke import ScaraGeometry, create_scara


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCARA_LOG = PROJECT_ROOT / "scara_smoke_result.txt"
PROTO_LOG = PROJECT_ROOT / "all_protos_smoke_output.txt"


class DummyJoint:
    def __init__(self, pos=0.0):
        self.pos = pos

    def position_filter_mode(self, bandwidth):
        pass

    def enable(self):
        pass

    def disable(self):
        pass

    def set_pos(self, pos):
        self.pos = pos

    def get_pos(self, timeout=None):
        return self.pos


class BoundaryAndLogTest(unittest.TestCase):
    def make_scara(self):
        return create_scara([DummyJoint() for _ in range(4)])

    def test_reach_boundary_singularity_does_not_crash(self):
        scara = self.make_scara()
        geometry = ScaraGeometry()
        pose = (geometry.a1 + geometry.a2 + geometry.a3, 0.0, geometry.d1 - geometry.d4, 0.0)
        solved = scara.inverse_kinematics(*pose, hand_coordinate=True)
        self.assertEqual(len(solved), 4)
        self.assertTrue(all(math.isfinite(value) for value in solved))
        pose_again = scara.forward_kinematics(*solved)
        for actual, expected in zip(pose_again, pose):
            self.assertLess(abs(actual - expected), 1e-9)

    def test_vertical_axis_extremes_roundtrip(self):
        scara = self.make_scara()
        for d in [-0.2, -0.05, 0.0, 0.05, 0.2]:
            with self.subTest(d=d):
                joints = (d, 0.4, 0.8, -0.3)
                pose = scara.forward_kinematics(*joints)
                solved = scara.inverse_kinematics(*pose, hand_coordinate=True)
                self.assertAlmostEqual(solved[0], d)
                pose_again = scara.forward_kinematics(*solved)
                for actual, expected in zip(pose_again, pose):
                    self.assertLess(abs(actual - expected), 1e-9)

    def test_scara_log_has_monotonic_target_sections(self):
        if not SCARA_LOG.exists():
            self.skipTest("run_webots_smoke has not produced a log yet")
        text = SCARA_LOG.read_text(encoding="utf-8")
        self.assertNotIn("SCARA_SMOKE: exception", text)
        self.assertIn("SCARA_SMOKE: multi target sequence ok", text)
        targets = re.findall(r"SCARA_SMOKE: target\[(\d+)\] convergence ok", text)
        self.assertEqual(targets, ["0", "1", "2"])

    def test_scara_log_final_errors_below_threshold(self):
        if not SCARA_LOG.exists():
            self.skipTest("run_webots_smoke has not produced a log yet")
        lines = SCARA_LOG.read_text(encoding="utf-8").splitlines()
        final_error_by_target = {}
        for line in lines:
            match = re.search(r"target=(\d+) step=\d+ rc=0 .* errors=(\[[^\]]+\])", line)
            if match:
                final_error_by_target[int(match.group(1))] = ast.literal_eval(match.group(2))
        self.assertEqual(set(final_error_by_target), {0, 1, 2})
        for target, errors in final_error_by_target.items():
            with self.subTest(target=target):
                self.assertLess(max(errors), 0.09)

    def test_proto_smoke_log_has_no_errors(self):
        if not PROTO_LOG.exists():
            self.skipTest("run_webots_proto_smoke has not produced a log yet")
        text = PROTO_LOG.read_text(encoding="utf-8", errors="replace")
        self.assertNotIn("ERROR:", text)

    def test_import_from_outside_project_with_pythonpath(self):
        code = "from robodyno_scara_smoke import ScaraGeometry; print(ScaraGeometry().a2)"
        with tempfile.TemporaryDirectory() as directory:
            completed = subprocess.run(
                [sys.executable, "-c", code],
                cwd=directory,
                env={**os.environ, "PYTHONPATH": str(PROJECT_ROOT)},
                text=True,
                capture_output=True,
                timeout=10,
            )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertEqual(completed.stdout.strip(), "0.15")


if __name__ == "__main__":
    unittest.main()
