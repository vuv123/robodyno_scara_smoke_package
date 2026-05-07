import importlib
import re
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
README = PROJECT_ROOT / "README.md"


class ProjectConsistencyTest(unittest.TestCase):
    def test_expected_entrypoint_modules_import(self):
        for module in [
            "robodyno_scara_smoke",
            "robodyno_scara_smoke.webots",
            "robodyno_scara_smoke.run_webots_smoke",
            "robodyno_scara_smoke.run_webots_proto_smoke",
            "robodyno_scara_smoke.run_all_smoke_tests",
        ]:
            with self.subTest(module=module):
                self.assertIsNotNone(importlib.import_module(module))

    def test_no_legacy_robodyno_damn_references_remain(self):
        offenders = []
        needle = "robodyno" + "_damn"
        for path in PROJECT_ROOT.rglob("*"):
            if not path.is_file() or "__pycache__" in path.parts or path.name == "README.original.md" or path.suffix.lower() in {".dae", ".png", ".jpg", ".jpeg", ".txt", ".pyc"}:
                continue
            text = path.read_text(encoding="utf-8", errors="ignore")
            if path == Path(__file__):
                text = text.replace(needle, "")
            if needle in text:
                offenders.append(path.relative_to(PROJECT_ROOT).as_posix())
        self.assertEqual(offenders, [])

    def test_no_broken_python_syntax_in_project_sources(self):
        for path in sorted(PROJECT_ROOT.rglob("*.py")):
            with self.subTest(path=path.relative_to(PROJECT_ROOT)):
                compile(path.read_text(encoding="utf-8"), str(path), "exec")

    def test_readme_lists_runnable_commands(self):
        text = README.read_text(encoding="utf-8")
        for command in [
            "python -m unittest discover -s tests",
            "python -m robodyno_scara_smoke.run_webots_smoke",
            "python -m robodyno_scara_smoke.run_webots_proto_smoke",
            "python -m robodyno_scara_smoke.run_all_smoke_tests",
        ]:
            with self.subTest(command=command):
                self.assertIn(command, text)

    def test_all_protos_world_contains_every_proto_once(self):
        world = (PROJECT_ROOT / "worlds" / "AllProtosSmoke.wbt").read_text(encoding="utf-8")
        proto_names = []
        for proto in sorted((PROJECT_ROOT / "protos" / "robodyno").rglob("*.proto")):
            match = re.search(r"^PROTO\s+(\S+)\s+\[", proto.read_text(encoding="utf-8"), re.MULTILINE)
            self.assertIsNotNone(match, proto)
            proto_names.append(match.group(1))
        for name in proto_names:
            with self.subTest(proto=name):
                self.assertIn(f"{name} {{", world)
        self.assertEqual(len(proto_names), 19)

    def test_generated_logs_are_not_exceptions(self):
        for log_name in ["scara_smoke_result.txt", "all_protos_smoke_output.txt"]:
            log = PROJECT_ROOT / log_name
            if not log.exists():
                self.skipTest(f"{log_name} not generated yet")
            with self.subTest(log=log_name):
                text = log.read_text(encoding="utf-8", errors="replace")
                self.assertNotIn("Traceback", text)
                self.assertNotIn("ERROR:", text)


if __name__ == "__main__":
    unittest.main()
