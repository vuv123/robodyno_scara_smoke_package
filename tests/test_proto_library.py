import re
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PROTO_ROOT = PROJECT_ROOT / "protos" / "robodyno"
WORLD = PROJECT_ROOT / "worlds" / "ScaraLocalSmoke_r2023b.wbt"


class ProtoLibraryTest(unittest.TestCase):
    def test_all_proto_files_have_assets(self):
        protos = sorted(PROTO_ROOT.rglob("*.proto"))
        self.assertEqual(len(protos), 19)
        for proto in protos:
            with self.subTest(proto=proto.relative_to(PROJECT_ROOT)):
                text = proto.read_text(encoding="utf-8")
                for quoted in re.findall(r'"([^"]+)"', text):
                    if not quoted.lower().endswith((".dae", ".png", ".jpg", ".jpeg")):
                        continue
                    asset = (proto.parent / quoted).resolve()
                    self.assertTrue(asset.exists(), f"missing asset {quoted}")

    def test_externproto_references_exist(self):
        files = [WORLD] + sorted(PROTO_ROOT.rglob("*.proto"))
        for source in files:
            text = source.read_text(encoding="utf-8")
            for reference in re.findall(r'EXTERNPROTO\s+"([^"]+)"', text):
                with self.subTest(source=source.relative_to(PROJECT_ROOT), reference=reference):
                    self.assertTrue((source.parent / reference).resolve().exists())

    def test_world_uses_available_joint_protos(self):
        text = WORLD.read_text(encoding="utf-8")
        self.assertIn("Pro_JP44.proto", text)
        self.assertIn("Pro_JP12.proto", text)
        self.assertNotIn("Pro_P44(J)", text)
        self.assertNotIn("Pro_P12(J)", text)

    def test_slider_module_exposes_local_slider_sensor(self):
        text = (PROTO_ROOT / "slider" / "SliderModule.proto").read_text(encoding="utf-8")
        self.assertIn('name %<= "\\\"" + fields.motor_id.value + "::slider\\\"" >%', text)
        self.assertIn('name %<= "\\\"" + fields.motor_id.value + "::slider_sensor\\\"" >%', text)


if __name__ == "__main__":
    unittest.main()
