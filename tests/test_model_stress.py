import unittest

from robodyno_scara_smoke.model_stress import regression_specs, run_stress, assert_roundtrip


class InstalledModelStressTest(unittest.TestCase):
    def test_known_regression_seeds_roundtrip(self):
        for spec, cases in regression_specs():
            robot = spec.factory()
            for axes in cases:
                with self.subTest(model=spec.name, axes=axes):
                    assert_roundtrip(robot, axes, spec.tolerance)

    def test_stress_runner_reports_all_models(self):
        results = run_stress(iterations=2, seed=20260508)
        self.assertEqual(
            [result.model for result in results],
            ["ThreeDoFPallet", "FourDoFPallet", "ThreeDoFDelta", "SixDoFCollabRobot"],
        )
        self.assertTrue(all(result.passed for result in results), results)
        self.assertTrue(all(result.cases >= 2 for result in results), results)


if __name__ == "__main__":
    unittest.main()
