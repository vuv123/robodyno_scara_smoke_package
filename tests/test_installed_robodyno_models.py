import unittest
from math import isclose, isfinite

from robodyno.robots.four_dof_palletizing_robot import FourDoFPallet
from robodyno.robots.six_dof_collaborative_robot import SixDoFCollabRobot
from robodyno.robots.three_dof_cartesian_robot import ThreeDoFCartesian
from robodyno.robots.three_dof_delta_robot import ThreeDoFDelta
from robodyno.robots.three_dof_palletizing_robot import ThreeDoFPallet


class FakeJoint:
    def __init__(self):
        self.pos = 0.0
        self.calls = []

    def position_filter_mode(self, *args):
        self.calls.append(("position_filter_mode", args))

    def position_track_mode(self, *args, **kwargs):
        self.calls.append(("position_track_mode", args, kwargs))

    def enable(self):
        self.calls.append(("enable",))

    def disable(self):
        self.calls.append(("disable",))

    def set_pos(self, pos):
        self.pos = pos
        self.calls.append(("set_pos", pos))

    def get_pos(self, timeout=None):
        del timeout
        return self.pos


def assert_finite_sequence(testcase, values):
    testcase.assertTrue(all(isfinite(float(value)) for value in values), values)


def assert_sequences_close(testcase, left, right, tolerance=1e-7):
    testcase.assertEqual(len(left), len(right))
    for actual, expected in zip(left, right):
        testcase.assertTrue(
            isclose(float(actual), float(expected), abs_tol=tolerance),
            (left, right),
        )


class InstalledRobodynoModelTest(unittest.TestCase):
    def test_three_dof_cartesian_fk_ik_roundtrip(self):
        robot = ThreeDoFCartesian(FakeJoint(), FakeJoint(), FakeJoint(), 0.01, 0.01, 0.01)
        axes = [0.1, -0.2, 0.3]
        fk = robot.forward_kinematics(axes)
        ik = robot.inverse_kinematics(*fk)
        assert_sequences_close(self, ik, axes)
        assert_sequences_close(self, robot.forward_kinematics(ik), fk)

    def test_three_dof_palletizing_fk_ik_finite_solution(self):
        robot = ThreeDoFPallet(FakeJoint(), FakeJoint(), FakeJoint(), 0.12, 0.10, 0.10, 0.04)
        fk = robot.forward_kinematics([0.1, 0.2, -0.1])
        ik = robot.inverse_kinematics(*fk)
        assert_finite_sequence(self, fk)
        assert_finite_sequence(self, ik)

    def test_four_dof_palletizing_fk_ik_roundtrip(self):
        robot = FourDoFPallet(FakeJoint(), FakeJoint(), FakeJoint(), FakeJoint(), 0.12, 0.10, 0.10, 0.04)
        axes = [0.1, 0.2, -0.1, 0.3]
        fk = robot.forward_kinematics(axes)
        ik = robot.inverse_kinematics(*fk)
        assert_sequences_close(self, robot.forward_kinematics(ik), fk)

    def test_three_dof_delta_fk_ik_roundtrip(self):
        robot = ThreeDoFDelta(FakeJoint(), FakeJoint(), FakeJoint(), 0.12, 0.28, 0.08, 0.03)
        fk = robot.forward_kinematics([0.1, 0.1, 0.1])
        ik = robot.inverse_kinematics(*fk)
        assert_sequences_close(self, robot.forward_kinematics(ik), fk)

    def test_six_dof_collaborative_fk_ik_roundtrip(self):
        robot = SixDoFCollabRobot(
            FakeJoint(),
            FakeJoint(),
            FakeJoint(),
            FakeJoint(),
            FakeJoint(),
            FakeJoint(),
            0.10,
            0.12,
            0.12,
            0.10,
            0.08,
            0.06,
        )
        axes = [0.1, -0.2, 0.2, -0.1, 0.1, 0.2]
        fk = robot.forward_kinematics(axes)
        ik = robot.inverse_kinematics(*fk)
        assert_sequences_close(self, robot.forward_kinematics(ik), fk)

    def test_basic_joint_methods_forward_to_all_models(self):
        model_specs = [
            (ThreeDoFCartesian, 3, [0.01, 0.01, 0.01]),
            (ThreeDoFPallet, 3, [0.12, 0.10, 0.10, 0.04]),
            (FourDoFPallet, 4, [0.12, 0.10, 0.10, 0.04]),
            (ThreeDoFDelta, 3, [0.12, 0.28, 0.08, 0.03]),
            (SixDoFCollabRobot, 6, [0.10, 0.12, 0.12, 0.10, 0.08, 0.06]),
        ]
        for robot_class, joint_count, dimensions in model_specs:
            with self.subTest(robot=robot_class.__name__):
                joints = [FakeJoint() for _ in range(joint_count)]
                robot = robot_class(*joints, *dimensions)
                robot.enable()
                robot.disable()
                robot.init([0.0] * joint_count)
                robot.set_joint_pos(0, 0.25)
                self.assertTrue(any(call[0] == "enable" for call in joints[0].calls))
                self.assertTrue(any(call[0] == "disable" for call in joints[0].calls))
                self.assertAlmostEqual(joints[0].pos, 0.25)


if __name__ == "__main__":
    unittest.main()
