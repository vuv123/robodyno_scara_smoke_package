import math
import random
import unittest
from unittest.mock import patch

from robodyno_scara_smoke import create_scara


class RecordingJoint:
    def __init__(self, pos=0.0):
        self.pos = pos
        self.commands = []

    def position_filter_mode(self, bandwidth):
        pass

    def enable(self):
        pass

    def disable(self):
        pass

    def set_pos(self, pos):
        self.pos = pos
        self.commands.append(float(pos))

    def get_pos(self, timeout=None):
        del timeout
        return self.pos


class ScaraTrajectoryStressTest(unittest.TestCase):
    def make_scara(self, positions=(0.0, 0.0, 0.0, 0.0)):
        joints = [RecordingJoint(pos) for pos in positions]
        return create_scara(joints), joints

    def assert_pose_close(self, first, second, tolerance=1e-8):
        self.assertEqual(len(first), len(second))
        for actual, expected in zip(first, second):
            self.assertLessEqual(abs(float(actual) - float(expected)), tolerance, (first, second))

    def test_cartesian_trajectory_sequence_preserves_pose_targets(self):
        scara, joints = self.make_scara()
        rng = random.Random(2026050801)
        with patch("robodyno.robots.four_dof_scara_robot.four_dof_scara_robot.time.sleep", return_value=None):
            for index in range(80):
                hand_coordinate = index % 2 == 0
                theta3 = rng.uniform(0.18, 1.65) if hand_coordinate else rng.uniform(-1.65, -0.18)
                axes = (
                    rng.uniform(-0.035, 0.035),
                    rng.uniform(-1.25, 1.25),
                    theta3,
                    rng.uniform(-1.7, 1.7),
                )
                target_pose = scara.forward_kinematics(*axes)
                scara.cartesian_space_interpolated_motion(*target_pose, hand_coordinate=hand_coordinate, duration=0)
                actual_pose = scara.forward_kinematics(*[joint.pos for joint in joints])
                with self.subTest(index=index, hand_coordinate=hand_coordinate):
                    self.assert_pose_close(actual_pose, target_pose)
                    self.assertEqual([len(joint.commands) for joint in joints], [index + 1] * 4)

    def test_joint_trajectory_sequence_preserves_final_command(self):
        scara, joints = self.make_scara()
        rng = random.Random(2026050802)
        expected = None
        with patch("robodyno.robots.four_dof_scara_robot.four_dof_scara_robot.time.sleep", return_value=None):
            for index in range(120):
                expected = [
                    rng.uniform(-0.045, 0.045),
                    rng.uniform(-1.45, 1.45),
                    rng.choice((-1, 1)) * rng.uniform(0.10, 1.85),
                    rng.uniform(-math.pi, math.pi),
                ]
                scara.joint_space_interpolated_motion(expected, duration=0)
                with self.subTest(index=index):
                    self.assert_pose_close([joint.pos for joint in joints], expected)
        self.assert_pose_close(scara.get_joints_poses(), expected)

    def test_boundary_near_singularity_cases_remain_finite(self):
        scara, joints = self.make_scara()
        cases = [
            (-0.049, -1.49, 0.101, -math.pi),
            (0.049, 1.49, 1.899, math.pi),
            (-0.049, 1.49, -0.101, math.pi - 0.01),
            (0.049, -1.49, -1.899, -math.pi + 0.01),
        ]
        with patch("robodyno.robots.four_dof_scara_robot.four_dof_scara_robot.time.sleep", return_value=None):
            for axes in cases:
                pose = scara.forward_kinematics(*axes)
                hand_coordinate = axes[2] > 0
                scara.cartesian_space_interpolated_motion(*pose, hand_coordinate=hand_coordinate, duration=0)
                actual_pose = scara.forward_kinematics(*[joint.pos for joint in joints])
                with self.subTest(axes=axes):
                    self.assertTrue(all(math.isfinite(value) for value in actual_pose))
                    self.assert_pose_close(actual_pose, pose, tolerance=1e-7)

    def test_unreachable_cartesian_target_does_not_move_joints(self):
        scara, joints = self.make_scara((0.01, 0.02, 0.03, 0.04))
        with patch("robodyno.robots.four_dof_scara_robot.four_dof_scara_robot.time.sleep", return_value=None):
            scara.cartesian_space_interpolated_motion(99.0, 99.0, 99.0, 0.0, duration=0)
        self.assert_pose_close([joint.pos for joint in joints], [0.01, 0.02, 0.03, 0.04])


if __name__ == "__main__":
    unittest.main()
