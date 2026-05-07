import math
import random
import unittest

from robodyno_scara_smoke import create_scara


class DummyJoint:
    def __init__(self):
        self.pos = 0.0

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


class KinematicsStressTest(unittest.TestCase):
    def setUp(self):
        self.scara = create_scara([DummyJoint() for _ in range(4)])

    def assert_pose_close(self, first, second, tolerance=1e-9):
        for actual, expected in zip(first, second):
            self.assertLess(abs(actual - expected), tolerance)

    def test_seeded_fk_ik_roundtrip_grid(self):
        random.seed(240618)
        for index in range(100):
            d = random.uniform(-0.04, 0.04)
            theta2 = random.uniform(-1.2, 1.2)
            theta3 = random.uniform(0.08, 1.8)
            theta4 = random.uniform(-1.5, 1.5)
            with self.subTest(index=index):
                joints = (d, theta2, theta3, theta4)
                pose = self.scara.forward_kinematics(*joints)
                solved = self.scara.inverse_kinematics(*pose, hand_coordinate=True)
                self.assert_pose_close(self.scara.forward_kinematics(*solved), pose)

    def test_left_hand_seeded_fk_ik_roundtrip_grid(self):
        random.seed(20260429)
        for index in range(100):
            d = random.uniform(-0.04, 0.04)
            theta2 = random.uniform(-1.2, 1.2)
            theta3 = random.uniform(-1.8, -0.08)
            theta4 = random.uniform(-1.5, 1.5)
            with self.subTest(index=index):
                joints = (d, theta2, theta3, theta4)
                pose = self.scara.forward_kinematics(*joints)
                solved = self.scara.inverse_kinematics(*pose, hand_coordinate=False)
                self.assert_pose_close(self.scara.forward_kinematics(*solved), pose)

    def test_yaw_continuity_for_repeated_full_turns(self):
        base = self.scara.forward_kinematics(0.0, 0.4, 0.6, 0.2)
        for turns in range(-2, 3):
            pose = (base[0], base[1], base[2], base[3] + turns * 2 * math.pi)
            solved = self.scara.inverse_kinematics(*pose, hand_coordinate=True)
            pose_again = self.scara.forward_kinematics(*solved)
            self.assertLess(abs(pose_again[0] - base[0]), 1e-9)
            self.assertLess(abs(pose_again[1] - base[1]), 1e-9)
            self.assertLess(abs(pose_again[2] - base[2]), 1e-9)

    def test_workspace_boundary_roundtrip_cases(self):
        cases = [
            (-0.05, -1.45, 0.10, -math.pi),
            (0.05, 1.45, 1.95, math.pi),
            (0.0, -1.50, 1.80, 2.8),
            (0.03, 1.50, 0.12, -2.8),
        ]
        for joints in cases:
            with self.subTest(joints=joints):
                pose = self.scara.forward_kinematics(*joints)
                solved = self.scara.inverse_kinematics(*pose, hand_coordinate=joints[2] > 0)
                self.assert_pose_close(self.scara.forward_kinematics(*solved), pose, tolerance=1e-8)

    def test_unreachable_xy_position_returns_current_positions(self):
        joints = [DummyJoint() for _ in range(4)]
        for joint, position in zip(joints, [0.01, 0.02, 0.03, 0.04]):
            joint.pos = position
        scara = create_scara(joints)
        self.assertEqual(scara.inverse_kinematics(10.0, 10.0, 0.0, 0.0), [0.01, 0.02, 0.03, 0.04])

    def test_batch_joint_commands_preserve_last_commanded_positions(self):
        joints = [DummyJoint() for _ in range(4)]
        scara = create_scara(joints)
        command_sequence = [
            (0.0, 0.1, 0.2, 0.3),
            (0.02, -0.4, 0.7, -0.8),
            (-0.03, 0.5, -0.6, 0.9),
        ]
        for command in command_sequence:
            scara.joint_space_interpolated_motion(command, duration=0)

        for joint, expected in zip(joints, command_sequence[-1]):
            self.assertAlmostEqual(joint.pos, expected)


if __name__ == "__main__":
    unittest.main()
