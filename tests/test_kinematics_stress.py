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


if __name__ == "__main__":
    unittest.main()
