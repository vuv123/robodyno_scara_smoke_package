import math
import unittest

from robodyno_scara_smoke import create_scara


class DummyJoint:
    def __init__(self):
        self.pos = 0.0
        self.enabled = False

    def position_filter_mode(self, bandwidth):
        self.bandwidth = bandwidth

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def set_pos(self, pos):
        self.pos = pos

    def get_pos(self, timeout=None):
        return self.pos


class KinematicsTest(unittest.TestCase):
    def test_fk_ik_fk_roundtrip(self):
        scara = create_scara([DummyJoint() for _ in range(4)])
        samples = [(-0.02, 0.2, 0.2, 0.2), (0.015, -0.35, 0.55, -0.1)]
        for joints in samples:
            with self.subTest(joints=joints):
                pose = scara.forward_kinematics(*joints)
                solved = scara.inverse_kinematics(*pose, hand_coordinate=joints[2] >= 0)
                pose_again = scara.forward_kinematics(*solved)
                for actual, expected in zip(pose_again, pose):
                    self.assertLess(abs(actual - expected), 1e-9)

    def test_requires_four_joints(self):
        with self.assertRaises(ValueError):
            create_scara([DummyJoint() for _ in range(3)])


if __name__ == "__main__":
    unittest.main()
