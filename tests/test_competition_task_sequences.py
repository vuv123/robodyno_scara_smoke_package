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
        self.pos = float(pos)
        self.commands.append(self.pos)

    def get_pos(self, timeout=None):
        del timeout
        return self.pos


class CompetitionTaskSequenceTest(unittest.TestCase):
    def make_scara(self, positions=(0.0, 0.0, 0.0, 0.0)):
        joints = [RecordingJoint(pos) for pos in positions]
        return create_scara(joints), joints

    def assert_pose_close(self, first, second, tolerance=1e-8):
        self.assertEqual(len(first), len(second))
        for actual, expected in zip(first, second):
            self.assertLessEqual(abs(float(actual) - float(expected)), tolerance, (first, second))

    def execute_cartesian(self, scara, joints, pose, hand_coordinate=True):
        before = [joint.pos for joint in joints]
        scara.cartesian_space_interpolated_motion(*pose, hand_coordinate=hand_coordinate, duration=0)
        after = [joint.pos for joint in joints]
        self.assertTrue(all(math.isfinite(value) for value in after))
        self.assertNotEqual(before, after)
        self.assert_pose_close(scara.forward_kinematics(*after), pose)

    def test_pick_place_cycle_reaches_all_waypoints(self):
        scara, joints = self.make_scara()
        rng = random.Random(2026050803)
        with patch("robodyno.robots.four_dof_scara_robot.four_dof_scara_robot.time.sleep", return_value=None):
            for cycle in range(12):
                base_theta = rng.uniform(-1.0, 1.0)
                pick_axes = (0.015, base_theta, 0.55, rng.uniform(-0.5, 0.5))
                place_axes = (-0.015, base_theta + rng.uniform(-0.35, 0.35), 0.75, rng.uniform(-0.5, 0.5))
                waypoints = [
                    (0.045, pick_axes[1], pick_axes[2], pick_axes[3]),
                    pick_axes,
                    (0.045, pick_axes[1], pick_axes[2], pick_axes[3]),
                    (0.045, place_axes[1], place_axes[2], place_axes[3]),
                    place_axes,
                    (0.045, place_axes[1], place_axes[2], place_axes[3]),
                ]
                for waypoint_index, axes in enumerate(waypoints):
                    with self.subTest(cycle=cycle, waypoint=waypoint_index):
                        self.execute_cartesian(scara, joints, scara.forward_kinematics(*axes), hand_coordinate=True)
        self.assertEqual([len(joint.commands) for joint in joints], [72, 72, 72, 72])

    def test_recovery_after_unreachable_waypoint_continues_to_next_valid_target(self):
        scara, joints = self.make_scara((0.01, 0.20, 0.40, -0.10))
        start = [joint.pos for joint in joints]
        valid_axes = (-0.02, -0.45, 0.70, 0.25)
        valid_pose = scara.forward_kinematics(*valid_axes)
        with patch("robodyno.robots.four_dof_scara_robot.four_dof_scara_robot.time.sleep", return_value=None):
            scara.cartesian_space_interpolated_motion(99.0, 99.0, 99.0, 0.0, duration=0)
            self.assert_pose_close([joint.pos for joint in joints], start)
            self.execute_cartesian(scara, joints, valid_pose, hand_coordinate=True)

    def test_yaw_wrap_task_sequence_preserves_xy_z(self):
        scara, joints = self.make_scara()
        axes = (0.0, 0.35, 0.85, -0.20)
        base_pose = scara.forward_kinematics(*axes)
        with patch("robodyno.robots.four_dof_scara_robot.four_dof_scara_robot.time.sleep", return_value=None):
            for turn in range(-3, 4):
                yaw_pose = (base_pose[0], base_pose[1], base_pose[2], base_pose[3] + turn * 2 * math.pi)
                scara.cartesian_space_interpolated_motion(*yaw_pose, hand_coordinate=True, duration=0)
                actual_pose = scara.forward_kinematics(*[joint.pos for joint in joints])
                with self.subTest(turn=turn):
                    self.assert_pose_close(actual_pose[:3], base_pose[:3])
                    self.assertTrue(math.isfinite(actual_pose[3]))


if __name__ == "__main__":
    unittest.main()
