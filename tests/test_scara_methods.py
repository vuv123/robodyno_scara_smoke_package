import math
import unittest
from unittest.mock import patch

from robodyno_scara_smoke import ScaraGeometry, create_scara


class RecordingJoint:
    def __init__(self, pos=0.0):
        self.pos = pos
        self.enabled = False
        self.disabled = False
        self.filter_modes = []
        self.commands = []

    def position_filter_mode(self, bandwidth):
        self.filter_modes.append(bandwidth)

    def enable(self):
        self.enabled = True

    def disable(self):
        self.disabled = True

    def set_pos(self, pos):
        self.pos = pos
        self.commands.append(pos)

    def get_pos(self, timeout=None):
        return self.pos


class ScaraMethodCoverageTest(unittest.TestCase):
    def make_scara(self, positions=(0.0, 0.0, 0.0, 0.0), geometry=None):
        joints = [RecordingJoint(pos) for pos in positions]
        return create_scara(joints, geometry), joints

    def test_geometry_defaults_and_override(self):
        default = ScaraGeometry()
        self.assertEqual((default.d1, default.a1, default.a2, default.a3, default.d4), (0.24, 0.06, 0.15, 0.15, 0.045))
        geometry = ScaraGeometry(d1=1, a1=2, a2=3, a3=4, d4=5)
        scara, _ = self.make_scara(geometry=geometry)
        self.assertEqual((scara.d1, scara.a1, scara.a2, scara.a3, scara.d4), (1, 2, 3, 4, 5))

    def test_constructor_configures_rotary_joint_filters(self):
        _, joints = self.make_scara()
        self.assertEqual(joints[0].filter_modes, [])
        self.assertEqual(joints[1].filter_modes, [8])
        self.assertEqual(joints[2].filter_modes, [8])
        self.assertEqual(joints[3].filter_modes, [8])

    def test_enable_disable_forward_to_all_joints(self):
        scara, joints = self.make_scara()
        scara.enable()
        scara.disable()
        self.assertTrue(all(joint.enabled for joint in joints))
        self.assertTrue(all(joint.disabled for joint in joints))

    def test_init_zero_offsets_and_get_joints_poses(self):
        scara, joints = self.make_scara((0.1, 0.2, 0.3, 0.4))
        scara.init([0.01, 0.02, 0.03, 0.04])
        for actual, expected in zip(scara.get_joints_poses(), [0.01, 0.02, 0.03, 0.04]):
            self.assertAlmostEqual(actual, expected)
        scara.set_joint_pos(2, 0.5)
        self.assertAlmostEqual(joints[2].pos, 0.77)
        self.assertAlmostEqual(scara.get_joints_poses()[2], 0.5)

    def test_get_joints_poses_retries_and_fails(self):
        class MissingJoint(RecordingJoint):
            def get_pos(self, timeout=None):
                return None

        scara = create_scara([MissingJoint() for _ in range(4)])
        with self.assertRaises(RuntimeError):
            scara.get_joints_poses()

    def test_inverse_kinematics_left_and_right_hands(self):
        scara, _ = self.make_scara()
        right_source = (-0.01, 0.25, 0.35, -0.15)
        pose = scara.forward_kinematics(*right_source)
        right = scara.inverse_kinematics(*pose, hand_coordinate=True)
        left = scara.inverse_kinematics(*pose, hand_coordinate=False)
        self.assertGreater(right[2], 0)
        self.assertLess(left[2], 0)
        for solved in (right, left):
            pose_again = scara.forward_kinematics(*solved)
            for actual, expected in zip(pose_again, pose):
                self.assertLess(abs(actual - expected), 1e-9)

    def test_inverse_kinematics_out_of_range_returns_current_positions(self):
        scara, _ = self.make_scara((0.01, 0.02, 0.03, 0.04))
        self.assertEqual(scara.inverse_kinematics(99, 99, 99, 0), [0.01, 0.02, 0.03, 0.04])

    def test_joint_space_interpolated_motion_and_home(self):
        scara, joints = self.make_scara()
        with patch("robodyno.robots.four_dof_scara_robot.four_dof_scara_robot.time.sleep", return_value=None):
            scara.joint_space_interpolated_motion([0.1, 0.2, 0.3, 0.4], duration=0)
            scara.home(duration=0)
        self.assertEqual([joint.pos for joint in joints], [0, 0, 0, 0])
        self.assertTrue(all(joint.commands for joint in joints))

    def test_cartesian_space_interpolated_motion(self):
        scara, joints = self.make_scara()
        target_axes = (0.02, -0.25, 0.45, 0.1)
        target_pose = scara.forward_kinematics(*target_axes)
        with patch("robodyno.robots.four_dof_scara_robot.four_dof_scara_robot.time.sleep", return_value=None):
            scara.cartesian_space_interpolated_motion(*target_pose, hand_coordinate=True, duration=0)
        pose_after = scara.forward_kinematics(*[joint.pos for joint in joints])
        for actual, expected in zip(pose_after, target_pose):
            self.assertLess(abs(actual - expected), 1e-9)

    def test_joint_space_interpolated_motion_generates_smooth_steps(self):
        scara, joints = self.make_scara()
        target = [0.04, -0.35, 0.55, -0.25]
        with patch("robodyno.robots.four_dof_scara_robot.four_dof_scara_robot.time.sleep", return_value=None):
            scara.joint_space_interpolated_motion(target, duration=0.5)

        command_lengths = {len(joint.commands) for joint in joints}
        self.assertEqual(len(command_lengths), 1)
        self.assertGreater(command_lengths.pop(), 2)
        for joint, expected in zip(joints, target):
            self.assertAlmostEqual(joint.commands[-1], expected)
            deltas = [abs(after - before) for before, after in zip(joint.commands, joint.commands[1:])]
            self.assertTrue(all(delta <= abs(expected) for delta in deltas))

    def test_cartesian_motion_preserves_pose_across_both_hand_solutions(self):
        scara, joints = self.make_scara()
        target_axes = (-0.01, 0.35, -0.55, 0.2)
        target_pose = scara.forward_kinematics(*target_axes)

        with patch("robodyno.robots.four_dof_scara_robot.four_dof_scara_robot.time.sleep", return_value=None):
            scara.cartesian_space_interpolated_motion(*target_pose, hand_coordinate=False, duration=0)

        pose_after = scara.forward_kinematics(*[joint.pos for joint in joints])
        for actual, expected in zip(pose_after, target_pose):
            self.assertLess(abs(actual - expected), 1e-9)
        self.assertLess(joints[2].pos, 0)


if __name__ == "__main__":
    unittest.main()
