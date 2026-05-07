import threading
import unittest
from unittest.mock import patch

from robodyno_scara_smoke.webots import WebotsLinearSlider, create_webots_joints


class FakeSensor:
    def __init__(self, value=0.0):
        self.value = value
        self.enabled_steps = []

    def enable(self, step):
        self.enabled_steps.append(step)

    def getValue(self):
        return self.value


class FakeLinearMotor:
    def __init__(self):
        self.positions = []
        self.velocities = []

    def setVelocity(self, velocity):
        self.velocities.append(velocity)

    def setPosition(self, position):
        self.positions.append(position)


class FakeRobot:
    def __init__(self):
        self.devices = {
            "0x10::slider": FakeLinearMotor(),
            "0x10::slider_sensor": FakeSensor(0.123),
        }

    def getDevice(self, name):
        return self.devices.get(name)


class FakeWebots:
    def __init__(self):
        self.robot = FakeRobot()
        self.time_step = 32
        self.registered = []
        self.step_lock = threading.RLock()

    def register(self, device):
        self.registered.append(device)


class FakeMotor:
    def __init__(self, webots, id_, type_):
        self.webots = webots
        self.id = id_
        self.type = type_
        self.filter_modes = []

    def position_filter_mode(self, bandwidth):
        self.filter_modes.append(bandwidth)


class WebotsAdapterTest(unittest.TestCase):
    def test_linear_slider_device_lifecycle(self):
        webots = FakeWebots()
        slider = WebotsLinearSlider(webots, 0x10, max_vel=0.05)
        self.assertIs(webots.registered[0], slider)
        self.assertEqual(webots.robot.devices["0x10::slider_sensor"].enabled_steps, [32])
        self.assertEqual(webots.robot.devices["0x10::slider"].velocities, [0.05])

        slider.set_pos(-0.02)
        slider.enable()
        slider.step()
        self.assertEqual(webots.robot.devices["0x10::slider"].positions, [-0.02, -0.02])
        self.assertEqual(slider.get_pos(), 0.123)
        slider.position_track_mode(-0.03)
        self.assertEqual(webots.robot.devices["0x10::slider"].velocities[-1], 0.03)
        slider.disable()
        self.assertEqual(webots.robot.devices["0x10::slider"].velocities[-1], 0.0)

    def test_linear_slider_requires_devices(self):
        webots = FakeWebots()
        del webots.robot.devices["0x10::slider"]
        with self.assertRaises(RuntimeError):
            WebotsLinearSlider(webots, 0x10)

        webots = FakeWebots()
        del webots.robot.devices["0x10::slider_sensor"]
        with self.assertRaises(RuntimeError):
            WebotsLinearSlider(webots, 0x10)

    def test_create_webots_joints_forces_expected_motor_models(self):
        webots = FakeWebots()
        with patch("robodyno_scara_smoke.webots.Motor", FakeMotor):
            joints = create_webots_joints(webots)
        self.assertIsInstance(joints[0], WebotsLinearSlider)
        self.assertEqual([joint.id for joint in joints[1:]], [0x11, 0x12, 0x13])
        self.assertEqual([joint.type for joint in joints[1:]], ["ROBODYNO_PRO_01B", "ROBODYNO_PRO_01B", "ROBODYNO_PLUS_P12"])
        self.assertEqual([joint.filter_modes for joint in joints[1:]], [[8], [8], [8]])

    def test_linear_slider_concurrent_commands_keep_valid_target_and_feedback(self):
        webots = FakeWebots()
        slider = WebotsLinearSlider(webots, 0x10, max_vel=0.05)
        targets = [-0.01, -0.02, 0.03, 0.0, 0.04]

        def command_and_step(target):
            slider.set_pos(target)
            slider.step()

        threads = [threading.Thread(target=command_and_step, args=(target,)) for target in targets]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        positions = webots.robot.devices["0x10::slider"].positions
        self.assertGreaterEqual(len(positions), len(targets))
        self.assertIn(positions[-1], targets)
        self.assertIn(slider._input_pos, targets)
        self.assertEqual(slider.get_pos(), 0.123)


if __name__ == "__main__":
    unittest.main()
