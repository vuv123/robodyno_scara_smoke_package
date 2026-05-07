from math import pi
from typing import Optional

from robodyno.components.webots.motor import Motor


class WebotsLinearSlider:
    def __init__(
        self,
        webots,
        id_: int = 0x10,
        max_vel: float = 0.02,
        type_: str = "ROBODYNO_PRO_01B",
    ):
        self._webots = webots
        self.id = id_
        self._name = f"0x{id_:02X}::slider"
        self._sensor_name = f"0x{id_:02X}::slider_sensor"
        self._slider = webots.robot.getDevice(self._name)
        self._sensor = webots.robot.getDevice(self._sensor_name)
        if self._slider is None:
            raise RuntimeError(f"Webots device {self._name!r} not found")
        if self._sensor is None:
            raise RuntimeError(f"Webots device {self._sensor_name!r} not found")
        self._sensor.enable(int(webots.time_step))
        self._input_pos = 0.0
        self._pos_feedback = 0.0
        self.type = type_
        self.set_max_vel(max_vel)
        webots.register(self)

    def position_filter_mode(self, bandwidth: float) -> None:
        del bandwidth

    def position_track_mode(self, max_vel: float, acc: float = 40, dec: float = 40) -> None:
        del acc, dec
        self.set_max_vel(max_vel)

    def set_max_vel(self, max_vel: float) -> None:
        self._slider.setVelocity(abs(max_vel))

    def enable(self) -> None:
        self._slider.setPosition(self._input_pos)

    def disable(self) -> None:
        self._slider.setVelocity(0.0)

    def set_pos(self, pos: float) -> None:
        self._input_pos = pos
        self._slider.setPosition(pos)

    def get_pos(self, timeout: Optional[float] = None) -> float:
        del timeout
        return self._pos_feedback

    def parallel_step(self) -> None:
        pass

    def step(self) -> None:
        self._pos_feedback = self._sensor.getValue()


def create_webots_joints(webots):
    slider = WebotsLinearSlider(webots, 0x10)
    joint1 = Motor(webots, 0x11, "ROBODYNO_PRO_01B")
    joint2 = Motor(webots, 0x12, "ROBODYNO_PRO_01B")
    joint3 = Motor(webots, 0x13, "ROBODYNO_PLUS_P12")
    for joint in (joint1, joint2, joint3):
        joint.position_filter_mode(8)
    return [slider, joint1, joint2, joint3]
