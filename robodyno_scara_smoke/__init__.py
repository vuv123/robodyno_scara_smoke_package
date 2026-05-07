from dataclasses import dataclass
from typing import Optional, Sequence

from robodyno.robots.four_dof_scara_robot import FourDoFScara


@dataclass(frozen=True)
class ScaraGeometry:
    d1: float = 0.24
    a1: float = 0.06
    a2: float = 0.150
    a3: float = 0.150
    d4: float = 0.045


def create_scara(
    joints: Sequence[object], geometry: Optional[ScaraGeometry] = None
) -> FourDoFScara:
    if len(joints) != 4:
        raise ValueError("SCARA construction requires exactly four joints")
    geometry = geometry or ScaraGeometry()
    return FourDoFScara(
        joints[0],
        joints[1],
        joints[2],
        joints[3],
        geometry.d1,
        geometry.a1,
        geometry.a2,
        geometry.a3,
        geometry.d4,
    )


def create_webots_scara(webots, geometry: Optional[ScaraGeometry] = None) -> FourDoFScara:
    from .webots import create_webots_joints

    return create_scara(create_webots_joints(webots), geometry)


__all__ = ["ScaraGeometry", "create_scara", "create_webots_scara"]
