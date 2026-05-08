import random
import time
from dataclasses import dataclass
from math import isfinite

import sitecustomize

from robodyno.robots.four_dof_palletizing_robot import FourDoFPallet
from robodyno.robots.six_dof_collaborative_robot import SixDoFCollabRobot
from robodyno.robots.three_dof_delta_robot import ThreeDoFDelta
from robodyno.robots.three_dof_palletizing_robot import ThreeDoFPallet

del sitecustomize


class FakeJoint:
    def __init__(self):
        self.pos = 0.0

    def position_filter_mode(self, *args):
        pass

    def position_track_mode(self, *args, **kwargs):
        pass

    def enable(self):
        pass

    def disable(self):
        pass

    def set_pos(self, pos):
        self.pos = pos

    def get_pos(self, timeout=None):
        del timeout
        return self.pos


@dataclass(frozen=True)
class ModelSpec:
    name: str
    factory: object
    joint_ranges: tuple
    tolerance: float


@dataclass(frozen=True)
class StressResult:
    model: str
    cases: int
    worst_error: float
    elapsed_seconds: float
    failed_axes: tuple | None = None
    failed_pose: tuple | None = None
    failed_error: float | None = None

    @property
    def passed(self):
        return self.failed_axes is None


def _three_dof_pallet():
    return ThreeDoFPallet(FakeJoint(), FakeJoint(), FakeJoint(), 0.12, 0.10, 0.10, 0.04)


def _four_dof_pallet():
    return FourDoFPallet(FakeJoint(), FakeJoint(), FakeJoint(), FakeJoint(), 0.12, 0.10, 0.10, 0.04)


def _three_dof_delta():
    return ThreeDoFDelta(FakeJoint(), FakeJoint(), FakeJoint(), 0.12, 0.28, 0.08, 0.03)


def _six_dof_collab():
    return SixDoFCollabRobot(
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


MODEL_SPECS = (
    ModelSpec("ThreeDoFPallet", _three_dof_pallet, ((-3.0, 3.0), (-2.2, 2.2), (-2.4, 2.4)), 1e-6),
    ModelSpec("FourDoFPallet", _four_dof_pallet, ((-2.0, 2.0), (-1.2, 1.2), (-1.2, 1.2), (-2.0, 2.0)), 1e-6),
    ModelSpec("ThreeDoFDelta", _three_dof_delta, ((-0.9, 0.9), (-0.9, 0.9), (-0.9, 0.9)), 1e-6),
    ModelSpec(
        "SixDoFCollabRobot",
        _six_dof_collab,
        ((-1.2, 1.2), (-1.2, 1.2), (-1.2, 1.2), (-1.2, 1.2), (-1.2, 1.2), (-1.2, 1.2)),
        1e-6,
    ),
)


REGRESSION_CASES = {
    "ThreeDoFPallet": (
        (-2.6262193309662463, 1.7077827688478502, -0.8938803568395368),
        (3.052805577527237, 1.0683749425200153, -2.1532949926654927),
        (2.4722809967985953, 0.6367117838577552, 1.5814887953011194),
        (0.810167440720738, 2.042404416573577, 1.5843399934906577),
    ),
    "ThreeDoFDelta": (
        (0.07154808037361648, 0.36703725941325915, -0.3218783761687096),
        (0.6142266825333944, -0.7907372975614462, 0.2745582161662594),
        (0.33922390986767126, 0.7140833249187561, 0.48754694533109366),
    ),
    "SixDoFCollabRobot": (
        (
            -0.9957851743690496,
            5.6261337907097015,
            5.937147395643123,
            -5.157427632905599,
            -0.7966032724536909,
            3.028680854932075,
        ),
    ),
}


def _pose_error(first, second):
    return max(abs(float(actual) - float(expected)) for actual, expected in zip(first, second))


def _finite(values):
    return all(isfinite(float(value)) for value in values)


def assert_roundtrip(robot, axes, tolerance):
    pose = robot.forward_kinematics(list(axes))
    solved = robot.inverse_kinematics(*pose)
    solved_pose = robot.forward_kinematics(list(solved))
    error = _pose_error(solved_pose, pose)
    if not _finite(pose) or not _finite(solved) or not _finite(solved_pose) or error > tolerance:
        raise AssertionError((axes, pose, solved, solved_pose, error))
    return error, tuple(float(value) for value in pose)


def regression_specs():
    return tuple((spec, REGRESSION_CASES.get(spec.name, ())) for spec in MODEL_SPECS)


def random_axes(spec, rng):
    return tuple(rng.uniform(low, high) for low, high in spec.joint_ranges)


def run_stress(iterations=500, seed=20260508, include_regressions=True):
    rng = random.Random(seed)
    results = []
    for spec in MODEL_SPECS:
        start = time.perf_counter()
        robot = spec.factory()
        cases = list(REGRESSION_CASES.get(spec.name, ())) if include_regressions else []
        cases.extend(random_axes(spec, rng) for _ in range(iterations))
        worst_error = 0.0
        failed_axes = None
        failed_pose = None
        failed_error = None

        for axes in cases:
            try:
                error, pose = assert_roundtrip(robot, axes, spec.tolerance)
            except AssertionError as exc:
                detail = exc.args[0]
                failed_axes = tuple(float(value) for value in detail[0])
                failed_pose = tuple(float(value) for value in detail[1])
                failed_error = float(detail[4])
                worst_error = max(worst_error, failed_error)
                break
            worst_error = max(worst_error, error)

        results.append(
            StressResult(
                model=spec.name,
                cases=len(cases) if failed_axes is None else cases.index(axes) + 1,
                worst_error=worst_error,
                elapsed_seconds=time.perf_counter() - start,
                failed_axes=failed_axes,
                failed_pose=failed_pose,
                failed_error=failed_error,
            )
        )
    return tuple(results)
