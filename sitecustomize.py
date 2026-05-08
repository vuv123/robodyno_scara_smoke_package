from itertools import product
from math import atan, atan2, isfinite, sqrt

import numpy as np


def _as_float_array(values):
    return np.array([float(value) for value in values], dtype=float)


def _max_pose_error(robot, target_pose, axes):
    try:
        pose = _as_float_array(robot.forward_kinematics(list(axes)))
    except TypeError:
        try:
            pose = _as_float_array(robot.forward_kinematics(*list(axes)))
        except Exception:
            return float("inf")
    except Exception:
        return float("inf")
    return float(np.max(np.abs(pose - _as_float_array(target_pose))))


def _finite(values):
    return all(isfinite(float(value)) for value in values)


def _refine_by_forward_kinematics(robot, target_pose, seed, iterations=40, tolerance=1e-10):
    target = _as_float_array(target_pose)
    axes = _as_float_array(seed)
    best_axes = axes.copy()
    best_error = _max_pose_error(robot, target, best_axes)

    for _ in range(iterations):
        try:
            current = _as_float_array(robot.forward_kinematics(list(axes)))
        except Exception:
            break
        residual = current - target
        error = float(np.max(np.abs(residual)))
        if error <= tolerance:
            return list(axes)

        jacobian = np.zeros((len(target), len(axes)), dtype=float)
        epsilon = 1e-6
        for index in range(len(axes)):
            delta = np.zeros(len(axes), dtype=float)
            delta[index] = epsilon
            try:
                plus = _as_float_array(robot.forward_kinematics(list(axes + delta)))
                minus = _as_float_array(robot.forward_kinematics(list(axes - delta)))
            except Exception:
                return list(best_axes)
            jacobian[:, index] = (plus - minus) / (2 * epsilon)

        try:
            step = np.linalg.lstsq(jacobian, residual, rcond=None)[0]
        except np.linalg.LinAlgError:
            break

        improved = False
        for scale in (1.0, 0.5, 0.25, 0.1, 0.05, 0.01, 0.005):
            candidate = axes - step * scale
            candidate_error = _max_pose_error(robot, target, candidate)
            if candidate_error < best_error:
                axes = candidate
                best_axes = candidate.copy()
                best_error = candidate_error
                improved = True
                break
        if not improved:
            break

    return list(best_axes)


def _patch_three_dof_delta():
    from robodyno.robots.three_dof_delta_robot import ThreeDoFDelta

    original_inverse = ThreeDoFDelta.inverse_kinematics

    def inverse_kinematics(self, x, y, z):
        initial = original_inverse(self, x, y, z)
        target = (x, y, z)
        if _finite(initial) and _max_pose_error(self, target, initial) <= 1e-8:
            return initial

        m = (
            x * x
            + y * y
            + z * z
            + (self.r1 - self.r2) * (self.r1 - self.r2)
            + self.l1 * self.l1
            - self.l2 * self.l2
        )
        root3 = sqrt(3)
        a_values = [
            (m - 2 * x * (self.r1 - self.r2)) / (2 * self.l1) - (self.r1 - self.r2 - x),
            (m + (self.r1 - self.r2) * (x - root3 * y)) / self.l1 - 2 * (self.r1 - self.r2) - (x - root3 * y),
            (m + (self.r1 - self.r2) * (x + root3 * y)) / self.l1 - 2 * (self.r1 - self.r2) - (x + root3 * y),
        ]
        b_values = [2 * z, 4 * z, 4 * z]
        c_values = [
            (m - 2 * x * (self.r1 - self.r2)) / (2 * self.l1) + (self.r1 - self.r2 - x),
            (m + (self.r1 - self.r2) * (x - root3 * y)) / self.l1 + 2 * (self.r1 - self.r2) + (x - root3 * y),
            (m + (self.r1 - self.r2) * (x + root3 * y)) / self.l1 + 2 * (self.r1 - self.r2) + (x + root3 * y),
        ]

        root_options = []
        for a_value, b_value, c_value in zip(a_values, b_values, c_values):
            discriminant = b_value * b_value - 4 * a_value * c_value
            if discriminant < 0:
                root_options = []
                break
            root = sqrt(discriminant)
            options = []
            for sign in (-1, 1):
                t = (-b_value + sign * root) / (2 * a_value)
                options.append(2 * atan(t) - (np.pi / 2 + self._link_deviation))
            root_options.append(options)

        if root_options:
            candidates = [list(candidate) for candidate in product(*root_options)]
            best = min(candidates, key=lambda axes: _max_pose_error(self, target, axes))
            best_error = _max_pose_error(self, target, best)
            if best_error < _max_pose_error(self, target, initial):
                initial = best
                initial_error = best_error

        seeds = [-1.0, -0.5, 0.0, 0.5, 1.0]
        best_refined = initial
        best_error = _max_pose_error(self, target, best_refined)
        for seed in product(seeds, repeat=3):
            refined = _refine_by_forward_kinematics(self, target, seed, iterations=60)
            if not _finite(refined):
                continue
            refined_error = _max_pose_error(self, target, refined)
            if refined_error < best_error:
                best_refined = refined
                best_error = refined_error
                if best_error <= 1e-8:
                    return best_refined

        if best_error < _max_pose_error(self, target, initial):
            return best_refined
        return initial

    ThreeDoFDelta.inverse_kinematics = inverse_kinematics


def _patch_three_dof_pallet():
    from robodyno.robots.three_dof_palletizing_robot import ThreeDoFPallet

    original_inverse = ThreeDoFPallet.inverse_kinematics

    def inverse_kinematics(self, x, y, z):
        initial = original_inverse(self, x, y, z)
        target = (x, y, z)
        if not _finite(initial):
            return initial

        initial_error = _max_pose_error(self, target, initial)
        if initial_error <= 1e-8:
            return initial

        cf = z - self.l01 + self.l45
        planar = sqrt(x * x + y * y)
        reach = sqrt(planar * planar + cf * cf)
        cos_alpha = (reach * reach + self.l23 * self.l23 - self.l34 * self.l34) / (2 * reach * self.l23)
        cos_abc = (self.l23 * self.l23 + self.l34 * self.l34 - reach * reach) / (2 * self.l34 * self.l23)
        if abs(cos_alpha) > 1.0 or abs(cos_abc) > 1.0:
            return initial

        alpha = np.arccos(cos_alpha)
        abc = np.arccos(cos_abc)
        base_values = [atan2(planar, z), atan2(z, planar), -atan2(planar, z), -atan2(z, planar)]
        third_values = [
            abc - np.pi / 2,
            np.pi / 2 - abc,
            -abc - np.pi / 2,
            abc + np.pi / 2,
            -abc + np.pi / 2,
            np.pi / 2 + abc,
        ]

        candidates = []
        for base in base_values:
            for sign in (-1, 1):
                theta2 = base + sign * alpha
                for theta3 in third_values:
                    for theta2_shift in range(-3, 4):
                        for theta3_shift in range(-3, 4):
                            seed = [
                                atan2(y, x),
                                theta2 + theta2_shift * 2 * np.pi,
                                theta3 + theta3_shift * 2 * np.pi,
                            ]
                            candidates.append((_max_pose_error(self, target, seed), seed))

        candidates.append((initial_error, initial))
        candidates = [item for item in sorted(candidates, key=lambda item: item[0]) if item[0] <= 0.5]
        candidates = candidates[:24]

        for _, seed in candidates:
            refined = _refine_by_forward_kinematics(self, target, seed)
            if _finite(refined):
                refined_error = _max_pose_error(self, target, refined)
                if refined_error <= 1e-8:
                    return refined
                if refined_error < initial_error:
                    initial = refined
                    initial_error = refined_error

        return initial

    ThreeDoFPallet.inverse_kinematics = inverse_kinematics


def _patch_six_dof_collab():
    from robodyno.robots.six_dof_collaborative_robot import SixDoFCollabRobot

    original_inverse = SixDoFCollabRobot.inverse_kinematics

    def inverse_kinematics(self, x, y, z, roll, pitch, yaw, sol_id=2):
        target = (x, y, z, roll, pitch, yaw)
        initial = original_inverse(self, x, y, z, roll, pitch, yaw, sol_id)
        if _finite(initial) and _max_pose_error(self, target, initial) <= 1e-8:
            return initial

        candidates = []
        for candidate_id in range(8):
            candidate = original_inverse(self, x, y, z, roll, pitch, yaw, candidate_id)
            if _finite(candidate):
                candidates.append(candidate)
        if not candidates:
            return initial

        best = min(candidates, key=lambda axes: _max_pose_error(self, target, axes))
        if _max_pose_error(self, target, best) < _max_pose_error(self, target, initial):
            return best
        return initial

    SixDoFCollabRobot.inverse_kinematics = inverse_kinematics


try:
    _patch_three_dof_delta()
    _patch_three_dof_pallet()
    _patch_six_dof_collab()
except Exception:
    pass
