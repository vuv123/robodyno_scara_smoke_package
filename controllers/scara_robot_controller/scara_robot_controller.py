from pathlib import Path
import os
import sys
import traceback

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

LOG = Path(os.environ.get("SCARA_SMOKE_LOG", PROJECT_ROOT / "scara_smoke_result.txt"))


def log(message):
    with LOG.open("a", encoding="utf-8") as handle:
        handle.write(str(message) + "\n")
        handle.flush()


webots = None
try:
    LOG.write_text("SCARA_SMOKE: controller started\n", encoding="utf-8")
    from robodyno.interfaces import Webots
    from robodyno_scara_smoke import create_scara
    from robodyno_scara_smoke.webots import create_webots_joints

    log("SCARA_SMOKE: imports ok")
    webots = Webots()
    log("SCARA_SMOKE: webots constructed")

    actuators = create_webots_joints(webots)
    scara = create_scara(actuators)
    log("SCARA_SMOKE: scara constructed")

    samples = [(-0.02, 0.2, 0.2, 0.2), (0.015, -0.35, 0.55, -0.1)]
    for sample in samples:
        pose = scara.forward_kinematics(*sample)
        solved = scara.inverse_kinematics(*pose, hand_coordinate=sample[2] >= 0)
        pose_again = scara.forward_kinematics(*solved)
        max_error = max(abs(a - b) for a, b in zip(pose, pose_again))
        if max_error > 1e-9:
            raise AssertionError(f"FK/IK roundtrip error {max_error}")
    log("SCARA_SMOKE: fk/ik roundtrip ok")

    for actuator in actuators:
        actuator.enable()
    scara.enable()
    log("SCARA_SMOKE: actuators enabled")

    target_sequences = [
        [-0.02, 0.2, 0.2, 0.2],
        [0.015, -0.25, 0.45, -0.1],
        [0.0, 0.0, 0.0, 0.0],
    ]
    for target_index, targets in enumerate(target_sequences):
        for index, target in enumerate(targets):
            scara.set_joint_pos(index, target)
        log(f"SCARA_SMOKE: targets[{target_index}] issued {targets}")

        positions = [0.0, 0.0, 0.0, 0.0]
        for step in range(100):
            rc = webots.sleep(0.05)
            positions = [round(actuator.get_pos(), 5) for actuator in actuators]
            errors = [round(abs(position - target), 5) for position, target in zip(positions, targets)]
            log(f"SCARA_SMOKE: target={target_index} step={step} rc={rc} positions={positions} errors={errors}")
            if rc == -1:
                break
            if max(errors) < 0.09:
                break

        final_errors = [abs(position - target) for position, target in zip(positions, targets)]
        if max(final_errors) >= 0.55:
            raise AssertionError(
                f"Actuator feedback did not converge enough for target {target_index}: {final_errors}"
            )
        log(f"SCARA_SMOKE: target[{target_index}] convergence ok")

    log("SCARA_SMOKE: multi target sequence ok")
    log("SCARA_SMOKE: done")
    webots.stop()
except Exception:
    log("SCARA_SMOKE: exception")
    log(traceback.format_exc())
    if webots is not None:
        try:
            webots.stop()
        except Exception:
            pass
    raise
