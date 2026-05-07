import os
import subprocess
import time
from pathlib import Path

from .run_webots_smoke import configure_webots_environment, resolve_webots_executable


def prepare_log_path(default_path: Path) -> Path:
    if not default_path.exists():
        return default_path
    try:
        default_path.unlink()
        return default_path
    except PermissionError:
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        return default_path.with_name(f"{default_path.stem}-{timestamp}{default_path.suffix}")


def main() -> int:
    project_root = Path(__file__).resolve().parents[1]
    world = project_root / "worlds" / "AllProtosSmoke.wbt"
    output_log = prepare_log_path(project_root / "all_protos_smoke_output.txt")
    webots = resolve_webots_executable()
    if not webots.exists():
        raise FileNotFoundError(f"Webots executable not found: {webots}")

    env = os.environ.copy()
    env["PYTHONPATH"] = str(project_root) + os.pathsep + env.get("PYTHONPATH", "")
    configure_webots_environment(env, project_root)
    command = [
        str(webots),
        "--batch",
        "--mode=fast",
        "--stdout",
        "--stderr",
        f"--port={env['WEBOTS_PORT']}",
        str(world),
    ]
    with output_log.open("w", encoding="utf-8") as handle:
        process = subprocess.Popen(
            command,
            cwd=str(project_root),
            env=env,
            stdout=handle,
            stderr=subprocess.STDOUT,
            text=True,
        )
        deadline = time.monotonic() + 20
        try:
            while time.monotonic() < deadline:
                status = process.poll()
                if status is not None:
                    if status != 0:
                        raise RuntimeError(f"Webots proto smoke exited with status {status}; see {output_log}")
                    break
                time.sleep(0.5)
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()

    output = output_log.read_text(encoding="utf-8", errors="replace")
    errors = [line for line in output.splitlines() if "ERROR:" in line]
    if errors:
        raise AssertionError(f"Webots reported proto loading errors; see {output_log}")
    print(f"Webots proto library smoke passed: {output_log}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
