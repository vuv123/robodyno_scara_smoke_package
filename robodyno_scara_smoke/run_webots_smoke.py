import os
import subprocess
import sys
import time
from pathlib import Path


REQUIRED_MARKERS = [
    "imports ok",
    "webots constructed",
    "scara constructed",
    "fk/ik roundtrip ok",
    "actuators enabled",
    "multi target sequence ok",
    "target[0] convergence ok",
    "target[1] convergence ok",
    "target[2] convergence ok",
    "done",
]


def prepare_log_path(default_path: Path) -> Path:
    if not default_path.exists():
        return default_path
    try:
        default_path.unlink()
        return default_path
    except PermissionError:
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        return default_path.with_name(f"{default_path.stem}-{timestamp}{default_path.suffix}")


def default_webots_executable() -> Path:
    candidates = [
        Path("D:/webotmcp/webots/Webots/msys64/mingw64/bin/webotsw.exe"),
        Path("D:/webots/Webots/msys64/mingw64/bin/webotsw.exe"),
        Path("D:/webotmcp/webots/Webots/msys64/mingw64/bin/webots.exe"),
        Path("D:/webots/Webots/msys64/mingw64/bin/webots.exe"),
        Path("D:/Webots/Webots/msys64/mingw64/bin/webots.exe"),
        Path("D:/webots/Webots/webots.exe"),
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def resolve_webots_executable() -> Path:
    if os.environ.get("WEBOTS_HOME"):
        webots_home = Path(os.environ["WEBOTS_HOME"])
        candidates = [
            webots_home / "msys64" / "mingw64" / "bin" / "webotsw.exe",
            webots_home / "msys64" / "mingw64" / "bin" / "webots.exe",
            webots_home / "webots.exe",
        ]
        for candidate in candidates:
            if candidate.exists():
                return candidate
        return candidates[0]
    return default_webots_executable()


def assert_log_markers(log: Path) -> None:
    text = log.read_text(encoding="utf-8")
    missing = [marker for marker in REQUIRED_MARKERS if marker not in text]
    if missing:
        raise AssertionError(f"Missing Webots smoke markers: {missing}; see {log}")


def configure_webots_environment(env: dict[str, str], project_root: Path) -> None:
    scratch = project_root / ".webots_tmp"
    scratch.mkdir(exist_ok=True)
    env.setdefault("TMP", str(scratch))
    env.setdefault("TEMP", str(scratch))
    env.setdefault("WEBOTS_PORT", "1259")
    env.setdefault("QT_OPENGL", "software")
    env.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
    env.setdefault("WEBOTS_DISABLE_GPU", "1")


def main() -> int:
    project_root = Path(__file__).resolve().parents[1]
    world = project_root / "worlds" / "ScaraLocalSmoke_r2023b.wbt"
    log = prepare_log_path(project_root / "scara_smoke_result.txt")
    webots = resolve_webots_executable()

    if not webots.exists():
        raise FileNotFoundError(f"Webots executable not found: {webots}")

    env = os.environ.copy()
    env["PYTHONPATH"] = str(project_root) + os.pathsep + env.get("PYTHONPATH", "")
    env["SCARA_SMOKE_LOG"] = str(log)
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
    process = subprocess.Popen(command, cwd=str(project_root), env=env)
    deadline = time.monotonic() + 120
    try:
        while time.monotonic() < deadline:
            if log.exists():
                text = log.read_text(encoding="utf-8")
                if "SCARA_SMOKE: exception" in text:
                    raise RuntimeError(f"Webots controller failed; see {log}")
                if "SCARA_SMOKE: done" in text:
                    assert_log_markers(log)
                    print(f"Webots SCARA smoke passed: {log}")
                    return 0
            status = process.poll()
            if status is not None:
                if status != 0:
                    raise RuntimeError(f"Webots exited with status {status}; see {log}")
                break
            time.sleep(0.25)
        if process.poll() is None:
            raise TimeoutError(f"Timed out waiting for Webots smoke log; see {log}")
        assert_log_markers(log)
        print(f"Webots SCARA smoke passed: {log}")
        return 0
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()


if __name__ == "__main__":
    raise SystemExit(main())
