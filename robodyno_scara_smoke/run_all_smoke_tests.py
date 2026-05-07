import subprocess
import sys
from pathlib import Path


COMMANDS = [
    [sys.executable, "-m", "unittest", "discover", "-s", "tests"],
    [sys.executable, "-B", "-m", "robodyno_scara_smoke.run_webots_proto_smoke"],
    [sys.executable, "-B", "-m", "robodyno_scara_smoke.run_webots_smoke"],
]


def main() -> int:
    project_root = Path(__file__).resolve().parents[1]
    for command in COMMANDS:
        print("RUN", " ".join(command), flush=True)
        completed = subprocess.run(command, cwd=str(project_root))
        if completed.returncode != 0:
            return completed.returncode
    print("All robodyno SCARA smoke tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
