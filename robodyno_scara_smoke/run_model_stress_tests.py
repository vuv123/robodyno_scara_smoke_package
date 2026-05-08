import argparse
import sys

from .model_stress import run_stress


def main(argv=None):
    parser = argparse.ArgumentParser(description="Run installed Robodyno model FK/IK stress tests.")
    parser.add_argument("--iterations", type=int, default=500)
    parser.add_argument("--seed", type=int, default=20260508)
    args = parser.parse_args(argv)

    results = run_stress(iterations=args.iterations, seed=args.seed)
    failed = False
    for result in results:
        status = "OK" if result.passed else "FAIL"
        print(
            f"{status} {result.model}: cases={result.cases} "
            f"worst_error={result.worst_error:.3e} elapsed={result.elapsed_seconds:.2f}s"
        )
        if not result.passed:
            failed = True
            print(f"  failed_axes={result.failed_axes}")
            print(f"  failed_pose={result.failed_pose}")
            print(f"  failed_error={result.failed_error:.3e}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
