"""Read-only local checks; never opens a robot connection."""
import argparse
import json
import platform
import shutil
import sys
from pathlib import Path


def collect_report(sdk_root=None):
    system, machine = platform.system(), platform.machine()
    arch = {"arm64": "aarch64", "amd64": "x86_64"}.get(machine.lower(), machine.lower())
    checks = {
        "linux": system == "Linux",
        "supported_architecture": arch in {"aarch64", "x86_64"},
        "python_310_plus": sys.version_info >= (3, 10),
        "cmake_available": shutil.which("cmake") is not None,
        "cxx_compiler_available": shutil.which("g++") is not None,
    }
    if sdk_root is not None:
        root = Path(sdk_root)
        checks["as2_header_present"] = (root / "include/unitree/robot/as2/sport/sport_client.hpp").is_file()
        checks["sdk_library_present"] = any((root / "lib" / arch).glob("libunitree_sdk2.*"))
    else:
        checks["as2_header_present"] = False
        checks["sdk_library_present"] = False
    return {
        "schema_version": 1,
        "status": "LOCAL_CHECKS_PASSED" if all(checks.values()) else "LOCAL_CHECKS_INCOMPLETE",
        "system": system,
        "architecture": arch,
        "python_version": platform.python_version(),
        "checks": checks,
        "hardware_connected": False,
        "motion_enabled": False,
        "unverified": ["firmware", "sdk_compatibility", "sensor_access", "network", "operator_override"],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk-root", help="Official SDK source checkout (not install prefix)")
    args = parser.parse_args()
    report = collect_report(args.sdk_root)
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "LOCAL_CHECKS_PASSED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
