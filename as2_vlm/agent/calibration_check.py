"""Offline calibration evidence check, not an extrinsic estimator or motion tool."""
import argparse
import json
from pathlib import Path

import numpy as np

from .sensor_audit import SensorRecording, audit_recording, finite_vectors


KEYS = {"schema", "sensor_model", "hardware_binding", "lidar_length_unit",
        "imu_acceleration_unit", "imu_angular_velocity_unit", "point_time_unit",
        "point_time_reference", "imu_time_minus_lidar_time_s", "T_imu_from_lidar", "provenance"}


def profile_issues(profile):
    """Check completeness and numerical consistency, never certify calibration."""
    if not isinstance(profile, dict) or set(profile) != KEYS or type(profile.get("schema")) is not int or profile["schema"] != 1:
        return ["Unsupported calibration profile schema or keys"]
    issues = []
    for key in ("sensor_model", "hardware_binding", "provenance"):
        value = profile[key]
        if not isinstance(value, str) or not value.strip() or len(value) > 2048:
            issues.append(key + " requires an explicit evidence reference")
    units = {"lidar_length_unit": {"m"}, "imu_acceleration_unit": {"m/s^2", "g"},
             "imu_angular_velocity_unit": {"rad/s", "deg/s"},
             "point_time_unit": {"s", "ms", "us", "ns"},
             "point_time_reference": {"absolute_source_time", "relative_to_scan_start", "relative_to_scan_end"}}
    for key, allowed in units.items():
        if not isinstance(profile[key], str) or profile[key] not in allowed:
            issues.append(key + " is missing or unsupported")
    offset = profile["imu_time_minus_lidar_time_s"]
    if type(offset) not in (int, float) or not np.isfinite(offset):
        issues.append("A finite explicit IMU-minus-LiDAR time offset is required; zero must not be assumed")
    transform = profile["T_imu_from_lidar"]
    if not isinstance(transform, dict) or set(transform) != {"rotation", "translation_m"}:
        issues.append("T_imu_from_lidar requires rotation and translation_m")
    else:
        try:
            rotation = np.asarray(transform["rotation"], dtype=float)
            translation = np.asarray(transform["translation_m"], dtype=float)
            # Strings and booleans are not physical numeric coefficients.
            flat = list(np.asarray(transform["rotation"], dtype=object).flat) + list(np.asarray(transform["translation_m"], dtype=object).flat)
            numeric = all(type(value) in (int, float) for value in flat)
            if not numeric or rotation.shape != (3, 3) or translation.shape != (3,) or not np.isfinite(rotation).all() or not np.isfinite(translation).all():
                raise ValueError("Invalid transform shape or coefficients")
            if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5, rtol=0) or not np.isclose(np.linalg.det(rotation), 1, atol=1e-5, rtol=0):
                raise ValueError("Rotation is not a proper SO(3) matrix")
        except (TypeError, ValueError) as exc:
            issues.append(str(exc))
    return issues


def check_recording(recording, profile):
    audit = audit_recording(recording)
    issues = profile_issues(profile)
    acceleration = finite_vectors(recording.imu, "linear_acceleration", 3)
    observed = None
    if len(acceleration):
        mean = acceleration.mean(axis=0)
        norm = float(np.linalg.norm(mean))
        if norm > 1e-9:
            observed = {"mean_raw": mean.tolist(), "mean_norm_raw": norm,
                        "unit_direction_in_imu_message_axes": (mean / norm).tolist(),
                        "dominant_axis": audit["imu"]["dominant_static_acceleration_axis"],
                        "note": "Specific-force observation only; not a LiDAR axis or extrinsic estimate"}
    stationary = audit.get("stationarity", {}).get("odometry_consistent_with_stationary")
    return {"schema": 1, "read_only": True, "profile_complete": not issues,
            "calibration_verified": False, "slam_ready": False, "navigation_ready": False,
            "profile_issues": issues, "static_acceleration_observation": observed,
            "odometry_consistent_with_stationary": stationary,
            "stream_frames": {"lidar": audit["cloud"]["frames"], "imu": audit["imu"]["frames"]},
            "same_frame_name_proves_extrinsics": False,
            "candidate_scan_imu_coverage": audit.get("candidate_scan_imu_coverage"),
            "timing_warnings": audit["warnings"],
            "not_estimated": ["LiDAR-to-IMU rotation", "LiDAR-to-IMU translation",
                              "LiDAR-to-body transform", "camera extrinsics/intrinsics", "clock offset"],
            "limitations": ["A static acceleration direction alone leaves yaw and translation unconstrained",
                            "Reported body stationarity is not a calibrated sensor-to-body transform",
                            "A complete, numerically valid profile is not evidence that it matches this hardware",
                            "No profile from this checker is applied to live geometry or navigation"]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory")
    parser.add_argument("--profile", default="configs/calibration.example.json")
    args = parser.parse_args(argv)
    path = Path(args.profile)
    if path.stat().st_size > 65536:
        raise ValueError("Oversized calibration profile")
    profile = json.loads(path.read_text(encoding="utf-8"))
    recording = SensorRecording(args.directory)
    report = check_recording(recording, profile)
    output = recording.root / "calibration-report.json"
    output.write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps(report, indent=2, allow_nan=False))
    print("Evidence report saved:", output)
    print("Check completed; calibration and SLAM remain unverified.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
