"""Offline recording validation and timing audit. Never connects to robot control."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re

import numpy as np

from .lidar_geometry import decode_xyz


def read_rows(path, maximum=20000):
    rows = []
    with Path(path).open("r", encoding="utf-8") as source:
        while line := source.readline(65537):
            if len(line) > 65536 or len(rows) >= maximum:
                raise ValueError("Recording metadata exceeds limits")
            row = json.loads(line)
            if not isinstance(row, dict):
                raise ValueError("Expected recording objects")
            rows.append(row)
    return rows


def stamp_ns(row):
    sec, nano = row.get("stamp_sec"), row.get("stamp_nanosec")
    if type(sec) is not int or type(nano) is not int or not 0 <= sec <= 2**31-1 or not 0 <= nano < 1_000_000_000:
        return None
    return sec * 1_000_000_000 + nano


def host_ns(row):
    value = row.get("host_monotonic_ns")
    if not isinstance(value, str) or not re.fullmatch(r"[0-9]{1,19}", value):
        raise ValueError("Invalid host receive timestamp")
    result = int(value)
    if not 0 <= result <= 2**63-1:
        raise ValueError("Host receive timestamp exceeds int64")
    return result


def timing_summary(rows):
    stamps = [stamp_ns(row) for row in rows]
    valid = np.asarray([v for v in stamps if v is not None and v > 0], dtype=np.int64)
    arrivals = np.asarray([host_ns(row) for row in rows], dtype=np.int64)
    result = {"messages": len(rows), "positive_source_stamps": int(len(valid)),
              "zero_source_stamps": sum(v == 0 for v in stamps), "invalid_source_stamps": sum(v is None for v in stamps)}
    if len(arrivals) > 1 and arrivals[-1] > arrivals[0]:
        result["receive_hz"] = float((len(arrivals) - 1) * 1e9 / (arrivals[-1] - arrivals[0]))
    if len(valid) > 1:
        delta = np.diff(valid)
        result.update({"source_first_ns": str(int(valid[0])), "source_last_ns": str(int(valid[-1])),
                       "nonincreasing_source_stamps": int(np.count_nonzero(delta <= 0)),
                       "median_source_period_ms": float(np.median(delta) / 1e6),
                       "p95_source_period_ms": float(np.percentile(delta, 95) / 1e6),
                       "source_gaps_over_20ms": int(np.count_nonzero(delta > 20_000_000)),
                       "max_source_gap_ms": float(delta.max() / 1e6)})
        if valid[-1] > valid[0]:
            result["source_hz"] = float((len(valid) - 1) * 1e9 / (valid[-1] - valid[0]))
    return result


class SensorRecording:
    def __init__(self, directory):
        self.root = Path(directory).resolve()
        manifest_path = self.root / "manifest.json"
        if manifest_path.stat().st_size > 65536:
            raise ValueError("Oversized recording manifest")
        self.manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if self.manifest.get("schema") != 1 or self.manifest.get("read_only") is not True:
            raise ValueError("Unsupported recording schema")
        self.clouds = read_rows(self.root / "clouds.jsonl", 500)
        self.imu = read_rows(self.root / "imu.jsonl")
        self.odometry = read_rows(self.root / "odometry.jsonl")
        if self.manifest.get("cloud_files") != len(self.clouds):
            raise ValueError("Manifest/cloud index mismatch")

    def cloud_bytes(self, metadata):
        name = metadata.get("file")
        if not isinstance(name, str) or not re.fullmatch(r"cloud-[0-9]+\.bin", name):
            raise ValueError("Invalid cloud filename")
        path = (self.root / name).resolve()
        if not path.is_relative_to(self.root):
            raise ValueError("Cloud path leaves recording")
        with path.open("rb") as source:
            raw = source.read(16 * 1024 * 1024 + 9)
        if len(raw) > 16 * 1024 * 1024 + 8:
            raise ValueError("Oversized cloud")
        return raw

    def iter_clouds(self):
        """Deterministic replay iterator preserving source and receive timestamps."""
        for metadata in self.clouds:
            xyz = decode_xyz(self.cloud_bytes(metadata), metadata)
            if xyz is None:
                raise ValueError("Cloud sequence/header mismatch")
            yield metadata, xyz


def scalar_field(raw, metadata, name, formats):
    matching = [f for f in metadata["fields"] if f["name"] == name]
    if not matching:
        return None
    if len(matching) != 1:
        raise ValueError("Duplicate cloud attribute")
    field = matching[0]
    kind = formats.get(field.get("datatype"))
    if kind is None or field.get("count") != 1:
        raise ValueError("Unsupported cloud attribute")
    endian = ">" if metadata["bigendian"] else "<"
    scalar = np.dtype(endian + kind)
    offset, step = field["offset"], int(metadata["point_step"])
    if type(offset) is not int or offset < 0 or offset + scalar.itemsize > step:
        raise ValueError("Cloud attribute exceeds point step")
    dtype = np.dtype({"names": [name], "formats": [scalar], "offsets": [offset], "itemsize": step})
    points = np.ndarray((int(metadata["height"]), int(metadata["width"])), dtype=dtype,
        buffer=raw, offset=8, strides=(int(metadata["row_step"]), step))
    return points[name].reshape(-1)


def finite_vectors(rows, key, size):
    values = []
    for row in rows:
        try:
            vector = np.asarray(row.get(key), dtype=float)
        except (ValueError, TypeError):
            continue
        if vector.shape == (size,) and np.isfinite(vector).all():
            values.append(vector)
    return np.asarray(values, dtype=float).reshape(-1, size)


def nearest_stamp_gaps(clouds, imu):
    cloud_stamps = np.asarray([s for row in clouds if (s := stamp_ns(row)) is not None and s > 0], dtype=np.int64)
    imu_stamps = np.sort([s for row in imu if (s := stamp_ns(row)) is not None and s > 0])
    if not len(cloud_stamps) or not len(imu_stamps):
        return {"source_intervals_overlap": False}
    indices = np.searchsorted(imu_stamps, cloud_stamps)
    a = imu_stamps[np.clip(indices, 0, len(imu_stamps)-1)]
    b = imu_stamps[np.clip(indices-1, 0, len(imu_stamps)-1)]
    gaps = np.minimum(np.abs(a-cloud_stamps), np.abs(b-cloud_stamps)) / 1e6
    return {"source_intervals_overlap": bool(max(cloud_stamps.min(), imu_stamps.min()) <= min(cloud_stamps.max(), imu_stamps.max())),
            "nearest_imu_header_gap_ms_median": float(np.median(gaps)),
            "nearest_imu_header_gap_ms_max": float(gaps.max()),
            "hardware_synchronization_verified": False}


def scan_imu_coverage(intervals, imu):
    """Diagnostic only: assumes candidate point timestamps are source seconds."""
    stamps = np.sort([s / 1e9 for row in imu if (s := stamp_ns(row)) is not None and s > 0])
    covered, gaps = 0, []
    for low, high in intervals:
        before = np.searchsorted(stamps, low, side="right") - 1
        after = np.searchsorted(stamps, high, side="left")
        if before < 0 or after >= len(stamps):
            continue
        covered += 1
        delta = np.diff(stamps[before:after+1])
        if len(delta):
            gaps.append(float(delta.max() * 1000))
    return {"assumption": "point timestamps are source seconds; unverified",
            "scans_checked": len(intervals), "scans_bracketed": covered,
            "scans_without_full_imu_coverage": len(intervals)-covered,
            "max_bracketed_imu_gap_ms": max(gaps, default=None)}


def audit_recording(recording):
    cloud_timing = timing_summary(recording.clouds)
    imu_timing = timing_summary(recording.imu)
    odom_timing = timing_summary(recording.odometry)
    report = {"schema": 1, "slam_ready": False, "extrinsics_verified": False,
              "cloud": cloud_timing, "imu": imu_timing, "odometry": odom_timing,
              "clock_comparison": nearest_stamp_gaps(recording.clouds, recording.imu),
              "capture": recording.manifest, "blockers": [], "warnings": [], "cloud_integrity_sha256": {}}
    sizes, rings, time_spans, time_minima, time_maxima, header_deltas = [], set(), [], [], [], []
    for metadata in recording.clouds:
        raw = recording.cloud_bytes(metadata)
        xyz = decode_xyz(raw, metadata)
        if xyz is None:
            raise ValueError("Cloud sequence mismatch")
        sizes.append(len(xyz))
        report["cloud_integrity_sha256"][metadata["file"]] = hashlib.sha256(raw).hexdigest()
        ring = scalar_field(raw, metadata, "ring", {4: "u2", 2: "u1"})
        if ring is not None:
            rings.update(map(int, np.unique(ring)))
        times = scalar_field(raw, metadata, "timestamp", {7: "f4", 8: "f8"})
        if times is not None:
            times = times[np.isfinite(times)]
            if len(times):
                low, high = float(times.min()), float(times.max())
                time_minima.append(low); time_maxima.append(high); time_spans.append(high-low)
                header = stamp_ns(metadata)
                if header:
                    header_deltas.append(low - header / 1e9)
    report["cloud"].update({"frames": sorted({row.get("frame", "") for row in recording.clouds}),
                          "finite_xyz_per_scan_min": min(sizes, default=0),
                          "finite_xyz_per_scan_max": max(sizes, default=0),
                          "observed_ring_ids": sorted(rings)})
    if time_spans:
        span = float(np.median(time_spans))
        period = cloud_timing.get("median_source_period_ms", 0) / 1000
        candidates = [unit for unit, scale in (("seconds", 1), ("milliseconds", 1e-3), ("microseconds", 1e-6), ("nanoseconds", 1e-9))
                      if period > 0 and .5 * period <= span * scale <= 1.5 * period]
        report["point_time"] = {"raw_min": min(time_minima), "raw_max": max(time_maxima),
            "median_raw_scan_span": span, "duration_scale_hypotheses": candidates,
            "units_verified": False, "constant_time_scans": sum(v == 0 for v in time_spans),
            "median_raw_min_minus_header_seconds": float(np.median(header_deltas)) if header_deltas else None}
        if candidates == ["seconds"] and header_deltas and max(map(abs, header_deltas)) < .01:
            report["candidate_scan_imu_coverage"] = scan_imu_coverage(list(zip(time_minima, time_maxima)), recording.imu)
    acceleration = finite_vectors(recording.imu, "linear_acceleration", 3)
    gyro = finite_vectors(recording.imu, "angular_velocity", 3)
    orientation = finite_vectors(recording.imu, "orientation_xyzw", 4)
    report["imu"].update({"frames": sorted({row.get("frame", "") for row in recording.imu}),
        "finite_acceleration_samples": len(acceleration), "finite_gyro_samples": len(gyro),
        "unit_quaternion_samples": int(np.count_nonzero(np.abs(np.linalg.norm(orientation, axis=1)-1) < .05))})
    if len(acceleration):
        mean = acceleration.mean(axis=0)
        report["imu"].update({"mean_acceleration_raw": mean.tolist(), "acceleration_std_raw": acceleration.std(axis=0).tolist(),
            "median_acceleration_norm_raw": float(np.median(np.linalg.norm(acceleration, axis=1))),
            "dominant_static_acceleration_axis": ("+" if mean[np.argmax(np.abs(mean))] >= 0 else "-") + "XYZ"[np.argmax(np.abs(mean))],
            "acceleration_units_verified": False})
    if len(gyro):
        report["imu"].update({"mean_gyro_raw": gyro.mean(axis=0).tolist(),
            "gyro_norm_raw_p95": float(np.percentile(np.linalg.norm(gyro, axis=1), 95)), "gyro_units_verified": False})
    positions = finite_vectors(recording.odometry, "position", 3)
    angles = finite_vectors(recording.odometry, "rpy", 3)
    velocities = finite_vectors(recording.odometry, "velocity", 3)
    if len(positions) and len(angles) and len(velocities):
        shift = float(np.linalg.norm(positions-positions[0], axis=1).max())
        difference = np.arctan2(np.sin(angles-angles[0]), np.cos(angles-angles[0]))
        angular_span = float(np.abs(difference).max())
        speed = float(np.linalg.norm(velocities, axis=1).max())
        report["stationarity"] = {"max_position_shift_raw": shift, "max_rpy_change_rad": angular_span,
            "max_reported_speed_raw": speed, "odometry_consistent_with_stationary": shift < .02 and angular_span < .03 and speed < .05}
    if not report["clock_comparison"]["source_intervals_overlap"]:
        report["blockers"].append("LiDAR and IMU source timestamp intervals do not overlap")
    for name, summary in (("cloud", cloud_timing), ("imu", imu_timing)):
        if summary["positive_source_stamps"] != summary["messages"] or summary.get("nonincreasing_source_stamps", 0):
            report["blockers"].append(name + " has missing, invalid or nonincreasing source stamps")
    if not len(acceleration) or not len(gyro):
        report["blockers"].append("Finite accelerometer and gyro samples are required")
    if not time_spans or all(v == 0 for v in time_spans):
        report["blockers"].append("Nonconstant per-point timing was not verified")
    if not report["imu"]["unit_quaternion_samples"]:
        report["warnings"].append("No usable unit orientation quaternion; choose an estimator supporting raw six-axis IMU")
    if imu_timing.get("max_source_gap_ms", 0) > max(20, 3 * imu_timing.get("median_source_period_ms", 0)):
        report["warnings"].append("Large IMU source timestamp gaps; investigate delivery and driver before moving-sensor SLAM")
    cloud_period = cloud_timing.get("median_source_period_ms", 0)
    if cloud_period > 0 and cloud_timing.get("max_source_gap_ms", 0) > 1.5 * cloud_period:
        report["warnings"].append("Cloud source intervals exceed 1.5 times the median; investigate missing or irregular scans")
    if odom_timing["positive_source_stamps"] != odom_timing["messages"]:
        report["warnings"].append("Odometry lacks valid source timing; receive timestamps are not hardware synchronization")
    coverage = report.get("candidate_scan_imu_coverage", {})
    if coverage.get("scans_without_full_imu_coverage", 0):
        report["warnings"].append("Some scans lack IMU bracketing under the candidate seconds interpretation; exclude them from estimation")
    report["blockers"].extend(["Sensor model, scale, axes and LiDAR-to-IMU extrinsics are not independently verified",
                              "Point time semantics and any clock correction require driver-level verification",
                              "A stationary capture cannot calibrate extrinsics or validate moving-sensor SLAM"])
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory")
    parser.add_argument("--replay-check", action="store_true", help="Decode the dataset again without any robot access")
    args = parser.parse_args(argv)
    recording = SensorRecording(args.directory)
    report = audit_recording(recording)
    output = recording.root / "audit.json"
    output.write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k != "cloud_integrity_sha256"}, indent=2, allow_nan=False))
    print("Audit saved:", output)
    if args.replay_check:
        count = sum(1 for _ in recording.iter_clouds())
        print("Offline replay decode passed:", count, "clouds; no robot connection")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
