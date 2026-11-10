import json
from pathlib import Path
import struct
import tempfile
import unittest

from as2_vlm.agent.sensor_audit import (
    SensorRecording, audit_recording, host_ns, nearest_stamp_gaps,
    read_rows, scan_imu_coverage, stamp_ns, timing_summary,
)


def timed(sec, nano=0):
    return {"stamp_sec": sec, "stamp_nanosec": nano,
            "host_monotonic_ns": str(sec * 1_000_000_000 + nano)}


class SensorAuditTests(unittest.TestCase):
    def test_timestamp_bounds(self):
        self.assertEqual(stamp_ns(timed(0)), 0)
        for sec, nano in ((True, 0), (-1, 0), (2**40, 0), (1, 10**9)):
            self.assertIsNone(stamp_ns(timed(sec, nano)))
        with self.assertRaises(ValueError):
            host_ns({"host_monotonic_ns": str(2**63)})

    def test_source_zero_and_nonincreasing(self):
        result = timing_summary([timed(0), timed(2), timed(2), timed(1)])
        self.assertEqual(result["zero_source_stamps"], 1)
        self.assertEqual(result["nonincreasing_source_stamps"], 2)

    def test_disjoint_clocks(self):
        self.assertFalse(nearest_stamp_gaps([timed(1)], [timed(3)])["source_intervals_overlap"])
        self.assertFalse(nearest_stamp_gaps([], [timed(3)])["source_intervals_overlap"])

    def test_imu_gap_diagnostics(self):
        result = timing_summary([timed(1, nano) for nano in (0, 5_000_000, 10_000_000, 65_000_000)])
        self.assertEqual(result["source_gaps_over_20ms"], 1)
        self.assertEqual(result["max_source_gap_ms"], 55)

    def test_scan_requires_both_imu_boundaries(self):
        result = scan_imu_coverage([(1, 1.1), (1.1, 1.2), (1.2, 1.3)],
                                   [timed(1, 50_000_000), timed(1, 150_000_000), timed(1, 250_000_000)])
        self.assertEqual(result["scans_bracketed"], 1)
        self.assertEqual(result["scans_without_full_imu_coverage"], 2)
        self.assertAlmostEqual(result["max_bracketed_imu_gap_ms"], 100)

    def test_metadata_limits(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "rows.jsonl"
            path.write_text("[]\n", encoding="utf-8")
            with self.assertRaises(ValueError):
                read_rows(path)
            path.write_text(" " * 65537, encoding="utf-8")
            with self.assertRaises(ValueError):
                read_rows(path)

    def test_recording_decode_audit_and_sequence_integrity(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            meta = dict(timed(1), file="cloud-1.bin", cloud_sequence=1, width=2,
                        height=1, point_step=22, row_step=44, bigendian=False, frame="unverified",
                        fields=[{"name": name, "offset": offset, "datatype": kind, "count": 1}
                                for name, offset, kind in (("x", 0, 7), ("y", 4, 7), ("z", 8, 7),
                                                           ("ring", 12, 4), ("timestamp", 14, 8))])
            raw = struct.pack("<Q", 1) + b"".join(struct.pack("<fffHd", 1, 2, 3, ring, time)
                                                       for ring, time in ((0, 1.), (127, 1.1)))
            (root / "cloud-1.bin").write_bytes(raw)
            (root / "manifest.json").write_text(json.dumps({"schema": 1, "read_only": True, "cloud_files": 1}))
            (root / "clouds.jsonl").write_text(json.dumps(meta) + "\n")
            imu = [dict(timed(1, nano), linear_acceleration=[0, 9.8, 0],
                        angular_velocity=[0, 0, 0], orientation_xyzw=[0, 0, 0, 0])
                   for nano in (0, 50_000_000, 100_000_000)]
            (root / "imu.jsonl").write_text("".join(json.dumps(row) + "\n" for row in imu))
            (root / "odometry.jsonl").write_text(json.dumps(dict(timed(0), position=[0, 0, 0],
                                                              rpy=[0, 0, 0], velocity=[0, 0, 0])) + "\n")
            recording = SensorRecording(root)
            self.assertEqual(len(list(recording.iter_clouds())), 1)
            report = audit_recording(recording)
            self.assertFalse(report["slam_ready"])
            self.assertEqual(report["cloud"]["observed_ring_ids"], [0, 127])
            self.assertEqual(report["imu"]["dominant_static_acceleration_axis"], "+Y")
            self.assertTrue(any("quaternion" in warning for warning in report["warnings"]))
            self.assertTrue(any("Odometry" in warning for warning in report["warnings"]))
            with self.assertRaises(ValueError):
                recording.cloud_bytes({"file": "../cloud-1.bin"})
            (root / "cloud-1.bin").write_bytes(struct.pack("<Q", 2) + raw[8:])
            with self.assertRaises(ValueError):
                list(recording.iter_clouds())


if __name__ == "__main__":
    unittest.main()
