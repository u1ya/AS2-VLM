"""Experimental conservative LiDAR movement gate, independent of image inference.

Unknown/stale/unobserved directions deny motion. Calibration and robot dimensions
must be measured; the module never estimates them from a camera or assumes axes.
This is a software interlock, not an independently certified emergency stop.
"""
import copy
import json
from pathlib import Path
import threading
import time

import numpy as np


class ObstacleGuard:
    def __init__(self, profile=None, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.RLock()
        self.profile = profile
        self.last = -float("inf")
        self.sequence = None
        self.result = self.unavailable("lidar_body_calibration_required")
        if profile is not None:
            from .camera_lidar import CameraLidarCalibration
            keys = {"schema", "lidar_frame", "length_unit", "T_body_from_lidar", "floor_z_m",
                    "half_length_m", "half_width_m", "clearance_m", "evidence"}
            if (not isinstance(profile, dict) or set(profile) != keys or
                    type(profile["schema"]) is not int or profile["schema"] != 1 or profile["length_unit"] != "m"):
                raise ValueError("Invalid body/LiDAR calibration schema or units")
            if any(not isinstance(profile[k], str) or not profile[k].strip() for k in ("lidar_frame", "evidence")):
                raise ValueError("Body/LiDAR frame and measurement evidence required")
            transform = CameraLidarCalibration._array(profile["T_body_from_lidar"], (4, 4))
            rotation = transform[:3, :3]
            if (not np.allclose(transform[3], [0,0,0,1], atol=1e-8, rtol=0) or
                    not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5, rtol=0) or
                    not np.isclose(np.linalg.det(rotation), 1, atol=1e-5, rtol=0)):
                raise ValueError("Body/LiDAR transform must contain a proper rotation")
            for key in ("floor_z_m", "half_length_m", "half_width_m", "clearance_m"):
                value = profile[key]
                if type(value) not in (int, float) or not np.isfinite(value):
                    raise ValueError("Measured finite robot dimensions and ground level required")
            if (not -.8 <= profile["floor_z_m"] <= -.1 or
                    not .1 <= profile["half_length_m"] <= 1 or not .1 <= profile["half_width_m"] <= .7 or
                    not .45 <= profile["clearance_m"] <= 2):
                raise ValueError("Body dimensions, floor or clearance outside supported range")
            self.transform = transform
            self.result = self.unavailable("waiting_for_calibrated_cloud")

    @classmethod
    def load(cls, path):
        if not path:
            return cls()
        source = Path(path)
        if source.stat().st_size > 65536:
            raise ValueError("Oversized obstacle calibration")
        return cls(json.loads(source.read_text(encoding="utf-8")))

    @staticmethod
    def unavailable(reason):
        return {"reason": reason, "forward": False, "reverse": False, "rotate": False,
                "configured": False, "experimental": True}

    def ingest(self, xyz, lidar, state):
        if self.profile is None:
            return
        with self.lock:
            sequence = lidar.get("cloud_sequence")
            if type(sequence) is not int or sequence <= 0 or (self.sequence is not None and sequence < self.sequence):
                self.result = self.unavailable("invalid_cloud_sequence")
                return
            if sequence == self.sequence:
                return  # Re-reading old bytes must never refresh the watchdog.
            self.result = self.unavailable("invalid_or_stale_lidar")
            if (lidar.get("frame") != self.profile["lidar_frame"] or lidar.get("error") != 0 or
                    not 0 <= lidar.get("age_s", -1) <= .3 or state.get("error") != "0" or
                    not 0 <= state.get("age_s", -1) <= .3):
                return
            xyz = np.asarray(xyz)
            if xyz.ndim != 2 or xyz.shape[1] != 3 or not 100 <= len(xyz) <= 1_000_000:
                return
            finite = np.isfinite(xyz).all(axis=1)
            if finite.sum() < 100 or finite.mean() < .99:
                return
            p = xyz[finite] @ self.transform[:3, :3].T + self.transform[:3, 3]
            # Measured floor offset required. Low obstacles are retained above
            # 5 cm; this does not establish detection of cables, glass or dropoffs.
            height = p[:, 2]-self.profile["floor_z_m"]
            p = p[(height >= .05) & (height <= 1.5)]
            if len(p) < 100:
                return
            x, y = p[:, 0], p[:, 1]
            length, width, clearance = (self.profile[k] for k in ("half_length_m", "half_width_m", "clearance_m"))
            outside_body = (np.abs(x) > length) | (np.abs(y) > width)
            p = p[outside_body]
            x, y = p[:, 0], p[:, 1]
            front = (x >= length) & (x <= length+clearance) & (np.abs(y) <= width+.1)
            rear = (x <= -length) & (x >= -length-clearance) & (np.abs(y) <= width+.1)
            radius = np.hypot(length, width)+clearance
            turning = np.hypot(x,y) <= radius
            # Require farther visible returns in every coarse direction sector.
            # Empty cells/clouds never count as clear floor. Conservative refusal
            # is intentional in open space and where the sensor has blind spots.
            angles = np.arctan2(y,x)
            far = np.hypot(x,y) > radius
            counts, _ = np.histogram(angles[far], bins=24, range=(-np.pi, np.pi))
            forward_coverage = bool(np.all(counts[8:16] >= 3))
            reverse_coverage = bool(np.all(np.r_[counts[:4],counts[-4:]] >= 3))
            rotation_coverage = bool(np.all(counts >= 3))
            result = {"configured": True, "experimental": True,
                      "forward": forward_coverage and not bool(front.any()),
                      "reverse": reverse_coverage and not bool(rear.any()),
                      "rotate": rotation_coverage and not bool(turning.any()),
                      "obstacle_points": {"forward": int(front.sum()), "reverse": int(rear.sum()), "rotate": int(turning.sum())},
                      "coverage": {"forward": forward_coverage, "reverse": reverse_coverage, "rotate": rotation_coverage},
                      "cloud_sequence": lidar.get("cloud_sequence"), "frame": lidar["frame"],
                      "reason": "evaluated", "clearance_m": clearance}
            self.result = result
            self.sequence = lidar.get("cloud_sequence")
            self.last = self.clock()-lidar["age_s"]

    def status(self):
        with self.lock:
            result = copy.deepcopy(self.result)
            age = self.clock()-self.last
        result["age_s"] = age if np.isfinite(age) else None
        if age > .3 and result.get("configured"):
            result.update(forward=False, reverse=False, rotate=False, reason="stale_lidar")
        return result


def movement_permission(status, mode, amount):
    if mode not in {"walk", "rotate"} or type(amount) not in (int, float) or not np.isfinite(amount) or amount == 0:
        return {"allowed": False, "reason": "invalid_movement_request"}
    direction = "rotate" if mode == "rotate" else "forward" if amount > 0 else "reverse"
    allowed = status.get("configured") is True and status.get(direction) is True
    age = status.get("age_s")
    allowed = allowed and type(age) in (int, float) and np.isfinite(age) and 0 <= age <= .3
    reason = status.get("reason", "obstacle_monitor_unavailable")
    if not allowed and reason == "evaluated":
        reason = "obstacle_or_unobserved_direction_" + direction
    return {"allowed": bool(allowed), "direction": direction, "reason": reason}
