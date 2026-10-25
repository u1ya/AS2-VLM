"""Stationary, sensor-frame return-density projection, deliberately not SLAM."""
import base64
from collections import deque
import io
import json
import math
from pathlib import Path
import struct
import threading
import time
import uuid

import numpy as np
from PIL import Image


def decode_xyz(raw, metadata):
    """Parse PointCloud2 with NumPy structured dtypes, including padding/endianness."""
    if len(raw) < 8 or len(raw) > 16 * 1024 * 1024 + 8:
        raise ValueError("Invalid cloud size")
    sequence = metadata.get("cloud_sequence")
    if type(sequence) is not int or sequence <= 0:
        raise ValueError("Invalid cloud sequence")
    if struct.unpack_from("<Q", raw)[0] != sequence:
        return None  # Writer replaced the latest cloud before this reader opened it.
    sizes = [metadata.get(k) for k in ("width", "height", "point_step", "row_step")]
    if any(type(v) not in (int, float) or not math.isfinite(v) or v != int(v) or v <= 0 for v in sizes):
        raise ValueError("Invalid cloud dimensions")
    width, height, step, row = map(int, sizes)
    if step > 512 or width * height > 1_000_000 or row < width * step or row * height != len(raw) - 8:
        raise ValueError("Invalid cloud layout")
    fields = metadata.get("fields")
    if not isinstance(fields, list) or len(fields) > 32 or type(metadata.get("bigendian")) is not bool:
        raise ValueError("Invalid cloud field declarations")
    endian = ">" if metadata["bigendian"] else "<"
    formats, offsets = [], []
    for axis in "xyz":
        matching = [f for f in fields if isinstance(f, dict) and f.get("name") == axis]
        if len(matching) != 1:
            raise ValueError("Missing or duplicate XYZ field")
        field = matching[0]
        kind = field.get("datatype")
        offset = field.get("offset")
        if kind not in (7, 8) or field.get("count") != 1 or type(offset) is not int:
            raise ValueError("Unsupported XYZ field")
        size = 4 if kind == 7 else 8
        if offset < 0 or offset + size > step:
            raise ValueError("XYZ field exceeds point step")
        formats.append(endian + ("f4" if kind == 7 else "f8"))
        offsets.append(offset)
    dtype = np.dtype({"names": list("xyz"), "formats": formats, "offsets": offsets, "itemsize": step})
    points = np.ndarray((height, width), dtype=dtype, buffer=raw, offset=8, strides=(row, step))
    xyz = np.column_stack([points[axis].reshape(-1) for axis in "xyz"]).astype(np.float32)
    return xyz[np.isfinite(xyz).all(axis=1)]


class LocalLidarGeometry:
    def __init__(self, extent=16.0, resolution=.1, z_min=-.1, z_max=1.8, clock=time.monotonic):
        self.extent, self.resolution = extent, resolution
        self.z_min, self.z_max = z_min, z_max
        self.clock = clock
        self.size = int(round(2 * extent / resolution))
        if not 1 <= self.size <= 1000 or not z_min < z_max:
            raise ValueError("Invalid projection bounds")
        self._lock = threading.RLock()
        self._history = deque(maxlen=30)
        self._anchor = None
        self._frame = None
        self._last = -math.inf
        self._sequence = 0
        self._version = 0
        self._resets = 0
        self._mode = "waiting"
        self._error = None
        self._cache = None

    def invalidate(self, reason):
        with self._lock:
            self._history.clear()
            self._anchor = None
            self._last = -math.inf
            self._mode, self._error = "unavailable", reason
            self._version += 1
            self._cache = None

    def set_slice(self, z_min, z_max):
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in (z_min, z_max)) or not -10 <= z_min < z_max <= 10:
            raise ValueError("Z slice must satisfy -10 <= min < max <= 10")
        with self._lock:
            self.z_min, self.z_max = z_min, z_max
            self.invalidate("Height slice changed; waiting for a new cloud")

    def ingest(self, xyz, metadata, state):
        now = self.clock()
        with self._lock:
            sequence = metadata["cloud_sequence"]
            if sequence == self._sequence:
                return
            if metadata.get("age_s", -1) < 0 or metadata["age_s"] > .5 or metadata.get("error") != 0:
                self.invalidate("Cloud stale or lidar health unavailable")
                return
            if not 0 <= state.get("age_s", -1) <= .3 or state.get("error") != "0":
                self.invalidate("Fresh valid odometry required for accumulation")
                return
            position, rpy, velocity = (np.asarray(state[k], dtype=float) for k in ("position", "rpy", "velocity"))
            if any(v.shape != (3,) or not np.isfinite(v).all() for v in (position, rpy, velocity)):
                raise ValueError("Invalid geometry pose")
            pose = np.concatenate((position, rpy))
            frame = metadata.get("frame")
            if not isinstance(frame, str) or not frame or len(frame) > 128:
                raise ValueError("Invalid cloud frame")
            moving = np.linalg.norm(velocity) > .03
            changed = self._anchor is not None and (np.linalg.norm(pose[:3] - self._anchor[:3]) > .02 or
                np.max(np.abs(np.arctan2(np.sin(pose[3:] - self._anchor[3:]), np.cos(pose[3:] - self._anchor[3:])))) > .03)
            gap = math.isfinite(self._last) and now - self._last > 1
            if moving or changed or (self._frame and frame != self._frame) or sequence < self._sequence or gap:
                self._history.clear()
                self._anchor = None
                self._resets += 1
            self._anchor = pose if self._anchor is None else self._anchor
            self._frame, self._sequence = frame, sequence
            self._mode = "single_scan_motion" if moving else "stationary_window"
            self._error = None
            if xyz.ndim != 2 or xyz.shape[1] != 3 or xyz.shape[0] > 1_000_000:
                raise ValueError("Invalid point array")
            valid = np.isfinite(xyz).all(axis=1)
            valid &= (xyz[:, 2] >= self.z_min) & (xyz[:, 2] <= self.z_max)
            selected = xyz[valid]
            histogram, _, _ = np.histogram2d(selected[:, 0], selected[:, 1], bins=self.size,
                range=[[-self.extent, self.extent], [-self.extent, self.extent]])
            self._history.append((now, histogram.astype(np.uint32)))
            self._last = now - metadata["age_s"]
            self._version += 1
            self._cache = None

    def _trim(self):
        while self._history and self.clock() - self._history[0][0] > 5:
            self._history.popleft()
            self._version += 1
            self._cache = None

    def status(self):
        with self._lock:
            self._trim()
            age = self.clock() - self._last
            return {"version": self._version, "frame": self._frame, "connected": age <= 1,
                    "age_s": age if math.isfinite(age) else None, "mode": self._mode,
                    "window_scans": len(self._history), "resets": self._resets, "error": self._error,
                    "slam": False, "navigation_ready": False}

    def _render(self):
        self._trim()
        if self._cache is None:
            counts = np.zeros((self.size, self.size), dtype=np.uint32)
            for _, grid in self._history:
                counts += grid
            density = np.flipud(counts.T)
            rgb = np.empty((*density.shape, 3), dtype=np.uint8)
            rgb[:] = [22, 28, 33]
            strength = np.log1p(density.astype(float))
            maximum = strength.max()
            if maximum > 0:
                strength /= maximum
            mask = density > 0
            for axis, (low, high) in enumerate(((25, 235), (145, 245), (170, 220))):
                rgb[:, :, axis][mask] = (low + strength[mask] * (high - low)).astype(np.uint8)
            output = io.BytesIO()
            Image.fromarray(rgb).save(output, format="PNG")
            self._cache = (counts, output.getvalue())
        return self._cache

    def payload(self):
        with self._lock:
            counts, png = self._render()
            return {**self.status(), "bounds": [-self.extent, -self.extent, self.extent, self.extent],
                    "resolution": self.resolution, "width": self.size, "height": self.size,
                    "z_slice": [self.z_min, self.z_max], "units_verified": False,
                    "representation": "return_density_not_free_space", "return_cells": int(np.count_nonzero(counts)),
                    "returns_in_window": int(counts.sum()),
                    "image_png_base64": base64.b64encode(png).decode("ascii")}

    def export(self, directory):
        with self._lock:
            if not self.status()["connected"]:
                raise ValueError("Cannot save a stale or unavailable live projection")
            metadata = self.payload()
            metadata.pop("image_png_base64")
            counts, png = self._render()
            directory = Path(directory)
            directory.mkdir(parents=True, exist_ok=True)
            name = time.strftime("local-lidar-%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:8]
            metadata["saved_at"] = time.time()
            (directory / (name + ".png")).write_bytes(png)
            (directory / (name + ".json")).write_text(json.dumps(metadata, indent=2), encoding="utf-8")
            np.savez_compressed(directory / (name + ".npz"), return_counts=counts)
            return {"name": name, "directory": str(directory.resolve()), "metadata": metadata}
