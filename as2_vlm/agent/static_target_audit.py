"""Read-only diagnostic: save three stable checkerboard frames and full LiDAR fields."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import urllib.error
import uuid

import numpy as np

from .camera_calibration import detect_board
from .camera_lidar import capture_receipt_bundle
from .shared_sensor_adapter import HubClient


def corner_motion(first, current):
    """RMS pixels, allowing the detector's equivalent reversed corner ordering."""
    a, b = (np.asarray(p, dtype=float).reshape(54, 2) for p in (first, current))
    if not np.isfinite(a).all() or not np.isfinite(b).all():
        return float("inf")
    return min(float(np.sqrt(np.mean(np.sum((a-p)**2, axis=1)))) for p in (b, b[::-1]))


class FrozenCloud:
    def __init__(self, path, raw):
        self.path, self.raw = path, raw

    def request(self, path, binary=False):
        if path != self.path or not binary:
            raise ValueError("Unexpected cloud request")
        return self.raw


def collect(hub, directory, timeout=30):
    health = hub.request("/health")
    if health.get("read_only") is not True:
        raise ValueError("Read-only sensor hub required")
    instance = health["instance"]
    deadline = time.monotonic() + timeout
    samples, last_frame = [], None
    counts = {key: 0 for key in ('new_frames', 'pair_unavailable', 'robot_moving',
                                 'board_missing', 'target_moved', 'sequence_race')}
    while time.monotonic() < deadline:
        try:
            started = time.monotonic()
            snapshot = hub.request("/snapshot")
            if snapshot.get("instance") != instance:
                raise RuntimeError("Hub restarted during diagnostic")
            frame_id = snapshot["camera"]["sequence"]
            if frame_id == last_frame:
                time.sleep(.05)
                continue
            last_frame = frame_id
            counts['new_frames'] += 1
            jpeg = hub.request("/frame?sequence=" + str(frame_id), binary=True)
            path = "/cloud?sequence=" + str(snapshot["lidar"]["cloud_sequence"])
            raw = hub.request(path, binary=True)
            pair = capture_receipt_bundle(FrozenCloud(path, raw), snapshot, frame_id,
                                          time.monotonic()-started)
            if pair is None or not pair.stationary:
                counts['pair_unavailable' if pair is None else 'robot_moving'] += 1
                samples = []
                continue
            corners, size, _ = detect_board(jpeg)
            if corners is None:
                counts['board_missing'] += 1
                samples = []
                continue
            now = time.monotonic()
            if samples and (corner_motion(samples[0][4], corners) > 3 or now-samples[-1][0] > 3):
                counts['target_moved'] += 1
                samples = []
            samples.append((now, jpeg, raw, snapshot, corners, size, pair))
            # Limit retained full scans while requiring an observed stationary interval.
            if len(samples) > 3:
                samples.pop(1)
            if len(samples) == 3 and now-samples[0][0] >= 2:
                root = Path(directory) / ("static-" + datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")
                                          + "-" + uuid.uuid4().hex[:8])
                root.mkdir(parents=True, exist_ok=False)
                for i, (_, image, cloud, snap, points, image_size, bundle) in enumerate(samples, 1):
                    prefix = root / f"sample-{i:02d}"
                    prefix.with_suffix(".jpg").write_bytes(image)
                    prefix.with_suffix(".cloud").write_bytes(cloud)
                    prefix.with_suffix(".json").write_text(json.dumps({
                        "snapshot": snap, "pair": bundle.status(), "image_size": image_size,
                        "corners": points.tolist(), "square_m": .023,
                        "target_face_m": [.70, .45], "calibration_verified": False,
                        "navigation_ready": False}, indent=2, allow_nan=False), encoding="utf-8")
                return root
        except urllib.error.HTTPError as exc:
            if exc.code != 409:
                raise
            counts['sequence_race'] += 1
            samples = []
        time.sleep(.05)
    raise TimeoutError("Three fresh stationary checkerboard pairs were not observed: "
                       + json.dumps(counts, sort_keys=True))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=30)
    args = parser.parse_args(argv)
    if not 3 <= args.timeout <= 180:
        parser.error("Timeout must be between 3 and 180 seconds")
    print(collect(HubClient("http://127.0.0.1:8780"), "data/camera-lidar-diagnostics", args.timeout))


if __name__ == "__main__":
    main()
