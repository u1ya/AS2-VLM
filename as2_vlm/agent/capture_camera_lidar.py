"""Save one private, receipt-paired camera/cloud sample from the existing hub."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import uuid

import numpy as np

from .config import load_config
from .shared_sensor_adapter import SharedSensorAdapter


def save_sample(directory, frame, bundle):
    if bundle.frame_id != frame.frame_id:
        raise ValueError("Image/cloud pair frame identity mismatch")
    if (not bundle.stationary or abs(bundle.receipt_skew_s) > .15 or
            not 0 <= bundle.camera_age_s <= .5 or not 0 <= bundle.lidar_age_s <= .5):
        raise ValueError("Stationary fresh receipt pair required")
    name = "pair-" + datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:8]
    root = Path(directory) / name
    root.mkdir(parents=True, exist_ok=False)
    (root / "camera.jpg").write_bytes(frame.jpeg)
    np.savez_compressed(root / "cloud.npz", xyz=bundle.xyz)
    metadata = {"schema": 1, **bundle.status(), "image_size": [frame.width, frame.height],
                "camera_source": frame.source, "camera_receipt_time_unix_s": frame.timestamp,
                "calibration_verified": False, "navigation_ready": False,
                "length_units_verified": False}
    (root / "pair.json").write_text(json.dumps(metadata, indent=2, allow_nan=False), encoding="utf-8")
    return root


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/agent.as2-sensors.json")
    args = parser.parse_args(argv)
    config = load_config(args.config)
    if config.robot.motion_enabled or config.robot.transport != "cpp_readonly":
        parser.error("Read-only shared sensor profile required")
    source = SharedSensorAdapter(config)
    try:
        source.start()
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            frame = source.get_frame()
            if frame is not None:
                pair = source.capture_bundle(frame)
                if pair is not None and pair.stationary and abs(pair.receipt_skew_s) <= .15:
                    print(save_sample(config.resolve_path("data/camera-lidar"), frame, pair))
                    return 0
            time.sleep(.05)
        raise RuntimeError("No fresh stationary receipt pair; check sensor hub, camera and LiDAR")
    finally:
        source.close()


if __name__ == "__main__":
    raise SystemExit(main())
