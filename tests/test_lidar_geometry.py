import base64
import io
import json
from pathlib import Path
import struct
import tempfile
import unittest
import threading
from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
from types import SimpleNamespace

import numpy as np
from PIL import Image

from as2_vlm.agent.lidar_geometry import LocalLidarGeometry, decode_xyz


def cloud(big=False):
    endian = ">" if big else "<"
    dtype = np.dtype({"names": list("xyz"), "formats": [endian+"f4", endian+"f8", endian+"f4"],
                      "offsets": [0, 4, 12], "itemsize": 20})
    raw = bytearray(8 + 2 * 48)
    struct.pack_into("<Q", raw, 0, 7)
    points = np.ndarray((2, 2), dtype=dtype, buffer=raw, offset=8, strides=(48, 20))
    points["x"] = [[1, 2], [3, 4]]
    points["y"] = [[5, 6], [7, 8]]
    points["z"] = [[.1, .2], [.3, .4]]
    meta = {"cloud_sequence": 7, "width": 2, "height": 2, "point_step": 20,
            "row_step": 48, "bigendian": big,
            "fields": [{"name": name, "offset": offset, "datatype": kind, "count": 1}
                       for name, offset, kind in (("x", 0, 7), ("y", 4, 8), ("z", 12, 7))]}
    return raw, meta


class CloudDecodeTests(unittest.TestCase):
    def test_mixed_float_widths_row_padding_and_both_endians(self):
        for big in (False, True):
            raw, metadata = cloud(big)
            xyz = decode_xyz(raw, metadata)
            np.testing.assert_allclose(xyz, [[1, 5, .1], [2, 6, .2], [3, 7, .3], [4, 8, .4]])

    def test_replaced_binary_is_skipped(self):
        raw, metadata = cloud()
        metadata["cloud_sequence"] = 8
        self.assertIsNone(decode_xyz(raw, metadata))

    def test_invalid_layout_is_rejected(self):
        for changes in ({"point_step": 0}, {"row_step": 39}, {"width": 1_000_001},
                        {"fields": []}, {"height": float("nan")}, {"bigendian": 0}):
            raw, metadata = cloud()
            metadata.update(changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                decode_xyz(raw, metadata)

    def test_nonfinite_returns_are_filtered(self):
        raw, metadata = cloud()
        struct.pack_into("<f", raw, 8, float("nan"))
        self.assertEqual(decode_xyz(raw, metadata).shape, (3, 3))

    def test_truncation_and_duplicate_fields_are_rejected(self):
        raw, metadata = cloud()
        with self.assertRaises(ValueError):
            decode_xyz(raw[:-1], metadata)
        metadata["fields"].append(dict(metadata["fields"][0]))
        with self.assertRaises(ValueError):
            decode_xyz(raw, metadata)


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.map = LocalLidarGeometry(clock=lambda: self.now)
        self.meta = {"cloud_sequence": 1, "frame": "hesai_lidar", "age_s": .02, "error": 0}
        self.state = {"age_s": .01, "position": [0, 0, .3], "rpy": [0, 0, 0],
                      "velocity": [0, 0, 0], "error": "0"}
        self.points = np.asarray([[1, 1, .5], [1.01, 1.01, .5], [2, 2, -1], [100, 100, 0]], dtype=np.float32)

    def ingest(self):
        self.map.ingest(self.points, self.meta, self.state)

    def test_real_returns_project_without_claiming_free_space(self):
        self.ingest()
        payload = self.map.payload()
        self.assertEqual(payload["returns_in_window"], 2)
        self.assertGreater(payload["return_cells"], 0)
        self.assertFalse(payload["slam"])
        self.assertFalse(payload["navigation_ready"])
        self.assertFalse(payload["units_verified"])
        with Image.open(io.BytesIO(base64.b64decode(payload["image_png_base64"]))) as image:
            self.assertEqual(image.size, (320, 320))
            self.assertGreater(np.unique(np.asarray(image).reshape(-1, 3), axis=0).shape[0], 1)

    def test_duplicate_cloud_is_not_counted_twice(self):
        self.ingest(); self.ingest()
        self.assertEqual(self.map.payload()["window_scans"], 1)

    def test_accumulation_resets_on_pose_or_frame_change(self):
        self.ingest()
        self.now += .1; self.meta["cloud_sequence"] = 2
        self.ingest()
        self.assertEqual(self.map.payload()["window_scans"], 2)
        self.state["position"][0] = .1; self.meta["cloud_sequence"] = 3
        self.ingest()
        self.assertEqual(self.map.payload()["window_scans"], 1)
        self.meta["frame"] = "other"; self.meta["cloud_sequence"] = 4
        self.ingest()
        self.assertEqual(self.map.payload()["window_scans"], 1)

    def test_moving_sensor_uses_only_one_scan(self):
        self.state["velocity"][0] = .1
        for sequence in range(1, 4):
            self.meta["cloud_sequence"] = sequence; self.ingest()
        self.assertEqual(self.map.payload()["window_scans"], 1)
        self.assertEqual(self.map.payload()["mode"], "single_scan_motion")

    def test_stale_state_invalidates_accumulation(self):
        self.ingest()
        self.meta["cloud_sequence"] = 2
        self.state["age_s"] = 2
        self.ingest()
        self.assertFalse(self.map.payload()["connected"])
        self.assertEqual(self.map.payload()["return_cells"], 0)

    def test_idle_map_expires_and_cannot_be_exported(self):
        self.ingest(); self.now += 6
        self.assertFalse(self.map.payload()["connected"])
        self.assertEqual(self.map.payload()["window_scans"], 0)
        with self.assertRaises(ValueError):
            self.map.export("unused")

    def test_slice_change_resets_and_filters(self):
        self.ingest()
        self.map.set_slice(-2, -.5)
        self.assertEqual(self.map.payload()["return_cells"], 0)
        self.meta["cloud_sequence"] = 2; self.ingest()
        self.assertEqual(self.map.payload()["returns_in_window"], 1)
        for limits in ((1, 0), (float("nan"), 1), (-11, 2)):
            with self.assertRaises(ValueError):
                self.map.set_slice(*limits)

    def test_snapshot_preserves_data_and_limitations(self):
        self.ingest()
        with tempfile.TemporaryDirectory() as root:
            saved = self.map.export(root)
            base = Path(root) / saved["name"]
            metadata = json.loads(base.with_suffix(".json").read_text())
            self.assertFalse(metadata["navigation_ready"])
            self.assertTrue(base.with_suffix(".png").is_file())
            with np.load(base.with_suffix(".npz"), allow_pickle=False) as data:
                self.assertEqual(int(data["return_counts"].sum()), 2)

    def test_geometry_api_filter_and_private_export(self):
        from as2_vlm.agent.app import Handler
        self.ingest()
        geometry = self.map
        with tempfile.TemporaryDirectory() as root:
            class TestHandler(Handler):
                runtime = SimpleNamespace(adapter=SimpleNamespace(local_geometry=geometry),
                    geometry_payload=geometry.payload, config=SimpleNamespace(resolve_path=lambda _: root))
            server = ThreadingHTTPServer(("127.0.0.1", 0), TestHandler)
            worker = threading.Thread(target=server.serve_forever, daemon=True)
            worker.start()
            def request(method, path, body=None):
                connection = HTTPConnection("127.0.0.1", server.server_port, timeout=3)
                try:
                    connection.request(method, path, None if body is None else json.dumps(body),
                                       {"Content-Type": "application/json"})
                    response = connection.getresponse()
                    return response.status, json.loads(response.read())
                finally:
                    connection.close()
            try:
                code, payload = request("GET", "/api/geometry")
                self.assertEqual(code, 200)
                self.assertGreater(payload["return_cells"], 0)
                code, saved = request("POST", "/api/geometry/export", {})
                self.assertEqual(code, 200)
                self.assertTrue(Path(root, saved["name"] + ".npz").is_file())
                self.assertEqual(request("POST", "/api/geometry/filter", {"z_min": 5, "z_max": 2})[0], 400)
                self.assertEqual(request("POST", "/api/geometry/filter", {"z_min": 0, "z_max": 2})[0], 200)
                self.assertEqual(request("POST", "/api/geometry/export", {})[0], 400)
            finally:
                server.shutdown(); server.server_close(); worker.join(timeout=2)
