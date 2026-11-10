"""On-site camera intrinsics collection from the existing read-only sensor hub.

The robot stays still; the operator presents a printed 9x6-corner, 25 mm target.
This produces camera intrinsics only, never fabricated LiDAR/body extrinsics.
"""
import argparse
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
import time
import uuid

import numpy as np

from .shared_sensor_adapter import HubClient
from .camera_lidar import capture_receipt_bundle, CameraLidarCalibration

PATTERN = (9, 6)
SQUARE_M = .025


def detect_board(jpeg):
    import cv2
    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if image is None or image.size > 25_000_000*3:
        raise ValueError("Invalid camera image")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    scale = min(1., 960 / image.shape[1])
    small = cv2.resize(gray, None, fx=scale, fy=scale)
    # The exhaustive SB detector can spend tens of seconds on a textured office
    # image with no board. Fast rejection keeps the live calibration view fresh.
    found, corners = cv2.findChessboardCorners(small, PATTERN,
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
    if found:
        corners = (corners/scale).astype(np.float32)
        corners = cv2.cornerSubPix(gray, corners, (5,5), (-1,-1),
                                   (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, .01))
        cv2.drawChessboardCorners(image, PATTERN, corners, True)
    preview = cv2.resize(image, (960, round(image.shape[0]*960/image.shape[1])))
    ok, encoded = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 88])
    if not ok:
        raise ValueError("Preview encoding failed")
    return corners if found else None, (image.shape[1], image.shape[0]), encoded.tobytes()


def coverage_issues(samples, size):
    if not samples or size is None:
        return ["Покажите мишень в разных частях изображения"]
    centers = np.array([np.asarray(p).reshape(-1,2).mean(axis=0)/size for p in samples])
    issues = []
    if centers[:,0].min() > .35:
        issues.append("Добавьте ракурсы слева")
    if centers[:,0].max() < .65:
        issues.append("Добавьте ракурсы справа")
    if centers[:,1].min() > .4:
        issues.append("Добавьте ракурсы в верхней части изображения")
    if centers[:,1].max() < .6:
        issues.append("Добавьте ракурсы в нижней части изображения")
    return issues


def board_pose_diversity(samples, intrinsics, square_m):
    """Diagnose observability of the plane solver using its exact holdout split."""
    if len(samples) < 8:
        return {"ready": False, "hint": "Соберите не менее 8 пар с разными наклонами плоской стороны щита"}
    from .lidar_camera_extrinsics import camera_plane
    try:
        normals = np.asarray([camera_plane(p,intrinsics,square_m)[0] for p in samples])
        _, singular, vt = np.linalg.svd(normals[:-2],full_matrices=False)
        condition = float(singular[0]/singular[-1]) if singular[-1] > 1e-9 else None
        ready = condition is not None and condition <= 30
        if ready:
            hint = "Разнообразие наклонов достаточное; соответствие мишени в LiDAR ещё нужно проверить"
        elif abs(vt[-1,1]) >= .7:
            hint = "Не хватает наклона вперёд/назад: добавьте пары с верхним краем щита к роботу и от робота"
        else:
            hint = "Добавьте повороты щита влево/вправо и наклоны верхнего края к роботу и от робота"
        return {"ready":ready,"training_normal_condition":condition,"hint":hint,
                "training_views":len(samples)-2,"holdout_views":2,"navigation_ready":False}
    except Exception as exc:
        return {"ready":False,"hint":"Не удалось проверить наклоны сохранённых ракурсов", "error":str(exc)}


def solve_intrinsics(samples, size, square_m=SQUARE_M):
    import cv2
    if type(square_m) not in (int, float) or not np.isfinite(square_m) or not .005 <= square_m <= .1:
        raise ValueError("Размер клетки должен быть от 5 до 100 мм")
    if len(samples) < 12:
        raise ValueError("Нужно минимум 12 разных ракурсов мишени; желательно 15-20")
    width, height = size
    points = [np.asarray(s, dtype=np.float32).reshape(-1,1,2) for s in samples]
    if any(p.shape != (54,1,2) or not np.isfinite(p).all() for p in points):
        raise ValueError("Invalid calibration corner observations")
    issues = coverage_issues(points, size)
    if issues:
        raise ValueError("; ".join(issues))
    centers = np.array([p.mean(axis=0)[0]/[width,height] for p in points])
    if np.ptp(centers[:,0]) < .25 or np.ptp(centers[:,1]) < .2:
        raise ValueError("Добавьте ракурсы по краям изображения: слева/справа и сверху/снизу")
    perspective = []
    for corners in points:
        grid = corners.reshape(6,9,2)
        left, right = np.linalg.norm(grid[-1,0]-grid[0,0]), np.linalg.norm(grid[-1,-1]-grid[0,-1])
        top, bottom = np.linalg.norm(grid[0,-1]-grid[0,0]), np.linalg.norm(grid[-1,-1]-grid[-1,0])
        if min(left, right, top, bottom) <= 1:
            raise ValueError("Мишень слишком мала на изображении")
        perspective.append([np.log(left/right), np.log(top/bottom)])
    if np.max(np.ptp(perspective, axis=0)) < .1:
        raise ValueError("Нужны разные наклоны мишени, а не только её перемещение по изображению")
    obj = np.zeros((54,3), np.float32)
    obj[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)*square_m
    holdout = {len(points)-2, len(points)-1}
    train = [p for i,p in enumerate(points) if i not in holdout]
    rms, K, distortion, rvecs, tvecs = cv2.calibrateCamera([obj]*len(train), train, size, None, None)
    errors = []
    holdout_errors = []
    for i, image_points in enumerate(points):
        ok, rvec, tvec = cv2.solvePnP(obj, image_points, K, distortion)
        if not ok:
            raise ValueError("Camera pose fit failed")
        projected,_ = cv2.projectPoints(obj, rvec, tvec, K, distortion)
        error = float(np.sqrt(np.mean(np.sum((projected-image_points)**2, axis=2))))
        errors.append(error)
        if i in holdout:
            holdout_errors.append(error)
    quality = (np.isfinite(K).all() and np.isfinite(distortion).all() and rms < 1.5 and
               max(holdout_errors) < 2 and .1*width < K[0,0] < 5*width and
               .1*height < K[1,1] < 5*height and 0 < K[0,2] < width and 0 < K[1,2] < height)
    if not quality:
        raise ValueError(f"Калибровка не прошла проверку: RMS={rms:.2f}px, контроль={max(holdout_errors):.2f}px")
    # Low reprojection error alone can hide an unstable focal length, particularly
    # when a small target covers only one part of the image.
    folds = []
    for fold in range(4):
        subset = [p for i,p in enumerate(points) if i % 4 != fold]
        _, fitted, _, _, _ = cv2.calibrateCamera([obj]*len(subset), subset, size, None, None)
        folds.append([float(fitted[0,0]), float(fitted[1,1])])
    focal = np.asarray(folds)
    relative_range = np.ptp(focal, axis=0)/np.mean(focal, axis=0)
    if not np.isfinite(focal).all() or np.any(focal <= 0) or np.max(relative_range) > .05:
        raise ValueError(f"Неустойчивый результат: изменение фокуса {100*np.max(relative_range):.1f}%. Добавьте крупные ракурсы с разными наклонами по всему изображению")
    return {"schema": 1, "model": "opencv_brown_pinhole", "image_size": list(size),
            "K": K.tolist(), "distortion": distortion.reshape(-1)[:5].tolist(),
            "training_rms_px": float(rms), "per_view_rms_px": errors,
            "holdout_rms_px": holdout_errors, "views": len(points),
            "pattern_inner_corners": list(PATTERN), "square_m": square_m,
            "focal_stability_relative_range": relative_range.tolist(),
            "quality_checks_passed": True, "calibration_verified": False,
            "lidar_extrinsics_available": False, "navigation_ready": False}


class CalibrationSession:
    def __init__(self, hub, directory, square_m=SQUARE_M, resume=False, intrinsics_path=None):
        self.hub = hub
        self.directory = Path(directory)
        if resume and not self.directory.is_dir():
            raise ValueError("Calibration session does not exist")
        self.directory.mkdir(parents=True, exist_ok=resume)
        if type(square_m) not in (int,float) or not np.isfinite(square_m) or not .005 <= square_m <= .1:
            raise ValueError("Invalid measured square size")
        self.square_m = square_m
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.thread = None
        self.frame = None
        self.samples = []
        self.size = None
        self.error = None
        self.result = None
        self.paired_views = 0
        self.hub_instance = None
        self.reference_intrinsics = None
        self._pose_quality = None
        self._pose_quality_count = -1
        self.target_dimensions_cm = None
        measurement_path = self.directory/"target-measurements.json"
        if measurement_path.is_file():
            measurement = json.loads(measurement_path.read_text(encoding="utf-8"))
            dimensions = [measurement.get("width_m"),measurement.get("height_m")]
            if all(type(v) in (int,float) and np.isfinite(v) and v > 0 for v in dimensions):
                self.target_dimensions_cm = [round(v*100,2) for v in dimensions]
        if resume:
            self._restore()
        if intrinsics_path:
            reference = Path(intrinsics_path).resolve()
            if reference.stat().st_size > 65536:
                raise ValueError("Oversized camera calibration")
            profile = json.loads(reference.read_text(encoding="utf-8"))
            stability = np.asarray(profile.get("focal_stability_relative_range",[]),dtype=float)
            if (profile.get("quality_checks_passed") is not True or stability.shape != (2,) or
                    not np.isfinite(stability).all() or np.any(stability < 0) or np.max(stability) > .05):
                raise ValueError("Reference intrinsics have not passed stability checks")
            K = CameraLidarCalibration._array(profile.get("K"),(3,3))
            CameraLidarCalibration._array(profile.get("distortion"),(5,))
            if K[0,0] <= 0 or K[1,1] <= 0 or not np.allclose(K[2],[0,0,1]):
                raise ValueError("Invalid camera matrix")
            self.reference_intrinsics = profile
            self.result = profile
            (self.directory/"camera-intrinsics.json").write_text(json.dumps(profile,indent=2,allow_nan=False),encoding="utf-8")
        (self.directory/"session-settings.json").write_text(json.dumps({
            "schema": 1, "pattern_inner_corners": list(PATTERN),
            "square_m": self.square_m, "source": "operator_supplied_session_setting",
            "stage": "camera_lidar" if self.reference_intrinsics else "intrinsics",
            "intrinsics_source": str(intrinsics_path) if intrinsics_path else None,
            "navigation_ready": False}, indent=2), encoding="utf-8")

    def _restore(self):
        for index, path in enumerate(sorted(self.directory.glob("view-*.json")), 1):
            if path.stem != f"view-{index:03d}":
                raise ValueError("Non-contiguous calibration views; originals were not modified")
            metadata = json.loads(path.read_text(encoding="utf-8"))
            points = np.asarray(metadata["corners"], dtype=np.float32)
            size = tuple(metadata["image_size"])
            if (points.shape != (54,1,2) or not np.isfinite(points).all() or
                    len(size) != 2 or any(type(v) is not int or v <= 0 for v in size) or
                    not path.with_suffix(".jpg").is_file() or (self.size is not None and size != self.size)):
                raise ValueError("Invalid saved calibration view")
            instance = metadata.get("hub_instance")
            if instance is not None and self.hub_instance is not None and instance != self.hub_instance:
                raise ValueError("Saved views mix sensor hub instances")
            if instance is not None:
                self.hub_instance = instance
            self.samples.append(points)
            self.size = size
            if metadata.get("lidar_pair") is not None and path.with_suffix(".cloud.npz").is_file():
                self.paired_views += 1
        result = self.directory/"camera-intrinsics.json"
        if result.is_file():
            self.result = json.loads(result.read_text(encoding="utf-8"))
            # Historical results predate coverage/stability checks; never present
            # them as validated under the stronger current criteria.
            self.result = {**self.result, "quality_checks_passed": False, "revalidation_required": True}

    def start(self):
        self.thread = threading.Thread(target=self._poll, daemon=True)
        self.thread.start()

    def _poll(self):
        last = None
        while not self.stop.is_set():
            try:
                snapshot = self.hub.request("/snapshot")
                snapshot_time = time.monotonic()
                instance = snapshot.get("instance")
                with self.lock:
                    if self.samples and self.hub_instance is not None and instance != self.hub_instance:
                        raise ValueError("Источник сенсоров перезапущен: начните новую сессию калибровки")
                camera = snapshot["camera"]
                if camera["sequence"] != last and 0 <= camera["age_s"] < 1:
                    received = time.time()-camera["age_s"]
                    jpeg = self.hub.request("/frame?sequence="+str(camera["sequence"]), binary=True)
                    pair, pair_error = None, None
                    try:
                        pair = capture_receipt_bundle(self.hub, snapshot, camera["sequence"], time.monotonic()-snapshot_time)
                    except (ValueError, OSError) as exc:
                        pair_error = str(exc)
                    corners, size, preview = detect_board(jpeg)
                    with self.lock:
                        self.frame = {"id": camera["sequence"], "received": received, "jpeg": jpeg,
                                      "preview": preview, "corners": corners, "size": size,
                                      "pair": pair, "pair_error": pair_error, "hub_instance": instance}
                        self.hub_instance = instance
                        self.error = None
                    last = camera["sequence"]
            except Exception as exc:
                with self.lock:
                    self.error = str(exc)
            self.stop.wait(.2)

    def status(self):
        with self.lock:
            f = self.frame
            age = max(0, time.time()-f["received"]) if f else None
            if self.reference_intrinsics and self._pose_quality_count != len(self.samples):
                self._pose_quality = board_pose_diversity(self.samples,self.reference_intrinsics,self.square_m)
                self._pose_quality_count = len(self.samples)
            return {"motion_enabled": False, "views": len(self.samples), "minimum_views": 8 if self.reference_intrinsics else 12,
                    "pose_quality": self._pose_quality,
                    "target_dimensions_cm": self.target_dimensions_cm,
                    "stage": "camera_lidar" if self.reference_intrinsics else "intrinsics",
                    "paired_views": self.paired_views,
                    "square_mm": self.square_m*1000,
                    "coverage_issues": [] if self.reference_intrinsics else coverage_issues(self.samples, self.size),
                    "lidar_pair_available": bool(f and f.get("pair") is not None and f["pair"].stationary and age < 2),
                    "lidar_pair_error": f.get("pair_error") if f else None,
                    "board_found": f is not None and f["corners"] is not None and age < 2,
                    "camera_age_s": age, "error": self.error, "result": self.result}

    def capture(self):
        with self.lock:
            f = self.frame
            if self.error:
                raise ValueError(self.error)
            if not f or f["corners"] is None or time.time()-f["received"] >= 2:
                raise ValueError("Свежая шахматная мишень не найдена целиком")
            size = f["size"]
            if self.reference_intrinsics and tuple(size) != tuple(self.reference_intrinsics["image_size"]):
                raise ValueError("Разрешение камеры отличается от сохранённой калибровки")
            if self.size is not None and self.size != size:
                raise ValueError("Camera resolution changed; start a new session")
            normalized = f["corners"].reshape(-1,2)/size
            for sample in self.samples:
                old = sample.reshape(-1,2)/size
                difference = min(np.sqrt(np.mean(np.sum((normalized-old)**2,axis=1))),
                                 np.sqrt(np.mean(np.sum((normalized[::-1]-old)**2,axis=1))))
                if difference < .025:
                    raise ValueError("Этот ракурс уже сохранён — переместите или наклоните мишень")
            name = f"view-{len(self.samples)+1:03d}"
            pair = f.get("pair")
            if self.reference_intrinsics and (pair is None or not pair.stationary):
                raise ValueError("Дождитесь подходящего облака LiDAR для текущего кадра; робот должен быть неподвижен")
            if pair is not None and pair.stationary and pair.frame_id != f["id"]:
                raise ValueError("Несовпадение кадра камеры и пары LiDAR")
            (self.directory/(name+".jpg")).write_bytes(f["jpeg"])
            metadata = {"frame_id": f["id"], "received_unix_s": f["received"],
                        "image_size": list(size), "corners": f["corners"].tolist(),
                        "hub_instance": f.get("hub_instance"), "lidar_pair": None,
                        "square_m": self.square_m}
            if pair is not None and pair.stationary:
                # Save the acquisition-time cloud retained before corner detection,
                # never fetch a newer scan after the operator presses Capture.
                np.savez_compressed(self.directory/(name+".cloud.npz"), xyz=pair.xyz)
                metadata["lidar_pair"] = pair.status()
            (self.directory/(name+".json")).write_text(json.dumps(metadata, allow_nan=False), encoding="utf-8")
            self.samples.append(f["corners"].copy())
            if metadata["lidar_pair"] is not None:
                self.paired_views += 1
            self.size = size
        return self.status()

    def solve(self):
        with self.lock:
            if self.reference_intrinsics:
                raise ValueError("Параметры камеры уже рассчитаны. Эта сессия собирает пары для привязки к LiDAR")
            result = solve_intrinsics(self.samples, self.size, self.square_m)
            result["evidence"] = str(self.directory.resolve())
            path = self.directory/"camera-intrinsics.json"
            path.write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")
            self.result = result
        return self.status()


class Handler(BaseHTTPRequestHandler):
    session = None
    def log_message(self, *args):
        pass

    def send(self, code, data, mime="application/json"):
        raw = json.dumps(data, ensure_ascii=False, allow_nan=False).encode() if mime == "application/json" else data
        self.send_response(code); self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(raw))); self.send_header("Cache-Control", "no-store")
        self.end_headers(); self.wfile.write(raw)

    def trusted(self):
        expected = "127.0.0.1:"+str(self.server.server_port)
        return self.headers.get("Host") == expected and self.headers.get("Origin") in (None,"http://"+expected)

    def do_GET(self):
        if not self.trusted():
            self.send(403, {"error": "Origin or host rejected"}); return
        if self.path == "/":
            self.send(200, Path(__file__).with_name("static").joinpath("calibration.html").read_bytes(), "text/html; charset=utf-8")
        elif self.path == "/api/status":
            self.send(200, self.session.status())
        elif self.path.startswith("/camera.jpg"):
            with self.session.lock:
                frame = self.session.frame
                raw = frame["preview"] if frame else None
            self.send(200, raw, "image/jpeg") if raw else self.send(503, {"error": "Waiting for camera"})
        else:
            self.send(404, {"error": "Unknown endpoint"})

    def do_POST(self):
        if not self.trusted():
            self.send(403, {"error": "Origin or host rejected"}); return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 64 or self.headers.get_content_type() != "application/json" or json.loads(self.rfile.read(length)) != {}:
                raise ValueError("Expected empty JSON object")
            if self.path == "/api/capture":
                self.send(200, self.session.capture())
            elif self.path == "/api/solve":
                self.send(200, self.session.solve())
            else:
                self.send(404, {"error": "Unknown endpoint"})
        except (ValueError, TypeError) as exc:
            self.send(400, {"error": str(exc)})


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8773)
    parser.add_argument("--session", help="Resume an existing private calibration session")
    parser.add_argument("--square-mm", type=float, default=25, help="Measured physical chessboard cell size")
    parser.add_argument("--intrinsics", help="Reuse stable camera intrinsics; collect paired views for LiDAR calibration")
    args = parser.parse_args(argv)
    root = Path(__file__).resolve().parents[2]/"data/camera-calibration"
    name = "session-"+datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")+"-"+uuid.uuid4().hex[:6]
    directory = Path(args.session).resolve() if args.session else root/name
    session = CalibrationSession(HubClient("http://127.0.0.1:8780"), directory,
                                 square_m=args.square_mm/1000, resume=bool(args.session),intrinsics_path=args.intrinsics)
    handler = type("CalibrationHandler", (Handler,), {"session": session})
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler)
    server.daemon_threads = True
    try:
        session.start()
        print(f"AS2 camera calibration: http://127.0.0.1:{args.port}/", flush=True)
        server.serve_forever()
    finally:
        session.stop.set()
        session.thread.join(5)
        server.server_close()


if __name__ == "__main__":
    main()
