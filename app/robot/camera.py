"""Camera manager — MJPEG streaming, ring-buffer snapshots, permanent photo capture.

Two storage systems
───────────────────
snapshots/   Ring-buffer: auto-captured every SNAPSHOT_SECONDS while snapping=True.
             Max MAX_SNAPSHOTS_PER_CAM (10) per camera. Oldest deleted when full.

photos/      Permanent: saved on demand via take_photo(). Never auto-deleted.
             Filenames: photo_<cam>_<timestamp>.jpg
"""

import logging
import os
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from app import config

log = logging.getLogger(__name__)

MAX_SNAPSHOTS_PER_CAM = 10

try:
    import cv2
    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False
    log.warning("OpenCV not available — camera streams will use placeholder frames")

try:
    import numpy as np
    _NP_AVAILABLE = True
except ImportError:
    _NP_AVAILABLE = False


def _make_placeholder_jpeg(label: str) -> Optional[bytes]:
    if not (_CV2_AVAILABLE and _NP_AVAILABLE):
        return None
    frame = np.zeros((config.CAMERA_HEIGHT, config.CAMERA_WIDTH, 3), dtype=np.uint8)
    frame[:] = (30, 30, 40)
    cv2.putText(frame, f"[{label}] no signal", (20, config.CAMERA_HEIGHT // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (80, 180, 80), 2)
    ok, buf = cv2.imencode(".jpg", frame,
                           [int(cv2.IMWRITE_JPEG_QUALITY), config.JPEG_QUALITY])
    return buf.tobytes() if ok else None


class CameraWorker:
    def __init__(self, name: str, device: str) -> None:
        self.name = name
        self.device = device
        self._cap = None
        self._frame: Optional[bytes] = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._placeholder = _make_placeholder_jpeg(name)

    def start(self) -> bool:
        if not _CV2_AVAILABLE:
            log.warning("[camera:%s] OpenCV not available", self.name)
            return False
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if cap is None or not cap.isOpened():
            log.warning("[camera:%s] failed to open %s", self.name, self.device)
            return False
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS,          config.CAMERA_FPS)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._cap = cap
        self._running = True
        self._thread = threading.Thread(
            target=self._reader_loop, daemon=True, name=f"cam-{self.name}"
        )
        self._thread.start()
        log.info("[camera:%s] opened on %s", self.name, self.device)
        return True

    def stop(self) -> None:
        self._running = False
        if self._cap:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

    def get_jpeg(self) -> Optional[bytes]:
        with self._lock:
            return self._frame if self._frame is not None else self._placeholder

    def _reader_loop(self) -> None:
        consecutive_failures = 0
        while self._running:
            if not self._cap:
                time.sleep(0.05)
                continue
            ok, bgr = self._cap.read()
            if ok and bgr is not None:
                enc_ok, buf = cv2.imencode(
                    ".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), config.JPEG_QUALITY]
                )
                if enc_ok:
                    with self._lock:
                        self._frame = buf.tobytes()
                consecutive_failures = 0
            else:
                consecutive_failures += 1
                time.sleep(min(0.5, 0.05 * consecutive_failures))


class CameraManager:
    def __init__(self) -> None:
        self.cams: Dict[str, CameraWorker] = {}
        self._snap_dir  = Path(config.SNAPSHOT_DIR)
        self._photo_dir = Path(config.PHOTOS_DIR)
        self._snap_dir.mkdir(exist_ok=True, parents=True)
        self._photo_dir.mkdir(exist_ok=True, parents=True)
        self._running = False
        self._snapping = False
        self._snap_lock = threading.Lock()
        self._threads: list = []

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start(self) -> None:
        if self._running:
            return
        for name, device in config.CAMERA_DEVICES.items():
            worker = CameraWorker(name, device)
            worker.start()
            self.cams[name] = worker
        self._running = True
        t = threading.Thread(target=self._snapshot_loop, daemon=True, name="snap-loop")
        t.start()
        self._threads.append(t)

    def stop(self) -> None:
        self._running = False
        for w in self.cams.values():
            w.stop()
        self.cams.clear()

    # ── Ring-buffer snapshot control ──────────────────────────────────────────

    def start_snapping(self) -> None:
        with self._snap_lock:
            self._snapping = True
        log.info("Snapshot capture STARTED")

    def stop_snapping(self) -> None:
        with self._snap_lock:
            self._snapping = False
        log.info("Snapshot capture STOPPED")

    @property
    def snapping(self) -> bool:
        with self._snap_lock:
            return self._snapping

    # ── Permanent photo capture ───────────────────────────────────────────────

    def take_photo(self) -> Dict[str, str]:
        """Capture one frame from every camera and save to photos/.
        Returns {cam_name: filename} for successfully saved photos.
        Skips cameras that only have a placeholder (no real frame yet)."""
        results: Dict[str, str] = {}
        ts = int(time.time())
        for cam_name, worker in self.cams.items():
            # Only save real frames — skip placeholder-only cameras
            with worker._lock:
                jpeg = worker._frame   # None = no real frame captured yet
            if not jpeg:
                continue
            fname = f"photo_{cam_name}_{ts}.jpg"
            path  = self._photo_dir / fname
            try:
                path.write_bytes(jpeg)
                results[cam_name] = fname
                log.info("[camera:%s] permanent photo: %s", cam_name, fname)
            except OSError as exc:
                log.warning("[camera:%s] photo write failed: %s", cam_name, exc)
        return results

    # ── MJPEG streaming ───────────────────────────────────────────────────────

    def generate_mjpeg(self, cam_name: str):
        worker = self.cams.get(cam_name)
        if worker is None:
            raise KeyError(cam_name)
        while self._running:
            jpeg = worker.get_jpeg()
            if jpeg is None:
                time.sleep(0.05)
                continue
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
            )
            time.sleep(1.0 / config.CAMERA_FPS)

    # ── File listings ─────────────────────────────────────────────────────────

    def snapshot_files(self, cam_name: str) -> List[str]:
        """Snapshot filenames for a camera, newest first."""
        snaps = self._sorted_snaps_in(self._snap_dir, cam_name + "_")
        snaps.reverse()
        return snaps

    def snapshot_count(self) -> Dict[str, int]:
        return {name: len(self._sorted_snaps_in(self._snap_dir, name + "_"))
                for name in self.cams}

    def photo_files(self) -> List[Dict]:
        """All permanent photos, newest first, with metadata."""
        files = []
        try:
            for f in os.listdir(self._photo_dir):
                if not f.endswith(".jpg"):
                    continue
                p = self._photo_dir / f
                try:
                    stat = p.stat()
                    # Parse cam name from photo_<cam>_<ts>.jpg
                    parts = f.split("_")
                    cam = parts[1] if len(parts) >= 3 else "unknown"
                    ts  = int(parts[-1].split(".")[0]) if parts else int(stat.st_mtime)
                    files.append({
                        "filename": f,
                        "cam":      cam,
                        "ts":       ts,
                        "size":     stat.st_size,
                    })
                except (ValueError, IndexError, OSError):
                    continue
        except OSError:
            pass
        files.sort(key=lambda x: x["ts"], reverse=True)
        return files

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _sorted_snaps_in(self, directory: Path, prefix: str) -> List[str]:
        """Filenames in directory starting with prefix, sorted oldest→newest."""
        files: List[Tuple[int, str]] = []
        try:
            for f in os.listdir(directory):
                if f.startswith(prefix) and f.endswith(".jpg"):
                    try:
                        ts = int(f.split("_")[-1].split(".")[0])
                        files.append((ts, f))
                    except (ValueError, IndexError):
                        continue
        except OSError:
            pass
        files.sort()
        return [f for _, f in files]

    def _enforce_ring_buffer(self, cam_name: str) -> None:
        snaps = self._sorted_snaps_in(self._snap_dir, cam_name + "_")
        while len(snaps) >= MAX_SNAPSHOTS_PER_CAM:
            oldest = self._snap_dir / snaps.pop(0)
            try:
                oldest.unlink()
                log.debug("[camera:%s] ring-buffer evicted %s", cam_name, oldest.name)
            except OSError:
                pass

    def _take_snapshot(self, cam_name: str) -> Optional[Path]:
        worker = self.cams.get(cam_name)
        if worker is None:
            return None
        jpeg = worker.get_jpeg()
        if not jpeg:
            return None
        self._enforce_ring_buffer(cam_name)
        ts   = int(time.time())
        path = self._snap_dir / f"{cam_name}_{ts}.jpg"
        try:
            path.write_bytes(jpeg)
            log.info("[camera:%s] snapshot: %s", cam_name, path.name)
            return path
        except OSError as exc:
            log.warning("[camera:%s] snapshot write failed: %s", cam_name, exc)
            return None

    def _snapshot_loop(self) -> None:
        while self._running:
            time.sleep(config.SNAPSHOT_SECONDS)
            if not self.snapping:
                continue
            for cam_name in list(self.cams.keys()):
                self._take_snapshot(cam_name)
