"""Arespath Rover — main Flask application v7.

V7 Navigation Stack (ROS-inspired):
─────────────────────────────────────
• Sensor Processing: LiDAR + radar fusion
• Localization: ICP-corrected odometry
• Global Planner: Jump Point Search (JPS) with dynamic obstacles
• Local Planner: Cubic spline smoothing + velocity profiling
• Controller: Pure pursuit + PID heading control
• Motor Control: Smooth velocity with acceleration limiting

Routes
──────
  /                       → dashboard SPA
  /static/*               → CSS, JS
  /api/status             → GET robot status
  /api/arm                → POST {armed}
  /api/manual             → POST {linear, angular}
  /api/stop               → POST
  /api/mode               → POST {mode}
  /api/map/*              → mapping
  /api/pose               → POST {x,y,theta}
  /api/navigate/goal      → POST {x,y}
  /api/navigate/cancel    → POST
  /api/lidar              → GET lidar data + avoidance route
  /api/map/data           → GET map payload
  /api/map/list           → GET saved map names
  /api/settings           → GET all tunables | PATCH {key,value} | DELETE (reset)
  /api/cameras            → GET snapshot status + file lists
  /api/snapshot/start     → POST — begin ring-buffer capture
  /api/snapshot/stop      → POST — end ring-buffer capture
  /api/snapshot/files     → GET list of ring-buffer snapshots
  /api/photo/take         → POST — capture permanent photo (both cameras)
  /api/photo/files        → GET list of permanent photos
  /snapshots/<filename>   → serve + download snapshot
  /photos/<filename>      → serve + download permanent photo
  /video/<cam>            → MJPEG stream
  /healthz                → liveness
"""

import logging
import sys
import time
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from flask import Flask, Response, jsonify, request, send_from_directory
from flask_socketio import SocketIO, emit

from app import config
from app.robot.camera import CameraManager
# ── Runtime settings must be initialised first — control.py → lidar.py
#    import rtcfg at module level; init() must run before those imports. ──────
from app.robot import runtime_cfg as _runtime_cfg_mod
_runtime_cfg_mod.init(config.RUNTIME_SETTINGS_FILE)
from app.robot.runtime_cfg import rtcfg          # now safe: singleton is live
from app.robot.control import RobotRuntime

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
log = logging.getLogger(__name__)

app = Flask(__name__, static_folder="static", static_url_path="/static")
app.config["SECRET_KEY"] = "arespath-rover-secret"

socketio = SocketIO(
    app,
    async_mode="threading",
    cors_allowed_origins="*",
    logger=False,
    engineio_logger=False,
)

runtime = RobotRuntime()
cameras = CameraManager()
runtime.set_socketio(socketio)

_SPEED_MAP = {
    "forward": (1.0,  0.0),
    "back":    (-1.0, 0.0),
    "left":    (0.0,  1.0),   # positive angular = turn left
    "right":   (0.0, -1.0),   # negative angular = turn right
    "stop":    (0.0,  0.0),
}


_startup_lock = __import__('threading').Lock()

@app.before_request
def _startup_once():
    if not getattr(app, "_started", False):
        with _startup_lock:
            if not getattr(app, "_started", False):
                cameras.start()
                runtime.start()
                app._started = True


# ── Static / SPA ──────────────────────────────────────────────────────────────

@app.route("/")
def index():
    return send_from_directory(app.static_folder, "index.html")


# ── Status ────────────────────────────────────────────────────────────────────

@app.route("/api/status")
def api_status():
    return jsonify(runtime.get_status())


@app.route("/healthz")
def healthz():
    return jsonify({"ok": True, "ts": time.time()})


# ── Arm / Stop ────────────────────────────────────────────────────────────────

@app.route("/api/arm", methods=["POST"])
def api_arm():
    if not request.is_json:
        return jsonify({"ok": False, "error": "JSON body required"}), 400
    runtime.arm(bool(request.json.get("armed", False)))
    return jsonify({"ok": True})


@app.route("/api/stop", methods=["POST"])
def api_stop():
    runtime.force_stop()
    return jsonify({"ok": True})


# ── Manual drive ──────────────────────────────────────────────────────────────

@app.route("/api/manual", methods=["POST"])
def api_manual():
    if not request.is_json:
        return jsonify({"ok": False, "error": "JSON body required"}), 400
    try:
        linear  = float(request.json.get("linear",  0.0))
        angular = float(request.json.get("angular", 0.0))
    except (TypeError, ValueError) as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400
    runtime.set_manual_command(linear, angular)
    return jsonify({"ok": True})


# ── Mode ──────────────────────────────────────────────────────────────────────

@app.route("/api/mode", methods=["POST"])
def api_mode():
    if not request.is_json:
        return jsonify({"ok": False, "error": "JSON body required"}), 400
    mode = request.json.get("mode", "pilot")
    try:
        runtime.set_mode(mode)
    except ValueError as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400
    return jsonify({"ok": True, "mode": mode})


# ── Navigation ────────────────────────────────────────────────────────────────

@app.route("/api/navigate/goal", methods=["POST"])
def api_nav_goal():
    if not request.is_json:
        return jsonify({"ok": False, "error": "JSON body required"}), 400
    try:
        x = float(request.json["x"])
        y = float(request.json["y"])
    except (KeyError, TypeError, ValueError) as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400
    runtime.set_goal(x, y)
    return jsonify({"ok": True})


@app.route("/api/navigate/mission", methods=["POST"])
def api_nav_mission():
    if not request.is_json:
        return jsonify({"ok": False, "error": "JSON body required"}), 400
    raw_points = request.json.get("waypoints")
    if not isinstance(raw_points, list) or not raw_points:
        return jsonify({"ok": False, "error": "waypoints list required"}), 400

    try:
        waypoints = [
            (float(pt["x"]), float(pt["y"]))
            for pt in raw_points
            if isinstance(pt, dict)
        ]
    except (KeyError, TypeError, ValueError) as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400

    if not waypoints:
        return jsonify({"ok": False, "error": "no valid waypoints"}), 400

    runtime.set_mission(waypoints)
    return jsonify({"ok": True, "count": len(waypoints)})


@app.route("/api/navigate/cancel", methods=["POST"])
def api_nav_cancel():
    runtime.cancel_navigation()
    return jsonify({"ok": True})


# ── Runtime settings (tunable parameters) ────────────────────────────────────

@app.route("/api/settings", methods=["GET"])
def api_settings_get():
    """Return all tunables with current values, defaults, min/max, units."""
    return jsonify({"settings": rtcfg.all()})


@app.route("/api/settings", methods=["PATCH"])
def api_settings_patch():
    """Update one tunable parameter.  Body: {key: str, value: float}."""
    if not request.is_json:
        return jsonify({"ok": False, "error": "JSON body required"}), 400
    key = request.json.get("key")
    val = request.json.get("value")
    if key is None or val is None:
        return jsonify({"ok": False, "error": "key and value required"}), 400
    try:
        stored = rtcfg.set(key, float(val))
        return jsonify({"ok": True, "key": key, "value": stored})
    except KeyError as exc:
        return jsonify({"ok": False, "error": str(exc)}), 404
    except (TypeError, ValueError) as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400


@app.route("/api/settings", methods=["DELETE"])
def api_settings_reset():
    """Reset all tunable parameters to config.py defaults."""
    rtcfg.reset_to_defaults()
    return jsonify({"ok": True, "settings": rtcfg.all()})


# ── POI / Waypoint routes ─────────────────────────────────────────────────────

@app.route("/api/poi", methods=["GET"])
def api_poi_list():
    return jsonify({"pois": runtime.poi_list()})


@app.route("/api/poi", methods=["POST"])
def api_poi_add():
    if not request.is_json:
        return jsonify({"ok": False, "error": "JSON body required"}), 400
    data = request.json
    try:
        label = str(data.get("label", "Waypoint"))
        kind  = str(data.get("kind",  "waypoint"))
        x     = float(data["x"])
        y     = float(data["y"])
        note  = str(data.get("note", ""))
    except (KeyError, TypeError, ValueError) as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400
    poi = runtime.poi_add(label, kind, x, y, note)
    return jsonify({"ok": True, "poi": poi})


@app.route("/api/poi/<poi_id>", methods=["PATCH"])
def api_poi_update(poi_id: str):
    if not request.is_json:
        return jsonify({"ok": False, "error": "JSON body required"}), 400
    updated = runtime.poi_update(poi_id, **request.json)
    if updated is None:
        return jsonify({"ok": False, "error": "POI not found"}), 404
    return jsonify({"ok": True, "poi": updated})


@app.route("/api/poi/<poi_id>", methods=["DELETE"])
def api_poi_delete(poi_id: str):
    if runtime.poi_remove(poi_id):
        return jsonify({"ok": True})
    return jsonify({"ok": False, "error": "POI not found"}), 404


@app.route("/api/poi/<poi_id>/navigate", methods=["POST"])
def api_poi_navigate(poi_id: str):
    if runtime.poi_navigate(poi_id):
        return jsonify({"ok": True})
    return jsonify({"ok": False, "error": "POI not found"}), 404


@app.route("/api/pose", methods=["POST"])
def api_pose():
    if not request.is_json:
        return jsonify({"ok": False, "error": "JSON body required"}), 400
    try:
        x     = float(request.json.get("x",     0.0))
        y     = float(request.json.get("y",     0.0))
        theta = float(request.json.get("theta", 0.0))
    except (TypeError, ValueError) as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400
    runtime.set_pose(x, y, theta)
    return jsonify({"ok": True})


# ── Map ───────────────────────────────────────────────────────────────────────

@app.route("/api/map/start", methods=["POST"])
def api_map_start():
    clear = False
    if request.is_json and request.json:
        clear = bool(request.json.get("clear", False))
    runtime.start_mapping(clear=clear)
    return jsonify({"ok": True})


@app.route("/api/map/stop", methods=["POST"])
def api_map_stop():
    runtime.stop_mapping()
    return jsonify({"ok": True})


@app.route("/api/map/reset", methods=["POST"])
def api_map_reset():
    runtime.clear_map()
    return jsonify({"ok": True})


@app.route("/api/map/save", methods=["POST"])
def api_map_save():
    name = f"floor_{int(time.time())}"
    if request.is_json and request.json:
        name = request.json.get("name", name)
    result = runtime.save_map(name)
    return jsonify({"ok": True, "files": result})


@app.route("/api/map/load", methods=["POST"])
def api_map_load():
    if not request.is_json or not request.json:
        return jsonify({"ok": False, "error": "JSON body required"}), 400
    name = request.json.get("name")
    if not name:
        return jsonify({"ok": False, "error": "name is required"}), 400
    try:
        runtime.load_map(name)
    except FileNotFoundError as exc:
        return jsonify({"ok": False, "error": str(exc)}), 404
    return jsonify({"ok": True})


@app.route("/api/map/list")
def api_map_list():
    names = sorted({p.stem for p in config.MAPS_DIR.glob("*.yaml")})
    return jsonify({"maps": names})


@app.route("/api/map/data")
def api_map_data():
    return jsonify(runtime.get_map_payload())


# ── Lidar ─────────────────────────────────────────────────────────────────────

@app.route("/api/lidar")
def api_lidar():
    return jsonify(runtime.get_lidar_payload())


# ── Cameras ───────────────────────────────────────────────────────────────────

@app.route("/video/<cam_name>")
def video_feed(cam_name):
    if cam_name not in cameras.cams:
        return "Camera not found", 404
    return Response(
        cameras.generate_mjpeg(cam_name),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/api/cameras")
def api_cameras():
    return jsonify({
        "snapping": cameras.snapping,
        "counts":   cameras.snapshot_count(),
        "files":    {name: cameras.snapshot_files(name) for name in cameras.cams},
    })


# ── Ring-buffer snapshots ─────────────────────────────────────────────────────

@app.route("/api/snapshot/start", methods=["POST"])
def api_snapshot_start():
    cameras.start_snapping()
    return jsonify({"ok": True, "snapping": True})


@app.route("/api/snapshot/stop", methods=["POST"])
def api_snapshot_stop():
    cameras.stop_snapping()
    return jsonify({"ok": True, "snapping": False})


@app.route("/api/snapshot/files")
def api_snapshot_files():
    files = {}
    for cam_name in cameras.cams:
        files[cam_name] = cameras.snapshot_files(cam_name)
    return jsonify({"files": files, "snapping": cameras.snapping})


@app.route("/snapshots/<filename>")
def download_snapshot(filename):
    return send_from_directory(
        config.SNAPSHOT_DIR, filename,
        as_attachment=request.args.get("dl") == "1"
    )


# ── Permanent photos ──────────────────────────────────────────────────────────

@app.route("/api/photo/take", methods=["POST"])
def api_photo_take():
    saved = cameras.take_photo()
    if not saved:
        return jsonify({"ok": False, "error": "No frames available"}), 503
    return jsonify({"ok": True, "saved": saved})


@app.route("/api/photo/files")
def api_photo_files():
    return jsonify({"files": cameras.photo_files()})


@app.route("/photos/<filename>")
def download_photo(filename):
    return send_from_directory(
        config.PHOTOS_DIR, filename,
        as_attachment=request.args.get("dl") == "1"
    )


# ── Socket.IO events ──────────────────────────────────────────────────────────

@socketio.on("connect")
def on_connect():
    log.info("Socket.IO client connected: %s", request.sid)
    runtime.heartbeat()
    emit("status", runtime.get_status())


@socketio.on("disconnect")
def on_disconnect():
    log.info("Socket.IO client disconnected: %s", request.sid)
    runtime.force_stop()


@socketio.on("heartbeat")
def on_heartbeat(_data=None):
    runtime.heartbeat()
    emit("heartbeat_ack", {"ts": time.time()})


@socketio.on("manual_command")
def on_manual_command(data):
    cmd   = str(data.get("cmd", "stop")).lower()
    speed = max(0.0, min(100.0, float(data.get("speed", 55)))) / 100.0

    if cmd == "stop":
        runtime.force_stop()
        emit("ack", {"cmd": cmd, "ok": True})
        return

    lin_base, ang_base = _SPEED_MAP.get(cmd, (0.0, 0.0))
    runtime.set_manual_command(lin_base * speed, ang_base * speed)
    emit("ack", {"cmd": cmd, "ok": True})


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    cameras.start()
    runtime.start()
    app._started = True
    socketio.run(
        app,
        host="0.0.0.0",
        port=config.WEB_PORT,
        use_reloader=False,
        allow_unsafe_werkzeug=True,
    )
