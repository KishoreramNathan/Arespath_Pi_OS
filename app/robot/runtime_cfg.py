"""Runtime-tunable configuration overlay.

Parameters in TUNABLES can be changed from the web UI at any time and
take effect immediately — no restart needed.  Values are persisted to
``data/runtime_settings.json`` so they survive server restarts.

The static ``config.py`` provides the factory defaults; this module
provides the live, overridable layer on top.

Usage (inside control.py or any module)::

    from app.robot.runtime_cfg import rtcfg

    stop_dist = rtcfg.get("obstacle_stop_distance_m")   # returns float
    rtcfg.set("obstacle_wait_before_replan_s", 3.0)      # apply + persist
    all_vals  = rtcfg.all()                              # dict of all tunables
"""

import json
import logging
import threading
from pathlib import Path
from typing import Any, Dict

from app import config

log = logging.getLogger(__name__)

# ── Tunable parameter definitions ─────────────────────────────────────────────
# Each entry: key → {default, min, max, label, unit, description}
TUNABLES: Dict[str, Dict[str, Any]] = {
    "obstacle_stop_distance_m": {
        "default": config.OBSTACLE_STOP_DISTANCE_M,
        "min":     0.10,
        "max":     2.00,
        "label":   "Stop zone",
        "unit":    "m",
        "description": "Forward distance at which the rover stops for an obstacle",
    },
    "obstacle_wait_before_replan_s": {
        "default": config.OBSTACLE_WAIT_BEFORE_REPLAN_S,
        "min":     1.0,
        "max":    30.0,
        "label":   "Replan wait",
        "unit":    "s",
        "description": "Seconds to wait for obstacle to move before forcing a replan",
    },
    "obstacle_replan_reverse_s": {
        "default": config.OBSTACLE_REPLAN_REVERSE_S,
        "min":     0.2,
        "max":     3.0,
        "label":   "Reverse time",
        "unit":    "s",
        "description": "Duration of the back-up manoeuvre before replanning",
    },
    "nav_replan_seconds": {
        "default": config.NAV_REPLAN_SECONDS,
        "min":     0.5,
        "max":    10.0,
        "label":   "Path replan rate",
        "unit":    "s",
        "description": "How often A* reruns during normal navigation",
    },
    "max_linear_mps": {
        "default": config.MAX_LINEAR_MPS,
        "min":     0.05,
        "max":     1.00,
        "label":   "Max speed",
        "unit":    "m/s",
        "description": "Maximum forward speed of the rover",
    },
    "lidar_icp_weight": {
        "default": config.LIDAR_ICP_WEIGHT,
        "min":     0.0,
        "max":     1.0,
        "label":   "ICP weight",
        "unit":    "",
        "description": "Blend weight for LiDAR ICP correction (0=odometry only, 1=ICP only)",
    },
}


class RuntimeCfg:
    """Thread-safe live-tunable config store."""

    def __init__(self, path: Path) -> None:
        self._path = path
        self._lock = threading.Lock()
        # Start from defaults
        self._vals: Dict[str, float] = {
            k: float(v["default"]) for k, v in TUNABLES.items()
        }
        self._load()

    # ── Public API ────────────────────────────────────────────────────────────

    def get(self, key: str) -> float:
        """Return the current value for *key* (thread-safe)."""
        with self._lock:
            return self._vals[key]

    def set(self, key: str, value: float) -> float:
        """Validate, clamp, store, and persist *value* for *key*.

        Returns the clamped value actually stored.
        Raises ``KeyError`` for unknown keys.
        """
        if key not in TUNABLES:
            raise KeyError(f"Unknown tunable: {key!r}")
        spec = TUNABLES[key]
        clamped = float(max(spec["min"], min(spec["max"], float(value))))
        with self._lock:
            self._vals[key] = clamped
            self._save_locked()
        log.info("RuntimeCfg: %s = %.4g %s", key, clamped, spec.get("unit", ""))
        return clamped

    def all(self) -> Dict[str, Any]:
        """Return all tunables with their current values and metadata."""
        with self._lock:
            vals = dict(self._vals)
        result = {}
        for k, spec in TUNABLES.items():
            result[k] = {
                "value":       vals[k],
                "default":     spec["default"],
                "min":         spec["min"],
                "max":         spec["max"],
                "label":       spec["label"],
                "unit":        spec["unit"],
                "description": spec["description"],
            }
        return result

    def reset_to_defaults(self) -> None:
        """Reset all values to config.py defaults and persist."""
        with self._lock:
            self._vals = {k: float(v["default"]) for k, v in TUNABLES.items()}
            self._save_locked()
        log.info("RuntimeCfg: reset to defaults")

    # ── Persistence ───────────────────────────────────────────────────────────

    def _save_locked(self) -> None:
        """Write current values to JSON (must be called with self._lock held)."""
        try:
            self._path.write_text(json.dumps(self._vals, indent=2))
        except Exception as exc:
            log.error("RuntimeCfg save failed: %s", exc)

    def _load(self) -> None:
        """Load persisted overrides from JSON, ignoring unknown or out-of-range keys."""
        if not self._path.exists():
            return
        raw = self._path.read_text().strip()
        if not raw:
            return
        try:
            data = json.loads(raw)
            loaded = 0
            for k, v in data.items():
                if k not in TUNABLES:
                    continue
                spec = TUNABLES[k]
                clamped = float(max(spec["min"], min(spec["max"], float(v))))
                self._vals[k] = clamped
                loaded += 1
            log.info("RuntimeCfg: loaded %d overrides from %s", loaded, self._path)
        except Exception as exc:
            log.warning("RuntimeCfg load failed: %s", exc)


# ── Module-level singleton ────────────────────────────────────────────────────
# Import this anywhere:  from app.robot.runtime_cfg import rtcfg
rtcfg: RuntimeCfg  # assigned in main.py after Flask app is created

def init(path: Path) -> RuntimeCfg:
    """Create and register the singleton.  Called once from main.py."""
    global rtcfg
    rtcfg = RuntimeCfg(path)
    return rtcfg
