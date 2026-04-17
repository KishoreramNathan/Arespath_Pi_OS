"""Points of Interest (POI) manager.

Supports named waypoints (gazebo, inspection point, charging dock, etc.)
that are persisted as JSON and displayed on the mission map.

POI schema (one entry)::

    {
        "id":    "poi_1712345678_0",
        "label": "Gazebo",
        "kind":  "gazebo",          # gazebo | waypoint | dock | custom
        "x":     12.5,              # world metres
        "y":     -3.2,
        "note":  "optional text"
    }

Public API
──────────
    manager.add(label, kind, x, y, note) → poi dict
    manager.update(poi_id, **fields)      → updated poi dict | None
    manager.remove(poi_id)               → bool
    manager.list()                       → list[dict]
    manager.get(poi_id)                  → dict | None
    manager.navigate_to(poi_id)          → (x, y) | None
"""

import json
import logging
import threading
import time
import uuid
from pathlib import Path
from typing import Dict, List, Optional, Tuple

log = logging.getLogger(__name__)

KIND_ICONS: Dict[str, str] = {
    "gazebo":    "⛺",
    "waypoint":  "📍",
    "dock":      "🔌",
    "custom":    "⭐",
}

_VALID_KINDS = set(KIND_ICONS)


class PoiManager:
    def __init__(self, store_path: Path) -> None:
        self._path = store_path
        self._lock = threading.Lock()
        self._pois: Dict[str, dict] = {}
        self._load()

    # ── CRUD ──────────────────────────────────────────────────────────────────

    def add(self, label: str, kind: str, x: float, y: float,
            note: str = "") -> dict:
        kind = kind if kind in _VALID_KINDS else "custom"
        poi_id = f"poi_{int(time.time())}_{uuid.uuid4().hex[:8]}"
        entry = {
            "id":    poi_id,
            "label": label[:64],
            "kind":  kind,
            "x":     round(float(x), 4),
            "y":     round(float(y), 4),
            "note":  note[:256],
            "created": int(time.time()),
        }
        with self._lock:
            self._pois[poi_id] = entry
            self._save()
        log.info("POI added: %s @ (%.2f, %.2f)", label, x, y)
        return entry

    def update(self, poi_id: str, **fields) -> Optional[dict]:
        with self._lock:
            if poi_id not in self._pois:
                return None
            entry = self._pois[poi_id]
            for k, v in fields.items():
                if k in ("label", "kind", "note", "x", "y"):
                    if k in ("x", "y"):
                        v = round(float(v), 4)
                    elif k == "kind":
                        v = v if v in _VALID_KINDS else "custom"
                    elif k in ("label", "note"):
                        v = str(v)[:256]
                    entry[k] = v
            self._save()
            return dict(entry)

    def remove(self, poi_id: str) -> bool:
        with self._lock:
            if poi_id not in self._pois:
                return False
            del self._pois[poi_id]
            self._save()
            log.info("POI removed: %s", poi_id)
            return True

    def list(self) -> List[dict]:
        with self._lock:
            return [dict(v) for v in self._pois.values()]

    def get(self, poi_id: str) -> Optional[dict]:
        with self._lock:
            entry = self._pois.get(poi_id)
            return dict(entry) if entry else None

    def navigate_to(self, poi_id: str) -> Optional[Tuple[float, float]]:
        entry = self.get(poi_id)
        if entry is None:
            return None
        return (entry["x"], entry["y"])

    # ── Persistence ───────────────────────────────────────────────────────────

    def _save(self) -> None:
        try:
            self._path.write_text(
                json.dumps(list(self._pois.values()), indent=2)
            )
        except Exception as exc:
            log.error("POI save failed: %s", exc)

    def _load(self) -> None:
        if not self._path.exists():
            return
        try:
            data = json.loads(self._path.read_text())
            if isinstance(data, list):
                self._pois = {e["id"]: e for e in data if "id" in e}
            log.info("Loaded %d POIs", len(self._pois))
        except Exception as exc:
            log.warning("POI load failed: %s", exc)
