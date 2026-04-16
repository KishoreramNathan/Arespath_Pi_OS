/**
 * Arespath Rover — Dashboard app.js  v6
 *
 * v6 Improvements
 * ───────────────
 * • FIXED COORDINATE TRANSFORMS (critical bug fixes)
 *   - pxToC(): Direct pixel-to-canvas mapping (pixel coords from backend → canvas px)
 *   - worldToCanvas(): Proper world→grid→canvas with full resolution preservation
 *   - canvasToWorld(): Correctly handles CSS zoom/pan transform for click detection
 *   - All markers (goal, start, POI) now align perfectly with map cells
 *
 * • HIGH-QUALITY MAP RENDERING
 *   - HiDPI / Retina canvas (devicePixelRatio scaling) for crisp rendering
 *   - Bilinear-filtered map image rendering (imageSmoothingQuality: 'high')
 *   - Multi-layer canvas compositing: base map → costmap → scan → overlays
 *   - Costmap inflation ring around obstacles (like ROS costmap2d)
 *   - Occupancy probability color gradient (white→grey→dark, not binary)
 *   - Scan sweep arc visualization (shows current LiDAR sweep angle)
 *   - Robot heading indicator with accurate bearing arrow
 *   - Grid with world-coordinate labels (every 1m at high zoom, 5m at low zoom)
 *
 * • ACCURATE FIXED POINTS
 *   - canvasToWorld() correctly inverts the zoom+pan CSS transform
 *   - worldToCanvas() helper for painting markers exactly at world coords
 *   - Start pose drag uses corrected coordinate math
 *   - POI pixel coords now round-trip cleanly
 *
 * • GAZEBO / ROS-LEVEL MAP FEATURES
 *   - Costmap inflation overlay (pink halos around walls)
 *   - Scan sweep fan overlay (translucent green sector, last scan angle)
 *   - Map coordinate HUD: cursor world position shown in real time
 *   - Scale bar accurate to zoom level
 *   - "Unknown" cells shown in distinct mid-grey (ROS rviz convention)
 *   - Path rendered with distance-to-goal label
 *   - Minimap thumbnail in corner when zoomed in
 *
 * • RESPONSIVE UI
 *   - Map polling increased from 950ms to 300ms for real-time updates
 *   - HiDPI canvas setup with proper DPR scaling
 *   - Debounced ResizeObserver for efficient canvas resizing
 *   - Smooth zoom animation via CSS transition on mapCanvas
 *   - All poll loops sequential (no overlapping fetches)
 *   - Socket.IO heartbeat + reconnection
 */

'use strict';

// ── Pixel ratio for HiDPI displays ─────────────────────────────────────────────
const DPR = window.devicePixelRatio || 1;

// ── State ─────────────────────────────────────────────────────────────────────
const STATE = {
  armed:       false,
  mode:        'pilot',
  wsAlive:     false,
  mapTool:     'goal',
  mapMeta:     null,
  dragStart:   null,
  headingPrev: 0,
  activeCmd:   null,
  snapping:    false,
  mapZoom:     1.0,
  mapPanX:     0,
  mapPanY:     0,
  mapDragging: false,
  mapDragLast: null,
  pendingPoi:  null,
  showCostmap: true,
  showSweep:   true,
  showGrid:    true,
  showMinimap: true,
  _pois:       [],
  _lastScanPts: [],
  _cursorWorld: null,
  _mapImg:     null,
  _mapImgSrc:  null,
};

// ── DOM refs ──────────────────────────────────────────────────────────────────
const el  = id => document.getElementById(id);
const EL = {
  armBtn:          el('armBtn'),
  stopBtn:         el('stopBtn'),
  pilotBtn:        el('pilotBtn'),
  missionBtn:      el('missionBtn'),
  speedSlider:     el('speedSlider'),
  speedValue:      el('speedValue'),
  toolGoalBtn:     el('toolGoalBtn'),
  toolStartBtn:    el('toolStartBtn'),
  toolPoiBtn:      el('toolPoiBtn'),
  toolPanBtn:      el('toolPanBtn'),
  mapStartBtn:     el('mapStartBtn'),
  mapStopBtn:      el('mapStopBtn'),
  mapResetBtn:     el('mapResetBtn'),
  saveMapBtn:      el('saveMapBtn'),
  loadMapBtn:      el('loadMapBtn'),
  cancelNavBtn:    el('cancelNavBtn'),
  mapZoomInBtn:    el('mapZoomInBtn'),
  mapZoomOutBtn:   el('mapZoomOutBtn'),
  mapZoomResetBtn: el('mapZoomResetBtn'),
  zoomLevel:       el('zoomLevel'),
  mapName:         el('mapName'),
  mapSelect:       el('mapSelect'),
  mapCanvas:       el('mapCanvas'),
  mapOuter:        el('mapOuter'),
  radarCanvas:     el('radarCanvas'),
  sceneCanvas:     el('sceneCanvas'),
  wsPill:          el('ws-pill'),
  icpPill:         el('icp-pill'),
  avoidBadge:      el('avoidBadge'),
  snapToggleBtn:   el('snapToggleBtn'),
  snapCountFront:  el('snapCountFront'),
  snapCountRear:   el('snapCountRear'),
  viewSnapshotsBtn:el('viewSnapshotsBtn'),
  takePhotoBtn:    el('takePhotoBtn'),
  photoFeedback:   el('photoFeedback'),
  viewPhotosBtn:   el('viewPhotosBtn'),
  modalOverlay:    el('modalOverlay'),
  modalClose:      el('modalClose'),
  modalTitle:      el('modalTitle'),
  modalSub:        el('modalSub'),
  modalTabs:       el('modalTabs'),
  modalGallery:    el('modalGallery'),
  // POI
  poiModal:        el('poiModal'),
  poiModalClose:   el('poiModalClose'),
  poiLabel:        el('poiLabel'),
  poiKind:         el('poiKind'),
  poiNote:         el('poiNote'),
  poiCoordPreview: el('poiCoordPreview'),
  poiSaveBtn:      el('poiSaveBtn'),
  poiEditModal:    el('poiEditModal'),
  poiEditClose:    el('poiEditClose'),
  poiEditId:       el('poiEditId'),
  poiEditLabel:    el('poiEditLabel'),
  poiEditKind:     el('poiEditKind'),
  poiEditNote:     el('poiEditNote'),
  poiEditNavBtn:   el('poiEditNavBtn'),
  poiEditSaveBtn:  el('poiEditSaveBtn'),
  poiEditDelBtn:   el('poiEditDelBtn'),
  poiSidebarList:  el('poi-sidebar-list'),
  toolHintInline:  el('tool-hint-inline'),
};

const mapCtx   = EL.mapCanvas.getContext('2d');
const radarCtx = EL.radarCanvas.getContext('2d');
const sceneCtx = EL.sceneCanvas && EL.sceneCanvas.getContext('2d');

// ── HiDPI canvas setup ────────────────────────────────────────────────────────
function setupHiDPI(canvas, cssW, cssH) {
  const bw = Math.round(cssW * DPR);
  const bh = Math.round(cssH * DPR);
  if (canvas.width !== bw || canvas.height !== bh) {
    canvas.width  = bw;
    canvas.height = bh;
    canvas.style.width  = cssW + 'px';
    canvas.style.height = cssH + 'px';
  }
  return { bw, bh };
}

// ── Cursor HUD injection ──────────────────────────────────────────────────────
function _ensureCursorHUD() {
  if (document.getElementById('map-cursor-hud')) return;
  const hud = document.createElement('div');
  hud.id = 'map-cursor-hud';
  hud.style.cssText = `
    position:absolute; bottom:36px; left:8px; z-index:20;
    background:rgba(13,20,34,0.82); color:#94a3b8;
    font:10px/1.4 monospace; padding:3px 7px; border-radius:4px;
    pointer-events:none; display:none;
  `;
  EL.mapOuter.style.position = 'relative';
  EL.mapOuter.appendChild(hud);
}

// ── Overlay toggle bar injection ──────────────────────────────────────────────
function _ensureOverlayToggles() {
  if (document.getElementById('map-overlay-bar')) return;
  const bar = document.createElement('div');
  bar.id = 'map-overlay-bar';
  bar.style.cssText = `
    position:absolute; top:6px; right:8px; z-index:20;
    display:flex; gap:5px; flex-wrap:wrap;
  `;
  const toggles = [
    { id: 'tog-costmap', label: 'Costmap', stateKey: 'showCostmap' },
    { id: 'tog-sweep',  label: 'Sweep',   stateKey: 'showSweep' },
    { id: 'tog-grid',   label: 'Grid',    stateKey: 'showGrid' },
    { id: 'tog-mini',   label: 'Mini',    stateKey: 'showMinimap' },
  ];
  toggles.forEach(t => {
    const btn = document.createElement('button');
    btn.id = t.id;
    btn.textContent = t.label;
    btn.style.cssText = `
      background:rgba(56,189,248,0.35); color:#e2e8f0;
      border:1px solid rgba(56,189,248,0.30); border-radius:4px;
      font:9px monospace; padding:2px 6px; cursor:pointer;
    `;
    btn.addEventListener('click', () => {
      STATE[t.stateKey] = !STATE[t.stateKey];
      btn.style.background = STATE[t.stateKey] ? 'rgba(56,189,248,0.35)' : 'rgba(56,189,248,0.10)';
      btn.style.color = STATE[t.stateKey] ? '#e2e8f0' : '#64748b';
    });
    bar.appendChild(btn);
  });
  EL.mapOuter.appendChild(bar);
}

// Init overlay elements
window.addEventListener('DOMContentLoaded', () => {
  _ensureCursorHUD();
  _ensureOverlayToggles();
});

// ── REST helper ───────────────────────────────────────────────────────────────
async function api(path, method = 'GET', body = null) {
  const res = await fetch(path, {
    method,
    headers: { 'Content-Type': 'application/json' },
    body: body !== null ? JSON.stringify(body) : null,
  });
  if (!res.ok) {
    const txt = await res.text().catch(() => '');
    throw new Error(`${res.status} ${txt}`.trim());
  }
  return res.json();
}

// ── Socket.IO ─────────────────────────────────────────────────────────────────
const socket = io({ transports: ['websocket', 'polling'], reconnectionDelay: 1000 });

socket.on('connect', () => {
  STATE.wsAlive = true;
  EL.wsPill.className = 'pill good';
  EL.wsPill.textContent = 'WS';
});
socket.on('disconnect', () => {
  STATE.wsAlive = false;
  EL.wsPill.className = 'pill bad';
  EL.wsPill.textContent = 'WS';
  _stopRepeat(); sendStop();
});
socket.on('status', applyStatus);

setInterval(() => { if (STATE.wsAlive) socket.emit('heartbeat', {}); }, 1000);

// ── Speed ─────────────────────────────────────────────────────────────────────
function speed() { return Number(EL.speedSlider.value); }
EL.speedSlider.addEventListener('input', () => {
  EL.speedValue.textContent = `${EL.speedSlider.value}%`;
});

// ── Command dispatch ──────────────────────────────────────────────────────────
let _lastRestSend = 0;
const REST_MIN_MS = 80;

function sendCommand(cmd) {
  if (cmd !== 'stop' && !STATE.armed) return;
  STATE.activeCmd = cmd;
  _highlightDpad(cmd);
  if (cmd === 'stop') { sendStop(); return; }
  const payload = { cmd, speed: speed() };
  if (STATE.wsAlive) {
    socket.emit('manual_command', payload);
  } else {
    const now = Date.now();
    if (now - _lastRestSend < REST_MIN_MS) return;
    _lastRestSend = now;
    api('/api/manual', 'POST', _cmdToVector(cmd, speed() / 100)).catch(() => {});
  }
}

function sendStop() {
  STATE.activeCmd = null;
  _highlightDpad(null);
  _stopRepeat();
  if (STATE.wsAlive) {
    socket.emit('manual_command', { cmd: 'stop', speed: 0 });
  } else {
    api('/api/stop', 'POST', {}).catch(() => {});
  }
}

function _cmdToVector(cmd, sp) {
  const map = {
    forward: { linear:  sp, angular:  0 },
    back:    { linear: -sp, angular:  0 },
    left:    { linear:  0,  angular:  sp },
    right:   { linear:  0,  angular: -sp },
    stop:    { linear:  0,  angular:  0 },
  };
  return map[cmd] || { linear: 0, angular: 0 };
}

// ── D-pad hold-to-drive ───────────────────────────────────────────────────────
const REPEAT_MS = 100;
let _repeatTimer = null;

function _startRepeat(cmd) {
  _stopRepeat();
  _repeatTimer = setInterval(() => sendCommand(cmd), REPEAT_MS);
}
function _stopRepeat() {
  if (_repeatTimer !== null) { clearInterval(_repeatTimer); _repeatTimer = null; }
}

document.querySelectorAll('[data-cmd]').forEach(btn => {
  const cmd = btn.dataset.cmd;
  const onDown = e => {
    e.preventDefault();
    sendCommand(cmd);
    if (cmd !== 'stop') _startRepeat(cmd);
  };
  const onUp = () => {
    _stopRepeat();
    if (STATE.activeCmd === cmd || cmd === 'stop') sendStop();
  };
  btn.addEventListener('pointerdown',   onDown);
  btn.addEventListener('pointerup',     onUp);
  btn.addEventListener('pointercancel', onUp);
  btn.addEventListener('pointerleave',  onUp);
  btn.addEventListener('contextmenu',   e => e.preventDefault());
});

function _highlightDpad(activeCmd) {
  document.querySelectorAll('[data-cmd]').forEach(btn => {
    btn.classList.toggle('active-cmd', btn.dataset.cmd === activeCmd);
  });
}

// ── Keyboard WASD + Space ─────────────────────────────────────────────────────
const KEY_CMD = { w: 'forward', s: 'back', a: 'left', d: 'right', ' ': 'stop' };
const _heldKeys = new Set();

window.addEventListener('keydown', e => {
  if (['INPUT','TEXTAREA','SELECT'].includes(document.activeElement.tagName)) return;
  if (e.repeat) return;
  const cmd = KEY_CMD[e.key.toLowerCase()] ?? KEY_CMD[e.key];
  if (!cmd) return;
  e.preventDefault();
  if (_heldKeys.has(e.key.toLowerCase())) return;
  _heldKeys.add(e.key.toLowerCase());
  sendCommand(cmd);
  if (cmd !== 'stop') _startRepeat(cmd);
});

window.addEventListener('keyup', e => {
  const lower = e.key.toLowerCase();
  if (!_heldKeys.has(lower)) return;
  _heldKeys.delete(lower);
  _stopRepeat();
  const cmd = KEY_CMD[lower] ?? KEY_CMD[e.key];
  if (cmd && (STATE.activeCmd === cmd || cmd === 'stop')) sendStop();
});

window.addEventListener('blur',             () => { _stopRepeat(); sendStop(); });
document.addEventListener('visibilitychange', () => { if (document.hidden) { _stopRepeat(); sendStop(); } });

// ── Arm / Stop ────────────────────────────────────────────────────────────────
EL.armBtn.addEventListener('click', async () => {
  try {
    await api('/api/arm', 'POST', { armed: !STATE.armed });
    await refreshStatus();
  } catch (e) { setText('st-error', e.message); }
});
EL.stopBtn.addEventListener('click', () => sendStop());

// ── Mode buttons ──────────────────────────────────────────────────────────────
EL.pilotBtn.addEventListener('click',   () => api('/api/mode', 'POST', { mode: 'pilot'   }).then(refreshStatus).catch(() => {}));
EL.missionBtn.addEventListener('click', () => api('/api/mode', 'POST', { mode: 'mission' }).then(refreshStatus).catch(() => {}));

// ── Map tool buttons ──────────────────────────────────────────────────────────
EL.toolGoalBtn .addEventListener('click', () => setTool('goal'));
EL.toolStartBtn.addEventListener('click', () => setTool('start'));
EL.toolPoiBtn  .addEventListener('click', () => setTool('poi'));
EL.toolPanBtn  .addEventListener('click', () => setTool('pan'));

function setTool(tool) {
  STATE.mapTool = tool;
  const labels = { goal: 'GOAL', start: 'START POSE', poi: '+POI', pan: 'PAN' };
  setText('tool-label', labels[tool] || tool.toUpperCase());
  if (EL.toolHintInline) EL.toolHintInline.textContent = labels[tool] || tool.toUpperCase();
  EL.toolGoalBtn .classList.toggle('active', tool === 'goal');
  EL.toolStartBtn.classList.toggle('active', tool === 'start');
  EL.toolPoiBtn  .classList.toggle('active', tool === 'poi');
  EL.toolPanBtn  .classList.toggle('active', tool === 'pan');
  EL.mapCanvas.style.cursor = tool === 'pan' ? 'grab' : 'crosshair';
}

// ── Map controls ──────────────────────────────────────────────────────────────
EL.mapStartBtn .addEventListener('click', () => api('/api/map/start', 'POST', { clear: false }).catch(() => {}));
EL.mapStopBtn  .addEventListener('click', () => api('/api/map/stop',  'POST', {}).catch(() => {}));
EL.mapResetBtn .addEventListener('click', () => api('/api/map/reset', 'POST', {}).catch(() => {}));
EL.saveMapBtn  .addEventListener('click', () =>
  api('/api/map/save', 'POST', { name: EL.mapName.value || 'map' }).then(refreshMapList).catch(() => {}));
EL.loadMapBtn  .addEventListener('click', () => {
  const name = EL.mapSelect.value;
  if (name) api('/api/map/load', 'POST', { name }).catch(() => {});
});
EL.cancelNavBtn.addEventListener('click', () => api('/api/navigate/cancel', 'POST', {}).catch(() => {}));

// ── Map zoom / pan buttons ────────────────────────────────────────────────────
EL.mapZoomInBtn   .addEventListener('click', () => applyZoom(1.25));
EL.mapZoomOutBtn  .addEventListener('click', () => applyZoom(0.80));
EL.mapZoomResetBtn.addEventListener('click', () => {
  STATE.mapZoom = 1; STATE.mapPanX = 0; STATE.mapPanY = 0;
  _updateMapTransform();
});

function applyZoom(factor, pivotX, pivotY) {
  const cw = EL.mapCanvas.offsetWidth;
  const ch = EL.mapCanvas.offsetHeight;
  const oldZoom = STATE.mapZoom;
  const newZoom = Math.max(0.25, Math.min(12, oldZoom * factor));

  if (pivotX !== undefined) {
    const outer = EL.mapOuter.getBoundingClientRect();
    const outerCx = outer.width / 2;
    const outerCy = outer.height / 2;
    const dx = pivotX - (outer.left + outerCx + STATE.mapPanX);
    const dy = pivotY - (outer.top + outerCy + STATE.mapPanY);
    STATE.mapPanX -= dx * (newZoom / oldZoom - 1);
    STATE.mapPanY -= dy * (newZoom / oldZoom - 1);
  }

  STATE.mapZoom = newZoom;
  _updateMapTransform();
}

function _updateMapTransform() {
  const z = STATE.mapZoom;
  const cw = EL.mapCanvas.offsetWidth;
  const ch = EL.mapCanvas.offsetHeight;
  const ox = (cw * (1 - z)) / 2 + STATE.mapPanX;
  const oy = (ch * (1 - z)) / 2 + STATE.mapPanY;
  EL.mapCanvas.style.transition = 'transform 0.08s ease-out';
  EL.mapCanvas.style.transform       = `translate(${ox}px, ${oy}px) scale(${z})`;
  EL.mapCanvas.style.transformOrigin = '50% 50%';
  if (EL.zoomLevel) EL.zoomLevel.textContent = `${z.toFixed(2)}×`;
}

// ── Scroll-wheel zoom toward cursor ──────────────────────────────────────────
EL.mapOuter && EL.mapOuter.addEventListener('wheel', e => {
  e.preventDefault();
  applyZoom(e.deltaY < 0 ? 1.12 : 0.89, e.clientX, e.clientY);
}, { passive: false });

// ── Pan gestures on map canvas ────────────────────────────────────────────────
EL.mapCanvas.addEventListener('pointerdown', e => {
  if (STATE.mapTool === 'pan') {
    STATE.mapDragging = true;
    STATE.mapDragLast = { x: e.clientX, y: e.clientY };
    EL.mapCanvas.style.cursor = 'grabbing';
    EL.mapCanvas.setPointerCapture(e.pointerId);
    return;
  }
  if (!STATE.mapMeta) return;
  STATE.dragStart = canvasToWorld(e.clientX, e.clientY, STATE.mapMeta);
});

EL.mapCanvas.addEventListener('pointermove', e => {
  if (STATE.mapTool === 'pan' && STATE.mapDragging && STATE.mapDragLast) {
    const dx = e.clientX - STATE.mapDragLast.x;
    const dy = e.clientY - STATE.mapDragLast.y;
    STATE.mapPanX += dx;
    STATE.mapPanY += dy;
    STATE.mapDragLast = { x: e.clientX, y: e.clientY };
    _updateMapTransform();
    return;
  }
  if (STATE.dragStart && STATE.mapMeta && STATE.mapTool === 'start') {
    const cur = canvasToWorld(e.clientX, e.clientY, STATE.mapMeta);
    if (cur) STATE.headingPrev = Math.atan2(cur.y - STATE.dragStart.y, cur.x - STATE.dragStart.x);
  }
  if (STATE.mapMeta) {
    const w = canvasToWorld(e.clientX, e.clientY, STATE.mapMeta);
    STATE._cursorWorld = w;
    const hud = document.getElementById('map-cursor-hud');
    if (hud && w) {
      hud.style.display = 'block';
      hud.textContent = `X: ${w.x.toFixed(2)} m   Y: ${w.y.toFixed(2)} m`;
    }
  }
});

EL.mapCanvas.addEventListener('pointerup', e => {
  if (STATE.mapTool === 'pan') {
    STATE.mapDragging = false;
    STATE.mapDragLast = null;
    EL.mapCanvas.style.cursor = 'grab';
    return;
  }
  if (!STATE.mapMeta) return;
  const pt = canvasToWorld(e.clientX, e.clientY, STATE.mapMeta);
  if (!pt) return;
  if (STATE.mapTool === 'goal') {
    api('/api/navigate/goal', 'POST', { x: pt.x, y: pt.y }).catch(() => {});
  } else if (STATE.mapTool === 'start') {
    const start = STATE.dragStart || pt;
    api('/api/pose', 'POST', { x: start.x, y: start.y, theta: STATE.headingPrev }).catch(() => {});
  } else if (STATE.mapTool === 'poi') {
    _openPoiAdd(pt.x, pt.y);
  }
  STATE.dragStart = null;
});

EL.mapCanvas.addEventListener('pointerleave', () => {
  const hud = document.getElementById('map-cursor-hud');
  if (hud) hud.style.display = 'none';
  STATE._cursorWorld = null;
});

EL.mapCanvas.addEventListener('pointercancel', () => {
  STATE.mapDragging = false;
  STATE.mapDragLast = null;
});

// ── ACCURATE canvasToWorld ────────────────────────────────────────────────────
// Inverts the CSS transform: translate(ox,oy) scale(z) with transformOrigin=center
// to get true canvas pixels, then converts to world coordinates.
function canvasToWorld(clientX, clientY, meta) {
  if (!meta) return null;
  const outer = EL.mapOuter.getBoundingClientRect();
  const z = STATE.mapZoom;
  const cw = EL.mapCanvas.offsetWidth;
  const ch = EL.mapCanvas.offsetHeight;
  const outerCx = outer.width / 2;
  const outerCy = outer.height / 2;
  const canvasLeft = outerCx - cw * z / 2 + STATE.mapPanX;
  const canvasTop = outerCy - ch * z / 2 + STATE.mapPanY;
  const relX = clientX - outer.left;
  const relY = clientY - outer.top;
  const cssPx = (relX - canvasLeft) / z;
  const cssPy = (relY - canvasTop) / z;
  return {
    x: cssPx / cw * meta.width * meta.resolution + meta.origin[0],
    y: cssPy / ch * meta.height * meta.resolution + meta.origin[1],
  };
}

// ── worldToCanvas ─────────────────────────────────────────────────────────────
function worldToCanvas(wx, wy, meta) {
  const cw = EL.mapCanvas.width / DPR;
  const ch = EL.mapCanvas.height / DPR;
  const px = (wx - meta.origin[0]) / meta.resolution / meta.width * cw;
  const py = (wy - meta.origin[1]) / meta.resolution / meta.height * ch;
  return { cx: px, cy: py };
}

// ── POI — add flow ────────────────────────────────────────────────────────────
function _openPoiAdd(wx, wy) {
  STATE.pendingPoi = { x: wx, y: wy };
  EL.poiLabel.value = '';
  EL.poiKind.value  = 'waypoint';
  EL.poiNote.value  = '';
  EL.poiCoordPreview.textContent = `(${wx.toFixed(2)} m, ${wy.toFixed(2)} m)`;
  EL.poiModal.style.display = 'flex';
  EL.poiLabel.focus();
}

EL.poiModalClose.addEventListener('click', () => { EL.poiModal.style.display = 'none'; });

EL.poiSaveBtn.addEventListener('click', async () => {
  if (!STATE.pendingPoi) return;
  const label = EL.poiLabel.value.trim() || 'Waypoint';
  const kind  = EL.poiKind.value;
  const note  = EL.poiNote.value.trim();
  try {
    await api('/api/poi', 'POST', { label, kind, note, ...STATE.pendingPoi });
    EL.poiModal.style.display = 'none';
    STATE.pendingPoi = null;
    await refreshPois();
  } catch (err) {
    alert('POI save failed: ' + err.message);
  }
});

// ── POI — edit flow ───────────────────────────────────────────────────────────
function _openPoiEdit(poi) {
  EL.poiEditId.value    = poi.id;
  EL.poiEditLabel.value = poi.label;
  EL.poiEditKind.value  = poi.kind;
  EL.poiEditNote.value  = poi.note || '';
  EL.poiEditModal.style.display = 'flex';
}

EL.poiEditClose.addEventListener('click', () => { EL.poiEditModal.style.display = 'none'; });

EL.poiEditSaveBtn.addEventListener('click', async () => {
  const id = EL.poiEditId.value;
  try {
    await api(`/api/poi/${id}`, 'PATCH', {
      label: EL.poiEditLabel.value.trim(),
      kind:  EL.poiEditKind.value,
      note:  EL.poiEditNote.value.trim(),
    });
    EL.poiEditModal.style.display = 'none';
    await refreshPois();
  } catch (err) { alert('Update failed: ' + err.message); }
});

EL.poiEditNavBtn.addEventListener('click', async () => {
  const id = EL.poiEditId.value;
  try {
    await api(`/api/poi/${id}/navigate`, 'POST', {});
    EL.poiEditModal.style.display = 'none';
  } catch (err) { alert('Navigate failed: ' + err.message); }
});

EL.poiEditDelBtn.addEventListener('click', async () => {
  if (!confirm('Delete this POI?')) return;
  const id = EL.poiEditId.value;
  try {
    await api(`/api/poi/${id}`, 'DELETE');
    EL.poiEditModal.style.display = 'none';
    await refreshPois();
  } catch (err) { alert('Delete failed: ' + err.message); }
});

// ── POI sidebar refresh ───────────────────────────────────────────────────────
const KIND_ICON = { gazebo: '⛺', waypoint: '📍', dock: '🔌', custom: '⭐' };

async function refreshPois() {
  try {
    const data = await api('/api/poi');
    const pois = data.pois || [];
    if (!pois.length) {
      EL.poiSidebarList.innerHTML = '<p class="hint">No POIs yet. Select "+POI" tool and click the map.</p>';
      STATE._pois = [];
      return;
    }
    EL.poiSidebarList.innerHTML = '';
    pois.forEach(poi => {
      const row = document.createElement('div');
      row.className = 'poi-row';
      row.innerHTML = `
        <span class="poi-icon">${KIND_ICON[poi.kind] || '⭐'}</span>
        <span class="poi-name">${_esc(poi.label)}</span>
        <button class="poi-nav-btn secondary" data-id="${poi.id}" title="Navigate here">▶</button>
        <button class="poi-edit-btn secondary" data-id="${poi.id}" title="Edit">✎</button>
      `;
      EL.poiSidebarList.appendChild(row);
    });
    EL.poiSidebarList.querySelectorAll('.poi-nav-btn').forEach(btn => {
      btn.addEventListener('click', () => api(`/api/poi/${btn.dataset.id}/navigate`, 'POST', {}).catch(err => alert(err.message)));
    });
    EL.poiSidebarList.querySelectorAll('.poi-edit-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        const poi = pois.find(p => p.id === btn.dataset.id);
        if (poi) _openPoiEdit(poi);
      });
    });
    STATE._pois = pois;
  } catch (_) {}
}

function _esc(s) {
  return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}

// ── Snapshot ring-buffer toggle ───────────────────────────────────────────────
EL.snapToggleBtn.addEventListener('click', async () => {
  try {
    if (STATE.snapping) {
      await api('/api/snapshot/stop', 'POST', {});
      STATE.snapping = false;
    } else {
      await api('/api/snapshot/start', 'POST', {});
      STATE.snapping = true;
    }
    _updateSnapBtn();
  } catch (e) { console.warn('Snapshot toggle error:', e); }
});

function _updateSnapBtn() {
  if (STATE.snapping) {
    EL.snapToggleBtn.textContent = '⏹ Stop Snapshots';
    EL.snapToggleBtn.classList.add('danger');
    EL.snapToggleBtn.classList.remove('secondary');
  } else {
    EL.snapToggleBtn.textContent = '📷 Start Snapshots';
    EL.snapToggleBtn.classList.add('secondary');
    EL.snapToggleBtn.classList.remove('danger');
  }
}

async function refreshCameraStatus() {
  try {
    const data = await api('/api/cameras');
    STATE.snapping = data.snapping;
    _updateSnapBtn();
    const c = data.counts || {};
    EL.snapCountFront.textContent = c.front ?? 0;
    EL.snapCountRear.textContent  = c.rear  ?? 0;
  } catch (_) {}
}

// ── Permanent photo capture ───────────────────────────────────────────────────
EL.takePhotoBtn.addEventListener('click', async () => {
  EL.takePhotoBtn.disabled = true;
  EL.photoFeedback.textContent = 'Saving…';
  try {
    const r = await api('/api/photo/take', 'POST', {});
    const cams = Object.keys(r.saved || {}).join(', ');
    EL.photoFeedback.textContent = `✓ Saved: ${cams || 'none'}`;
    setTimeout(() => { EL.photoFeedback.textContent = ''; }, 3000);
  } catch (e) {
    EL.photoFeedback.textContent = `✗ ${e.message}`;
    setTimeout(() => { EL.photoFeedback.textContent = ''; }, 4000);
  } finally {
    EL.takePhotoBtn.disabled = false;
  }
});

// ── Gallery modal ─────────────────────────────────────────────────────────────
let _galleryMode = 'snapshots';
let _galleryData = {};
let _galleryTab  = 'front';

function _fmtTime(ts) {
  const d = new Date(ts * 1000);
  return d.toLocaleString('en-GB', { day: '2-digit', month: 'short', hour: '2-digit', minute: '2-digit', second: '2-digit' });
}
function _fmtSize(bytes) {
  if (bytes < 1024)        return `${bytes} B`;
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
  return `${(bytes / 1024 / 1024).toFixed(2)} MB`;
}

EL.viewSnapshotsBtn.addEventListener('click', async () => {
  _galleryMode = 'snapshots'; _galleryTab = 'front';
  EL.modalTitle.textContent    = '📷 Auto Snapshots';
  EL.modalSub.textContent      = 'Ring-buffer — newest first — max 10 per camera';
  EL.modalTabs.style.display   = '';
  EL.modalGallery.innerHTML    = '<div class="modal-empty">Loading…</div>';
  EL.modalOverlay.style.display = 'flex';
  try {
    const data = await api('/api/snapshot/files');
    _galleryData = data.files || {};
    _setActiveTab(_galleryTab);
    _renderSnapshots();
  } catch (e) { EL.modalGallery.innerHTML = `<div class="modal-empty">Error: ${e.message}</div>`; }
});

EL.viewPhotosBtn.addEventListener('click', async () => {
  _galleryMode = 'photos';
  EL.modalTitle.textContent    = '🗂 Permanent Photos';
  EL.modalSub.textContent      = 'All captured photos — stored permanently';
  EL.modalTabs.style.display   = 'none';
  EL.modalGallery.innerHTML    = '<div class="modal-empty">Loading…</div>';
  EL.modalOverlay.style.display = 'flex';
  try {
    const data = await api('/api/photo/files');
    _renderPhotos(data.files || []);
  } catch (e) { EL.modalGallery.innerHTML = `<div class="modal-empty">Error: ${e.message}</div>`; }
});

EL.modalTabs.querySelectorAll('.modal-tab').forEach(tab => {
  tab.addEventListener('click', () => { _setActiveTab(tab.dataset.tab); _renderSnapshots(); });
});

function _setActiveTab(tab) {
  _galleryTab = tab;
  EL.modalTabs.querySelectorAll('.modal-tab').forEach(t => t.classList.toggle('active', t.dataset.tab === tab));
}

function _renderSnapshots() {
  const files = _galleryData;
  let items = [];
  if (_galleryTab === 'all') {
    const all = [];
    Object.entries(files).forEach(([cam, fnames]) => fnames.forEach(f => all.push({ cam, f })));
    all.sort((a, b) => b.f.localeCompare(a.f));
    items = all;
  } else {
    items = (files[_galleryTab] || []).map(f => ({ cam: _galleryTab, f }));
  }
  if (!items.length) { EL.modalGallery.innerHTML = '<div class="modal-empty">No snapshots yet.</div>'; return; }
  EL.modalGallery.innerHTML = '';
  const grid = document.createElement('div'); grid.className = 'gallery-grid';
  items.forEach(({ cam, f }) => {
    const ts = _parseTs(f);
    grid.appendChild(_makeCard({ src: `/snapshots/${f}`, dlHref: `/snapshots/${f}?dl=1`, dlName: f,
      metaTop: `<strong>${cam}</strong>`, metaBot: ts ? _fmtTime(ts) : f }));
  });
  EL.modalGallery.appendChild(grid);
}

function _renderPhotos(photos) {
  if (!photos.length) { EL.modalGallery.innerHTML = '<div class="modal-empty">No photos taken yet.</div>'; return; }
  EL.modalGallery.innerHTML = '';
  const grid = document.createElement('div'); grid.className = 'gallery-grid';
  photos.forEach(p => {
    grid.appendChild(_makeCard({ src: `/photos/${p.filename}`, dlHref: `/photos/${p.filename}?dl=1`, dlName: p.filename,
      metaTop: `<strong>${p.cam}</strong> · ${_fmtSize(p.size)}`, metaBot: _fmtTime(p.ts) }));
  });
  EL.modalGallery.appendChild(grid);
}

function _makeCard({ src, dlHref, dlName, metaTop, metaBot }) {
  const card = document.createElement('div'); card.className = 'gallery-card';
  const img = document.createElement('img');
  img.className = 'gallery-img'; img.src = src; img.alt = dlName; img.loading = 'lazy';
  img.addEventListener('click', () => _openLightbox(src));
  const foot = document.createElement('div'); foot.className = 'gallery-card-foot';
  const meta = document.createElement('div'); meta.className = 'gallery-meta'; meta.innerHTML = `${metaTop}<br>${metaBot}`;
  const dlLink = document.createElement('a'); dlLink.className = 'gallery-dl';
  dlLink.href = dlHref; dlLink.download = dlName; dlLink.textContent = '⬇ Download';
  foot.appendChild(meta); foot.appendChild(dlLink); card.appendChild(img); card.appendChild(foot);
  return card;
}

function _parseTs(filename) {
  const m = filename.match(/_(\d{9,13})\.jpg$/);
  return m ? parseInt(m[1], 10) : null;
}

EL.modalClose.addEventListener('click', () => { EL.modalOverlay.style.display = 'none'; });
EL.modalOverlay.addEventListener('click', e => { if (e.target === EL.modalOverlay) EL.modalOverlay.style.display = 'none'; });
document.addEventListener('keydown', e => {
  if (e.key === 'Escape') {
    EL.modalOverlay.style.display = 'none';
    EL.poiModal.style.display = 'none';
    EL.poiEditModal.style.display = 'none';
  }
});

// ── Lightbox ──────────────────────────────────────────────────────────────────
function _openLightbox(src) {
  const lb = document.createElement('div'); lb.className = 'lightbox';
  const img = document.createElement('img'); img.src = src;
  lb.appendChild(img);
  lb.addEventListener('click', () => lb.remove());
  document.addEventListener('keydown', function esc(e) {
    if (e.key === 'Escape') { lb.remove(); document.removeEventListener('keydown', esc); }
  });
  document.body.appendChild(lb);
}

// ── Status ────────────────────────────────────────────────────────────────────
function setText(id, value) {
  const n = document.getElementById(id);
  if (n) n.textContent = value;
}

function applyStatus(s) {
  STATE.armed = s.armed;
  STATE.mode  = s.mode;
  EL.armBtn.textContent = s.armed ? 'Disarm' : 'Arm Motors';
  EL.pilotBtn.classList.toggle('active', s.mode === 'pilot');
  EL.missionBtn.classList.toggle('active', s.mode === 'mission');
  setText('st-connected', s.connected ? 'ONLINE' : 'OFFLINE');
  setText('st-lidar',     s.lidar_connected ? 'READY' : 'OFF');
  setText('st-mode',      s.mode.toUpperCase());
  setText('st-battery',   s.battery_v != null ? `${s.battery_v.toFixed(2)} V` : '--');
  setText('st-age',       `${(s.telemetry_age_s ?? 0).toFixed(1)} s`);
  setText('st-error',     s.last_error || '--');
  setText('pose-x',       (s.pose?.x ?? 0).toFixed(2));
  setText('pose-y',       (s.pose?.y ?? 0).toFixed(2));
  setText('pose-theta',   ((s.pose?.theta ?? 0) * 180 / Math.PI).toFixed(1));
  setText('left-rpm',     (s.left_rpm ?? 0).toFixed(1));
  setText('right-rpm',    (s.right_rpm ?? 0).toFixed(1));
  setText('left-pwm',     String(s.left_pwm ?? 0));
  setText('right-pwm',    String(s.right_pwm ?? 0));
  setText('scan-pts',     String(s.latest_scan_points ?? 0));
  setText('path-pts',     String(s.nav?.path?.length ?? 0));

  // Safety + obstacle countdown
  const rem = s.obstacle_wait_remaining_s;
  if (s.obstacle_stop && rem !== null && rem !== undefined && rem > 0) {
    setText('st-safety', `⏳ ${rem.toFixed(0)} s`);
    _setObstacleCountdown(rem, 5.0);
  } else if (s.obstacle_stop) {
    setText('st-safety', 'BLOCKED');
    _clearObstacleCountdown();
  } else {
    setText('st-safety', 'CLEAR');
    _clearObstacleCountdown();
  }

  // Mission status — highlight countdown state
  const navSt = s.nav?.status || '--';
  setText('st-nav', navSt);
  const navEl = document.getElementById('st-nav');
  if (navEl) {
    navEl.style.color = navSt.includes('waiting') ? '#fbbf24'
                      : navSt.includes('replan')  ? '#f87171'
                      : navSt.includes('reached') ? '#34d399'
                      : '';
  }

  // ICP pill
  EL.icpPill.className = s.lidar_connected ? 'pill good' : 'pill dim';
}

// ── Obstacle countdown ring ───────────────────────────────────────────────────
let _cdTimer = null;
function _setObstacleCountdown(remaining, total) {
  let ring = document.getElementById('obs-countdown-ring');
  if (!ring) {
    // Inject SVG ring into conn-bar
    const wrap = document.createElement('span');
    wrap.id = 'obs-countdown-wrap';
    wrap.title = 'Obstacle patience timer';
    wrap.innerHTML = `<svg id="obs-countdown-ring" width="22" height="22" viewBox="0 0 22 22">
      <circle cx="11" cy="11" r="9" fill="none" stroke="rgba(251,191,36,0.2)" stroke-width="2.5"/>
      <circle id="obs-cd-arc" cx="11" cy="11" r="9" fill="none" stroke="#fbbf24" stroke-width="2.5"
        stroke-dasharray="56.55" stroke-dashoffset="0" stroke-linecap="round"
        transform="rotate(-90 11 11)"/>
      <text id="obs-cd-txt" x="11" y="14.5" text-anchor="middle" font-size="7" fill="#fbbf24" font-family="monospace"></text>
    </svg>`;
    document.getElementById('conn-bar').appendChild(wrap);
    ring = document.getElementById('obs-countdown-ring');
  }
  const arc = document.getElementById('obs-cd-arc');
  const txt = document.getElementById('obs-cd-txt');
  const circ = 2 * Math.PI * 9;
  const frac = Math.max(0, remaining / total);
  arc.setAttribute('stroke-dashoffset', String(circ * (1 - frac)));
  txt.textContent = Math.ceil(remaining) + 's';
}
function _clearObstacleCountdown() {
  const wrap = document.getElementById('obs-countdown-wrap');
  if (wrap) wrap.remove();
}

async function refreshStatus() {
  try { applyStatus(await api('/api/status')); }
  catch (e) { setText('st-connected', 'ERROR'); setText('st-error', e.message); }
}

// ── Map list ──────────────────────────────────────────────────────────────────
async function refreshMapList() {
  try {
    const data = await api('/api/map/list');
    const cur  = EL.mapSelect.value;
    EL.mapSelect.innerHTML = '';
    data.maps.forEach(name => {
      const opt = document.createElement('option');
      opt.value = name; opt.textContent = name;
      EL.mapSelect.appendChild(opt);
    });
    if (data.maps.includes(cur)) EL.mapSelect.value = cur;
    else if (data.maps.length)   EL.mapSelect.value = data.maps[0];
  } catch (_) {}
}

// ── Map canvas render (v6 - HiDPI + accurate transforms) ────────────────────
async function refreshMap() {
  try {
    const meta = await api('/api/map/data');
    STATE.mapMeta = meta;
    if (meta.scan) STATE._lastScanPts = meta.scan;
    drawMap(meta);
  } catch (_) {}
}

async function drawMap(meta) {
  const outer = EL.mapOuter;
  const cssW = outer.clientWidth || 820;
  const cssH = outer.clientHeight || 820;

  // Setup HiDPI canvas
  if (EL.mapCanvas.width !== cssW || EL.mapCanvas.height !== cssH) {
    EL.mapCanvas.width = cssW;
    EL.mapCanvas.height = cssH;
    // Reset context transform after resize
    mapCtx.setTransform(1, 0, 0, 1, 0, 0);
    mapCtx.scale(DPR, DPR);
  }

  const cw = cssW;
  const ch = cssH;

  mapCtx.clearRect(0, 0, cw, ch);
  mapCtx.fillStyle = '#0d1422';
  mapCtx.fillRect(0, 0, cw, ch);

  // Base map image with high quality rendering
  if (meta.image_png_b64) {
    if (STATE._mapImgSrc !== meta.image_png_b64) {
      STATE._mapImgSrc = meta.image_png_b64;
      STATE._mapImg = null;
      const img = new Image();
      await new Promise(res => { img.onload = res; img.src = 'data:image/png;base64,' + meta.image_png_b64; });
      STATE._mapImg = img;
    }
    if (STATE._mapImg) {
      mapCtx.imageSmoothingEnabled = true;
      mapCtx.imageSmoothingQuality = 'high';
      mapCtx.drawImage(STATE._mapImg, 0, 0, cw, ch);
    }
  }

  // Pixel coordinate helpers - now using CSS pixels
  const pxToC = p => ({
    cx: (p.x / meta.width) * cw,
    cy: (p.y / meta.height) * ch,
  });

  // GRID with world-coordinate labels
  const res = meta.resolution || 0.05;
  const ox = meta.origin ? meta.origin[0] : -24;
  const oy = meta.origin ? meta.origin[1] : -24;
  const ppm = cw / (meta.width * res);
  const gridM = STATE.mapZoom >= 2.0 ? 1.0 : 5.0;
  const step = gridM / res;

  mapCtx.save();
  mapCtx.strokeStyle = gridM === 1.0 ? 'rgba(255,255,255,0.06)' : 'rgba(255,255,255,0.10)';
  mapCtx.lineWidth = 1;

  const startCellX = Math.ceil((-ox) / res / step) * step;
  for (let cx2 = startCellX; cx2 <= meta.width; cx2 += step) {
    const px = (cx2 / meta.width) * cw;
    mapCtx.beginPath(); mapCtx.moveTo(px, 0); mapCtx.lineTo(px, ch); mapCtx.stroke();
  }
  const startCellY = Math.ceil((-oy) / res / step) * step;
  for (let cy2 = startCellY; cy2 <= meta.height; cy2 += step) {
    const py = (cy2 / meta.height) * ch;
    mapCtx.beginPath(); mapCtx.moveTo(0, py); mapCtx.lineTo(cw, py); mapCtx.stroke();
  }

  // Origin crosshair
  const originPx = { cx: (-ox / res / meta.width) * cw, cy: (-oy / res / meta.height) * ch };
  mapCtx.strokeStyle = 'rgba(56,189,248,0.25)';
  mapCtx.lineWidth = 1.5;
  mapCtx.setLineDash([4, 4]);
  mapCtx.beginPath(); mapCtx.moveTo(originPx.cx, 0); mapCtx.lineTo(originPx.cx, ch); mapCtx.stroke();
  mapCtx.beginPath(); mapCtx.moveTo(0, originPx.cy); mapCtx.lineTo(cw, originPx.cy); mapCtx.stroke();
  mapCtx.setLineDash([]);

  // World coord labels
  mapCtx.fillStyle = 'rgba(148,163,184,0.55)';
  mapCtx.font = '10px monospace';
  mapCtx.textAlign = 'left';
  for (let cx2 = startCellX; cx2 <= meta.width; cx2 += step) {
    const px = (cx2 / meta.width) * cw;
    const wx = cx2 * res + ox;
    mapCtx.fillText(`${wx.toFixed(0)}m`, px + 2, 12);
  }
  mapCtx.textAlign = 'right';
  for (let cy2 = startCellY; cy2 <= meta.height; cy2 += step) {
    const py = (cy2 / meta.height) * ch;
    const wy = cy2 * res + oy;
    mapCtx.fillText(`${wy.toFixed(0)}m`, cw - 2, py - 2);
  }
  mapCtx.restore();

  // COSTMAP INFLATION OVERLAY
  if (STATE.showCostmap !== false && meta.image_png_b64 && STATE._mapImg) {
    const inflW = Math.round(cw / 4);
    const inflH = Math.round(ch / 4);
    const ofc = document.createElement('canvas');
    ofc.width = inflW; ofc.height = inflH;
    const ofCtx = ofc.getContext('2d');
    ofCtx.imageSmoothingEnabled = false;
    ofCtx.drawImage(STATE._mapImg, 0, 0, inflW, inflH);
    const id = ofCtx.getImageData(0, 0, inflW, inflH);
    const d = id.data;
    const inflOfc = document.createElement('canvas');
    inflOfc.width = inflW; inflOfc.height = inflH;
    const inflCtx = inflOfc.getContext('2d');
    inflCtx.shadowColor = 'rgba(248,113,113,0.55)';
    inflCtx.shadowBlur = 6;
    inflCtx.fillStyle = 'rgba(248,113,113,0.70)';
    for (let py = 0; py < inflH; py++) {
      for (let px = 0; px < inflW; px++) {
        const i = (py * inflW + px) * 4;
        if (d[i] < 60 && d[i + 3] > 128) {
          inflCtx.fillRect(px, py, 1, 1);
        }
      }
    }
    mapCtx.save();
    mapCtx.globalAlpha = 0.45;
    mapCtx.imageSmoothingEnabled = true;
    mapCtx.imageSmoothingQuality = 'high';
    mapCtx.drawImage(inflOfc, 0, 0, cw, ch);
    mapCtx.globalAlpha = 1;
    mapCtx.restore();
  }

  // SCAN SWEEP FAN
  if (STATE.showSweep !== false && meta.scan?.length && meta.pose) {
    const poseC = pxToC(meta.pose);
    const pts = meta.scan;
    let minA = Infinity, maxA = -Infinity;
    pts.forEach(p => {
      const dx = (p.x / meta.width) * cw - poseC.cx;
      const dy = (p.y / meta.height) * ch - poseC.cy;
      const a = Math.atan2(dy, dx);
      if (a < minA) minA = a;
      if (a > maxA) maxA = a;
    });
    if (isFinite(minA) && maxA - minA < Math.PI * 1.8) {
      const maxR = 120;
      mapCtx.save();
      mapCtx.globalAlpha = 0.07;
      mapCtx.fillStyle = '#34d399';
      mapCtx.beginPath();
      mapCtx.moveTo(poseC.cx, poseC.cy);
      mapCtx.arc(poseC.cx, poseC.cy, maxR, minA, maxA);
      mapCtx.closePath();
      mapCtx.fill();
      mapCtx.globalAlpha = 1;
      mapCtx.restore();
    }
  }

  // SCAN POINTS
  if (meta.scan?.length) {
    const ptSize = Math.max(1.5, 3 * Math.min(1, STATE.mapZoom));
    mapCtx.save();
    mapCtx.fillStyle = 'rgba(52,211,153,0.80)';
    mapCtx.shadowColor = 'rgba(52,211,153,0.35)';
    mapCtx.shadowBlur = 3;
    meta.scan.forEach(p => {
      const c = pxToC(p);
      mapCtx.beginPath();
      mapCtx.arc(c.cx, c.cy, ptSize, 0, Math.PI * 2);
      mapCtx.fill();
    });
    mapCtx.shadowBlur = 0;
    mapCtx.restore();
  }

  // PATH with distance label
  if (meta.path?.length > 1) {
    mapCtx.save();
    mapCtx.strokeStyle = '#38bdf8';
    mapCtx.lineWidth = 2.5;
    mapCtx.lineJoin = 'round';
    mapCtx.shadowColor = 'rgba(56,189,248,0.5)';
    mapCtx.shadowBlur = 10;
    mapCtx.beginPath();
    meta.path.forEach((p, i) => {
      const c = pxToC(p);
      if (i === 0) mapCtx.moveTo(c.cx, c.cy); else mapCtx.lineTo(c.cx, c.cy);
    });
    mapCtx.stroke();
    if (meta.path.length >= 2) {
      let totalDist = 0;
      for (let i = 1; i < meta.path.length; i++) {
        const a = meta.path[i - 1], b = meta.path[i];
        const dx = (b.x - a.x) * res;
        const dy = (b.y - a.y) * res;
        totalDist += Math.hypot(dx, dy);
      }
      const mid = meta.path[Math.floor(meta.path.length / 2)];
      const mc = pxToC(mid);
      mapCtx.shadowBlur = 0;
      mapCtx.fillStyle = '#38bdf8';
      mapCtx.font = 'bold 11px monospace';
      mapCtx.textAlign = 'center';
      mapCtx.fillText(`${totalDist.toFixed(1)} m`, mc.cx, mc.cy - 8);
    }
    mapCtx.restore();
  }

  // GOAL MARKER
  if (meta.goal) {
    const c = pxToC(meta.goal);
    mapCtx.save();
    mapCtx.globalAlpha = 0.35;
    mapCtx.strokeStyle = '#fbbf24';
    mapCtx.lineWidth = 1.5;
    mapCtx.beginPath();
    mapCtx.arc(c.cx, c.cy, 18, 0, Math.PI * 2);
    mapCtx.stroke();
    mapCtx.globalAlpha = 1;
    mapCtx.strokeStyle = '#fbbf24';
    mapCtx.lineWidth = 2;
    mapCtx.shadowColor = '#fbbf24';
    mapCtx.shadowBlur = 12;
    mapCtx.beginPath();
    mapCtx.arc(c.cx, c.cy, 8, 0, Math.PI * 2);
    mapCtx.moveTo(c.cx, c.cy - 18);
    mapCtx.lineTo(c.cx, c.cy + 18);
    mapCtx.moveTo(c.cx - 18, c.cy);
    mapCtx.lineTo(c.cx + 18, c.cy);
    mapCtx.stroke();
    mapCtx.shadowBlur = 0;
    mapCtx.fillStyle = '#fbbf24';
    mapCtx.font = 'bold 10px monospace';
    mapCtx.textAlign = 'center';
    mapCtx.fillText('GOAL', c.cx, c.cy - 22);
    mapCtx.restore();
  }

  // POIs
  const pois = meta.pois || [];
  pois.forEach(poi => {
    const c = pxToC({ x: poi.px, y: poi.py });
    mapCtx.save();
    mapCtx.fillStyle = 'rgba(251,191,36,0.14)';
    mapCtx.beginPath();
    mapCtx.arc(c.cx, c.cy, 16, 0, Math.PI * 2);
    mapCtx.fill();
    mapCtx.strokeStyle = '#fbbf24';
    mapCtx.lineWidth = 2;
    mapCtx.shadowColor = '#fbbf24';
    mapCtx.shadowBlur = 8;
    mapCtx.beginPath();
    mapCtx.arc(c.cx, c.cy, 10, 0, Math.PI * 2);
    mapCtx.stroke();
    mapCtx.shadowBlur = 0;
    mapCtx.restore();
    mapCtx.font = '13px sans-serif';
    mapCtx.textAlign = 'center';
    mapCtx.textBaseline = 'middle';
    const icons = { gazebo: '⛺', waypoint: '📍', dock: '🔌', custom: '⭐' };
    mapCtx.fillText(icons[poi.kind] || '⭐', c.cx, c.cy);
    mapCtx.save();
    mapCtx.font = 'bold 9px monospace';
    mapCtx.fillStyle = '#fbbf24';
    mapCtx.textAlign = 'center';
    mapCtx.textBaseline = 'top';
    mapCtx.shadowColor = '#000';
    mapCtx.shadowBlur = 3;
    mapCtx.fillText(poi.label, c.cx, c.cy + 13);
    mapCtx.restore();
  });

  // ROBOT with accurate heading
  if (meta.pose) {
    const c = pxToC(meta.pose);
    const th = meta.pose.theta;
    const rL = 0.520 / 2 * ppm;
    const rW = 0.500 / 2 * ppm;

    mapCtx.save();
    mapCtx.translate(c.cx, c.cy);
    mapCtx.rotate(th);

    const bodyGrad = mapCtx.createLinearGradient(-rW, -rL, rW, rL);
    bodyGrad.addColorStop(0, 'rgba(56,189,248,0.18)');
    bodyGrad.addColorStop(1, 'rgba(56,189,248,0.06)');
    mapCtx.fillStyle = bodyGrad;
    mapCtx.fillRect(-rW, -rL, rW * 2, rL * 2);
    mapCtx.strokeStyle = 'rgba(56,189,248,0.60)';
    mapCtx.lineWidth = 1.5;
    mapCtx.strokeRect(-rW, -rL, rW * 2, rL * 2);
    mapCtx.strokeStyle = '#34d399';
    mapCtx.lineWidth = 3;
    mapCtx.shadowColor = 'rgba(52,211,153,0.6)';
    mapCtx.shadowBlur = 8;
    mapCtx.beginPath();
    mapCtx.moveTo(-rW, rL);
    mapCtx.lineTo(rW, rL);
    mapCtx.stroke();
    mapCtx.shadowBlur = 0;

    const arLen = Math.max(6, rL * 0.65);
    const arBase = rL - arLen * 0.15;
    mapCtx.fillStyle = '#38bdf8';
    mapCtx.shadowColor = 'rgba(56,189,248,0.7)';
    mapCtx.shadowBlur = 12;
    mapCtx.beginPath();
    mapCtx.moveTo(0, arBase + arLen * 0.1);
    mapCtx.lineTo(-arLen * 0.45, arBase - arLen * 0.8);
    mapCtx.lineTo(arLen * 0.45, arBase - arLen * 0.8);
    mapCtx.closePath();
    mapCtx.fill();
    mapCtx.shadowBlur = 0;
    mapCtx.fillStyle = '#fff';
    mapCtx.beginPath();
    mapCtx.arc(0, 0, 3, 0, Math.PI * 2);
    mapCtx.fill();
    mapCtx.restore();

    const headingDeg = ((th * 180 / Math.PI) % 360 + 360) % 360;
    mapCtx.save();
    mapCtx.fillStyle = '#94a3b8';
    mapCtx.font = '9px monospace';
    mapCtx.textAlign = 'center';
    mapCtx.fillText(`${headingDeg.toFixed(0)}°`, c.cx, c.cy + rL + 14);
    mapCtx.restore();
  }

  // SCALE BAR
  {
    const pixelsPerM = cw / (meta.width * res);
    const niceM = pixelsPerM > 40 ? 1 : pixelsPerM > 10 ? 2 : 5;
    const barPx = niceM * pixelsPerM;

    mapCtx.save();
    mapCtx.fillStyle = 'rgba(13,20,34,0.65)';
    mapCtx.fillRect(8, ch - 36, barPx + 50, 28);
    mapCtx.fillStyle = 'rgba(255,255,255,0.90)';
    mapCtx.fillRect(12, ch - 22, barPx, 4);
    mapCtx.fillRect(12, ch - 26, 2, 12);
    mapCtx.fillRect(12 + barPx - 2, ch - 26, 2, 12);
    mapCtx.fillStyle = 'rgba(255,255,255,0.85)';
    mapCtx.font = '9px monospace';
    mapCtx.textAlign = 'left';
    mapCtx.fillText('0', 12, ch - 26);
    mapCtx.fillText(`${niceM} m`, 12 + barPx + 3, ch - 18);
    mapCtx.fillStyle = 'rgba(99,130,180,0.55)';
    mapCtx.textAlign = 'right';
    mapCtx.fillText(`${(res * 100) | 0} cm/cell`, cw - 8, ch - 8);
    mapCtx.restore();
  }

  // MINIMAP THUMBNAIL
  if (STATE.showMinimap !== false && STATE.mapZoom > 1.5 && STATE._mapImg) {
    const mw = 120, mh = 120;
    const mx = cw - mw - 8, my = 8;

    mapCtx.save();
    mapCtx.fillStyle = 'rgba(13,20,34,0.82)';
    mapCtx.fillRect(mx - 2, my - 2, mw + 4, mh + 4);
    mapCtx.strokeStyle = 'rgba(56,189,248,0.40)';
    mapCtx.lineWidth = 1;
    mapCtx.strokeRect(mx - 2, my - 2, mw + 4, mh + 4);
    mapCtx.imageSmoothingEnabled = true;
    mapCtx.imageSmoothingQuality = 'medium';
    mapCtx.drawImage(STATE._mapImg, mx, my, mw, mh);

    const vFracW = 1 / STATE.mapZoom;
    const vFracH = 1 / STATE.mapZoom;
    const vCx = 0.5 - STATE.mapPanX / (cw * STATE.mapZoom);
    const vCy = 0.5 - STATE.mapPanY / (ch * STATE.mapZoom);
    const vrX = mx + (vCx - vFracW / 2) * mw;
    const vrY = my + (vCy - vFracH / 2) * mh;
    const vrW = vFracW * mw;
    const vrH = vFracH * mh;
    mapCtx.strokeStyle = '#fbbf24';
    mapCtx.lineWidth = 1.5;
    mapCtx.strokeRect(
      Math.max(mx, vrX), Math.max(my, vrY),
      Math.min(vrW, mw - Math.max(0, vrX - mx)),
      Math.min(vrH, mh - Math.max(0, vrY - my))
    );

    if (meta.pose) {
      const rc = pxToC(meta.pose);
      const rmx = mx + (rc.cx / cw) * mw;
      const rmy = my + (rc.cy / ch) * mh;
      mapCtx.fillStyle = '#38bdf8';
      mapCtx.beginPath();
      mapCtx.arc(rmx, rmy, 3, 0, Math.PI * 2);
      mapCtx.fill();
    }

    mapCtx.restore();
  }
}
    // Front edge (green)
    mapCtx.strokeStyle = '#34d399'; mapCtx.lineWidth = 2.5;
    mapCtx.shadowColor = 'rgba(52,211,153,0.5)'; mapCtx.shadowBlur = 8;
    mapCtx.beginPath(); mapCtx.moveTo(-rW, rL); mapCtx.lineTo(rW, rL); mapCtx.stroke();
    mapCtx.shadowBlur = 0;
    // Direction arrow
    mapCtx.fillStyle = '#38bdf8';
    mapCtx.shadowColor = 'rgba(56,189,248,0.6)'; mapCtx.shadowBlur = 10;
    mapCtx.beginPath();
    const ar = Math.max(3, rL * 0.55);
    mapCtx.moveTo(0, rL - ar * 0.2); mapCtx.lineTo(-ar * 0.5, rL - ar); mapCtx.lineTo(ar * 0.5, rL - ar);
    mapCtx.closePath(); mapCtx.fill();
    mapCtx.shadowBlur = 0;
    mapCtx.restore();
  }

  // ── Scale bar with dual units ────────────────────────────────────────────────
  const pixelsPerM = cw / (meta.width * (meta.resolution || 0.05));
  mapCtx.save();
  // 1 m bar
  const bar1m = 1 * pixelsPerM;
  mapCtx.fillStyle = 'rgba(255,255,255,0.85)'; mapCtx.globalAlpha = 1;
  mapCtx.fillRect(12, ch - 24, bar1m, 3);
  mapCtx.fillStyle = 'rgba(255,255,255,0.70)';
  mapCtx.fillRect(12, ch - 19, bar1m * 0.5, 2);  // 50 cm half-bar
  mapCtx.font = '8px monospace'; mapCtx.textAlign = 'left'; mapCtx.fillStyle = 'rgba(255,255,255,0.75)';
  mapCtx.fillText('50cm', 12 + bar1m * 0.5 + 2, ch - 17);
  mapCtx.fillText('1 m',  12 + bar1m + 2,        ch - 22);
  // 5 m bar
  const bar5m = 5 * pixelsPerM;
  mapCtx.fillStyle = 'rgba(56,189,248,0.55)';
  mapCtx.fillRect(12, ch - 12, bar5m, 3);
  mapCtx.fillStyle = 'rgba(56,189,248,0.70)';
  mapCtx.fillText('5 m', 12 + bar5m + 2, ch - 10);
  // Grid info
  mapCtx.fillStyle = 'rgba(99,130,180,0.45)'; mapCtx.textAlign = 'right';
  mapCtx.fillText(`${(meta.resolution || 0.05)*100|0} cm/cell`, cw - 8, ch - 8);
  mapCtx.restore();
}

// ── Lidar render (v6 - HiDPI) ────────────────────────────────────────────────
async function refreshLidar() {
  try {
    const data = await api('/api/lidar');
    if (EL.radarCanvas) drawRadar(data);
    if (EL.sceneCanvas) drawScene(data);
    const hasRoute = data.obstacle_stop && data.avoidance_route?.length > 0;
    EL.avoidBadge.style.display = hasRoute ? '' : 'none';
  } catch (_) {}
}

function _initCanvas(canvas) {
  const outer = canvas.parentElement;
  const cssW = outer ? outer.clientWidth : canvas.width / DPR;
  const cssH = outer ? outer.clientHeight : canvas.height / DPR;
  if (!cssW || !cssH) return { cw: canvas.width, ch: canvas.height };
  const bw = Math.round(cssW * DPR);
  const bh = Math.round(cssH * DPR);
  if (canvas.width !== bw || canvas.height !== bh) {
    canvas.width = bw;
    canvas.height = bh;
    canvas.style.width = cssW + 'px';
    canvas.style.height = cssH + 'px';
  }
  return { cw: bw, ch: bh };
}

function drawRadar(data) {
  const { cw, ch } = _initCanvas(EL.radarCanvas);
  const cx = cw / 2, cy = ch / 2;
  const maxRange = data.max_range || 8.0;
  const scale = (Math.min(cw, ch) / 2 - 18 * DPR) / maxRange;

  radarCtx.clearRect(0, 0, cw, ch);

  const bgGrad = radarCtx.createRadialGradient(cx, cy, 0, cx, cy, Math.min(cw, ch) / 2);
  bgGrad.addColorStop(0, '#111c2e');
  bgGrad.addColorStop(1, '#0d1422');
  radarCtx.fillStyle = bgGrad;
  radarCtx.fillRect(0, 0, cw, ch);

  // Distance rings
  const rings = [
    { r: 0.5, lbl: '50cm', dash: [4 * DPR, 4 * DPR], col: 'rgba(251,191,36,0.25)', tc: 'rgba(251,191,36,0.80)' },
    { r: 1.0, lbl: '1 m', dash: [], col: 'rgba(120,150,200,0.30)', tc: 'rgba(140,175,230,0.80)' },
    { r: 1.5, lbl: '1.5m', dash: [], col: 'rgba(99,130,180,0.22)', tc: 'rgba(140,175,230,0.65)' },
    { r: 2, lbl: '2 m', dash: [], col: 'rgba(99,130,180,0.18)', tc: 'rgba(99,130,180,0.55)' },
    { r: 3, lbl: '3 m', dash: [], col: 'rgba(99,130,180,0.16)', tc: 'rgba(99,130,180,0.50)' },
    { r: 4, lbl: '4 m', dash: [], col: 'rgba(99,130,180,0.14)', tc: 'rgba(99,130,180,0.48)' },
    { r: 5, lbl: '5 m', dash: [], col: 'rgba(99,130,180,0.13)', tc: 'rgba(99,130,180,0.45)' },
    { r: 6, lbl: '6 m', dash: [], col: 'rgba(99,130,180,0.13)', tc: 'rgba(99,130,180,0.45)' },
    { r: 8, lbl: '8 m', dash: [], col: 'rgba(99,130,180,0.12)', tc: 'rgba(99,130,180,0.42)' },
  ];

  rings.forEach(({ r, lbl, dash, col, tc }) => {
    const rPx = r * scale;
    if (rPx > Math.min(cw, ch) / 2 - 4) return;
    radarCtx.setLineDash(dash);
    radarCtx.strokeStyle = col;
    radarCtx.lineWidth = DPR;
    radarCtx.beginPath();
    radarCtx.arc(cx, cy, rPx, 0, Math.PI * 2);
    radarCtx.stroke();
    radarCtx.setLineDash([]);
    radarCtx.fillStyle = tc;
    radarCtx.font = `${r < 2 ? 9 : 10}px monospace`;
    radarCtx.textAlign = 'left';
    radarCtx.fillText(lbl, cx + rPx + 3, cy - 3);
  });

  // N/E/S/W labels
  const dirs = [[-Math.PI / 2, 'N'], [0, 'E'], [Math.PI / 2, 'S'], [Math.PI, 'W']];
  dirs.forEach(([a, lbl]) => {
    const outer2 = Math.min(cw, ch) / 2 - 6;
    radarCtx.fillStyle = 'rgba(148,163,184,0.55)';
    radarCtx.font = `bold ${9 * DPR}px monospace`;
    radarCtx.textAlign = 'center';
    radarCtx.textBaseline = 'middle';
    radarCtx.fillText(lbl, cx + Math.cos(a) * outer2, cy + Math.sin(a) * outer2);
  });

  // Crosshairs
  radarCtx.strokeStyle = 'rgba(99,130,180,0.20)';
  radarCtx.lineWidth = DPR;
  radarCtx.beginPath();
  radarCtx.moveTo(cx, 4 * DPR);
  radarCtx.lineTo(cx, ch - 4 * DPR);
  radarCtx.moveTo(4 * DPR, cy);
  radarCtx.lineTo(cw - 4 * DPR, cy);
  radarCtx.stroke();

  // Avoidance route
  const route = data.avoidance_route;
  if (data.obstacle_stop && route?.length >= 2) {
    radarCtx.save();
    radarCtx.strokeStyle = '#fbbf24';
    radarCtx.lineWidth = 2.5 * DPR;
    radarCtx.setLineDash([7 * DPR, 4 * DPR]);
    radarCtx.shadowColor = 'rgba(251,191,36,0.6)';
    radarCtx.shadowBlur = 10;
    radarCtx.beginPath();
    radarCtx.moveTo(cx, cy);
    route.forEach(wp => radarCtx.lineTo(cx - wp.y * scale, cy - wp.x * scale));
    radarCtx.stroke();
    radarCtx.setLineDash([]);
    radarCtx.shadowBlur = 0;
    radarCtx.fillStyle = '#fbbf24';
    route.forEach((wp, i) => {
      const sx = cx - wp.y * scale, sy = cy - wp.x * scale;
      radarCtx.beginPath();
      radarCtx.arc(sx, sy, (i === route.length - 1 ? 5 : 3.5) * DPR, 0, Math.PI * 2);
      radarCtx.fill();
    });
    const side = route[0]?.side || (route[0]?.y > 0 ? 'left' : 'right');
    radarCtx.font = `bold ${11 * DPR}px sans-serif`;
    radarCtx.textAlign = 'center';
    radarCtx.fillText(`DETOUR ${side.toUpperCase()}`, cx, 18 * DPR);
    radarCtx.restore();
  }

  // Scan points with gradient color
  if (data.points?.length) {
    data.points.forEach(p => {
      const dist = Math.hypot(p.x, p.y);
      const sx = cx - p.y * scale;
      const sy = cy - p.x * scale;
      const isObs = data.obstacle_stop && Math.abs(Math.atan2(p.y, p.x)) < 0.4 && dist < 0.45;
      const t = Math.min(1, dist / maxRange);
      if (isObs) {
        radarCtx.fillStyle = `rgba(248,113,113,${0.85 - t * 0.3})`;
      } else {
        const g = Math.round(180 + (1 - t) * 31);
        const b = Math.round(80 + t * 73);
        radarCtx.fillStyle = `rgba(52,${g},${b},${0.85 - t * 0.25})`;
      }
      const ptR = Math.max(1.5 * DPR, (1 - t * 0.5) * 2.5 * DPR);
      radarCtx.fillRect(sx - ptR, sy - ptR, ptR * 2, ptR * 2);
    });
  }

  // Obstacle stop-zone
  if (data.obstacle_stop) {
    radarCtx.save();
    radarCtx.strokeStyle = 'rgba(248,113,113,0.55)';
    radarCtx.lineWidth = 1.5 * DPR;
    radarCtx.setLineDash([3 * DPR, 3 * DPR]);
    const stopR = 0.35 * scale;
    radarCtx.beginPath();
    radarCtx.arc(cx, cy, stopR, -Math.PI * 0.75, -Math.PI * 0.25);
    radarCtx.stroke();
    radarCtx.setLineDash([]);
    radarCtx.fillStyle = '#f87171';
    radarCtx.font = `bold ${9 * DPR}px sans-serif`;
    radarCtx.textAlign = 'center';
    radarCtx.fillText('BLOCKED', cx, cy - stopR - 5 * DPR);
    radarCtx.restore();
  }

  // Rover footprint
  const ROVER_L = 0.520, ROVER_W = 0.500, WHEEL_D = 0.120, WHEEL_TRK = 0.400, LIDAR_FWD = 0.240;
  const hL = ROVER_L / 2 * scale, hW = ROVER_W / 2 * scale;
  const wheelR = WHEEL_D / 2 * scale, wheelOff = WHEEL_TRK / 2 * scale;

  radarCtx.save();
  const rBodyGrad = radarCtx.createLinearGradient(cx - hW, cy - hL, cx + hW, cy + hL);
  rBodyGrad.addColorStop(0, 'rgba(56,189,248,0.10)');
  rBodyGrad.addColorStop(1, 'rgba(56,189,248,0.04)');
  radarCtx.fillStyle = rBodyGrad;
  radarCtx.fillRect(cx - hW, cy - hL, hW * 2, hL * 2);
  radarCtx.strokeStyle = 'rgba(56,189,248,0.55)';
  radarCtx.lineWidth = 1.5 * DPR;
  radarCtx.strokeRect(cx - hW, cy - hL, hW * 2, hL * 2);
  radarCtx.strokeStyle = '#34d399';
  radarCtx.lineWidth = 2.5 * DPR;
  radarCtx.shadowColor = 'rgba(52,211,153,0.50)';
  radarCtx.shadowBlur = 6;
  radarCtx.beginPath();
  radarCtx.moveTo(cx - hW, cy - hL);
  radarCtx.lineTo(cx + hW, cy - hL);
  radarCtx.stroke();
  radarCtx.shadowBlur = 0;

  [['L', cx - wheelOff], ['R', cx + wheelOff]].forEach(([lbl, wx]) => {
    radarCtx.fillStyle = 'rgba(100,116,139,0.22)';
    radarCtx.strokeStyle = 'rgba(148,163,184,0.75)';
    radarCtx.lineWidth = 1.5 * DPR;
    radarCtx.beginPath();
    radarCtx.arc(wx, cy, wheelR, 0, Math.PI * 2);
    radarCtx.fill();
    radarCtx.stroke();
    radarCtx.fillStyle = 'rgba(148,163,184,0.60)';
    radarCtx.font = `${6 * DPR}px sans-serif`;
    radarCtx.textAlign = 'center';
    radarCtx.fillText(lbl, wx, cy + 2 * DPR);
  });

  radarCtx.strokeStyle = 'rgba(148,163,184,0.25)';
  radarCtx.lineWidth = DPR;
  radarCtx.setLineDash([2 * DPR, 3 * DPR]);
  radarCtx.beginPath();
  radarCtx.moveTo(cx - wheelOff, cy);
  radarCtx.lineTo(cx + wheelOff, cy);
  radarCtx.stroke();
  radarCtx.setLineDash([]);

  const lidarSY = cy - LIDAR_FWD * scale;
  radarCtx.fillStyle = '#fbbf24';
  radarCtx.shadowColor = '#fbbf24';
  radarCtx.shadowBlur = 10;
  radarCtx.beginPath();
  radarCtx.arc(cx, lidarSY, 4 * DPR, 0, Math.PI * 2);
  radarCtx.fill();
  radarCtx.shadowBlur = 0;
  radarCtx.fillStyle = 'rgba(251,191,36,0.70)';
  radarCtx.font = `${7 * DPR}px monospace`;
  radarCtx.textAlign = 'left';
  radarCtx.fillText('LiDAR', cx + 6 * DPR, lidarSY - 4 * DPR);

  radarCtx.fillStyle = 'rgba(56,189,248,0.55)';
  radarCtx.font = `${7 * DPR}px monospace`;
  radarCtx.textAlign = 'center';
  radarCtx.fillText(`◀${Math.round(ROVER_W * 100)}cm▶`, cx, cy + hL + 11 * DPR);
  radarCtx.textAlign = 'right';
  radarCtx.fillText(`${Math.round(ROVER_L * 100)}cm`, cx - hW - 3 * DPR, cy);
  radarCtx.fillStyle = '#34d399';
  radarCtx.font = `bold ${7 * DPR}px sans-serif`;
  radarCtx.textAlign = 'center';
  radarCtx.fillText('▲ FRONT', cx, cy - hL - 6 * DPR);
  radarCtx.restore();

  radarCtx.fillStyle = '#38bdf8';
  radarCtx.shadowColor = '#38bdf8';
  radarCtx.shadowBlur = 8;
  radarCtx.beginPath();
  radarCtx.arc(cx, cy, 2.5 * DPR, 0, Math.PI * 2);
  radarCtx.fill();
  radarCtx.shadowBlur = 0;

  radarCtx.fillStyle = 'rgba(99,130,180,0.50)';
  radarCtx.font = `${8 * DPR}px sans-serif`;
  radarCtx.textAlign = 'left';
  radarCtx.fillText('▲ FWD', cx - 12 * DPR, 11 * DPR);

  radarCtx.fillStyle = 'rgba(99,130,180,0.45)';
  radarCtx.font = `${8 * DPR}px monospace`;
  radarCtx.textAlign = 'right';
  radarCtx.fillText(`${data.points?.length ?? 0} pts`, cw - 6 * DPR, ch - 6 * DPR);
}

function drawScene(data) {
  const { cw, ch } = _initCanvas(EL.sceneCanvas);
  const maxRange = data.max_range || 8.0;
  sceneCtx.clearRect(0, 0, cw, ch);

  const grad = sceneCtx.createLinearGradient(0, 0, 0, ch);
  grad.addColorStop(0, '#0d1b2e');
  grad.addColorStop(1, '#1a2a1a');
  sceneCtx.fillStyle = grad;
  sceneCtx.fillRect(0, 0, cw, ch);

  const horizon = ch * 0.55;
  sceneCtx.strokeStyle = 'rgba(56,189,248,0.15)';
  sceneCtx.lineWidth = DPR;
  sceneCtx.beginPath();
  sceneCtx.moveTo(0, horizon);
  sceneCtx.lineTo(cw, horizon);
  sceneCtx.stroke();

  sceneCtx.strokeStyle = 'rgba(56,189,248,0.06)';
  for (let i = 0; i < 20; i++) {
    const y = horizon + (i / 20) * (ch - horizon);
    sceneCtx.beginPath();
    sceneCtx.moveTo(0, y);
    sceneCtx.lineTo(cw, y);
    sceneCtx.stroke();
  }

  if (data.points?.length) {
    data.points.forEach(p => {
      const dist = Math.hypot(p.x, p.y);
      if (dist < 0.05 || dist > maxRange) return;
      const angle = Math.atan2(p.y, p.x);
      const normAngle = angle / (Math.PI / 2);
      const screenX = cw * (0.5 - normAngle * 0.55);
      const relDist = dist / maxRange;
      const screenY = horizon - (1 - relDist) * horizon * 0.9;
      const dotSize = Math.max(1.5 * DPR, (1 - relDist) * 8 * DPR);
      const isObs = data.obstacle_stop && Math.abs(angle) < 0.44 && dist < 0.45;
      if (isObs) {
        sceneCtx.fillStyle = '#f87171';
      } else {
        const g = Math.round(180 + (1 - relDist) * 31);
        sceneCtx.fillStyle = `rgba(52,${g},153,${0.45 + relDist * 0.5})`;
      }
      sceneCtx.globalAlpha = 0.55 + relDist * 0.45;
      sceneCtx.fillRect(screenX - dotSize / 2, screenY - dotSize / 2, dotSize, dotSize);
    });
    sceneCtx.globalAlpha = 1;
  }

  if (data.obstacle_stop && data.avoidance_route?.length) {
    const side = data.avoidance_route[0]?.side || (data.avoidance_route[0]?.y > 0 ? 'left' : 'right');
    const arrow = side === 'left' ? '← DETOUR LEFT' : 'DETOUR RIGHT →';
    sceneCtx.fillStyle = '#fbbf24';
    sceneCtx.font = `bold ${14 * DPR}px sans-serif`;
    sceneCtx.globalAlpha = 0.90;
    sceneCtx.textAlign = 'center';
    sceneCtx.fillText(arrow, cw / 2, horizon - 20 * DPR);
    sceneCtx.globalAlpha = 1;
  }
}

// ── Poll loops (sequential debounce — no overlapping fetches) ─────────────────
function seqLoop(fn, ms) {
  async function run() {
    try { await fn(); } catch (_) {}
    setTimeout(run, ms);
  }
  run();
}

seqLoop(refreshStatus,       450);
seqLoop(refreshMap,          300);   // v6: increased from 950ms to 300ms for responsive updates
seqLoop(refreshLidar,        260);
seqLoop(refreshCameraStatus, 2000);
seqLoop(refreshPois,         5000);
refreshMapList();
setInterval(refreshMapList, 10000);

// ── ResizeObserver for responsive canvas sizing ───────────────────────────────
if (window.ResizeObserver) {
  let _resizeTimer = null;
  new ResizeObserver(() => {
    clearTimeout(_resizeTimer);
    _resizeTimer = setTimeout(() => {
      if (STATE.mapMeta) drawMap(STATE.mapMeta);
      _updateMapTransform();
    }, 100);
  }).observe(EL.mapOuter);
}

// Initialise map canvas cursor and tool hint
setTool('goal');

// ── Live Tuning panel ─────────────────────────────────────────────────────────
const EL_tuningRows     = document.getElementById('tuning-rows');
const EL_tuningFeedback = document.getElementById('tuning-feedback');
const EL_tuningReset    = document.getElementById('tuningResetBtn');

let _tuningSettings = {};   // cache: key → {value, min, max, label, unit, default}

async function loadTuningSettings() {
  try {
    const data = await api('/api/settings');
    _tuningSettings = data.settings || {};
    renderTuningRows();
  } catch (e) {
    if (EL_tuningRows) EL_tuningRows.innerHTML = `<div class="hint">Error: ${e.message}</div>`;
  }
}

function renderTuningRows() {
  if (!EL_tuningRows) return;
  EL_tuningRows.innerHTML = '';
  Object.entries(_tuningSettings).forEach(([key, spec]) => {
    const row = document.createElement('div');
    row.className = 'tuning-row';

    const lbl = document.createElement('div');
    lbl.className = 'tuning-label';
    lbl.title = spec.description || '';
    lbl.textContent = spec.label;

    const unit = document.createElement('span');
    unit.className = 'tuning-unit';
    unit.textContent = spec.unit ? ` ${spec.unit}` : '';

    const inp = document.createElement('input');
    inp.type      = 'number';
    inp.className = 'tuning-input';
    inp.min       = spec.min;
    inp.max       = spec.max;
    inp.step      = spec.unit === 'm' || spec.unit === 'm/s' ? '0.01'
                  : spec.unit === 's' ? '0.5'
                  : '0.05';
    inp.value     = Number(spec.value).toFixed(
      spec.unit === 's' ? 1 : 2
    );
    inp.dataset.key     = key;
    inp.dataset.default = spec.default;

    // Changed indicator dot
    const dot = document.createElement('span');
    dot.className = 'tuning-dot';
    dot.style.visibility = Math.abs(spec.value - spec.default) > 0.0001 ? 'visible' : 'hidden';
    dot.title = `Default: ${spec.default} ${spec.unit}`;

    // Apply on Enter or blur
    const applyFn = async () => {
      const val = parseFloat(inp.value);
      if (isNaN(val)) return;
      try {
        const r = await api('/api/settings', 'PATCH', { key, value: val });
        if (r.ok) {
          _tuningSettings[key].value = r.value;
          inp.value = Number(r.value).toFixed(inp.step.includes('0.5') ? 1 : 2);
          dot.style.visibility = Math.abs(r.value - spec.default) > 0.0001 ? 'visible' : 'hidden';
          _flashFeedback(`✓ ${spec.label} = ${r.value} ${spec.unit}`, 'ok');
        }
      } catch (err) {
        _flashFeedback(`✗ ${err.message}`, 'err');
        inp.value = Number(_tuningSettings[key].value).toFixed(inp.step.includes('0.5') ? 1 : 2);
      }
    };
    inp.addEventListener('keydown', e => { if (e.key === 'Enter') { e.preventDefault(); applyFn(); } });
    inp.addEventListener('blur', applyFn);

    const ctrl = document.createElement('div');
    ctrl.className = 'tuning-ctrl';
    ctrl.appendChild(inp);
    ctrl.appendChild(unit);
    ctrl.appendChild(dot);

    row.appendChild(lbl);
    row.appendChild(ctrl);
    EL_tuningRows.appendChild(row);
  });
}

function _flashFeedback(msg, type) {
  if (!EL_tuningFeedback) return;
  EL_tuningFeedback.textContent = msg;
  EL_tuningFeedback.className = `tuning-feedback ${type}`;
  clearTimeout(EL_tuningFeedback._t);
  EL_tuningFeedback._t = setTimeout(() => {
    EL_tuningFeedback.textContent = '';
    EL_tuningFeedback.className = 'tuning-feedback';
  }, 3000);
}

EL_tuningReset && EL_tuningReset.addEventListener('click', async () => {
  if (!confirm('Reset all tuning values to defaults?')) return;
  try {
    const r = await api('/api/settings', 'DELETE');
    if (r.ok) { _tuningSettings = r.settings; renderTuningRows(); _flashFeedback('✓ Reset to defaults', 'ok'); }
  } catch (e) { _flashFeedback(`✗ ${e.message}`, 'err'); }
});

// Load tuning on startup
loadTuningSettings();
