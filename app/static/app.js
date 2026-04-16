/**
 * Arespath Rover — Dashboard app.js  v4
 *
 * New in v4
 * ─────────
 * • 10 000 sq.ft map (48 m × 48 m grid, 960 × 960 cells)
 * • POI / Gazebo waypoints — add, move, label, navigate-to, delete
 * • Map pan & zoom — scroll wheel + drag-to-pan (Pan tool) + zoom buttons
 * • LiDAR ICP localisation status pill
 * • Mapping + pilot control glitch fix — map worker thread decoupled in backend;
 *   frontend uses independent debounced loops that never block each other
 * • All poll loops use a sequential debounce pattern (next tick starts only
 *   after previous await resolves) → no overlapping fetches
 */

'use strict';

// ── State ─────────────────────────────────────────────────────────────────────
const STATE = {
  armed:       false,
  mode:        'pilot',
  wsAlive:     false,
  mapTool:     'goal',   // 'goal' | 'start' | 'poi' | 'pan'
  mapMeta:     null,
  dragStart:   null,
  headingPrev: 0,
  activeCmd:   null,
  snapping:    false,
  // Map viewport (pan + zoom)
  mapZoom:     1.0,
  mapPanX:     0,      // canvas-pixel offset
  mapPanY:     0,
  mapDragging: false,
  mapDragLast: null,
  // POI pending placement
  pendingPoi:  null,   // {x, y} world coords awaiting form submit
  // Goal pending placement (placed on map, not yet sent to backend)
  pendingGoal: null,   // {x, y} world coords
  mapCentered: false,  // auto-center on rover
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
  startMissionBtn: el('startMissionBtn'),
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
const sceneCtx = EL.sceneCanvas.getContext('2d');

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
EL.mapResetBtn .addEventListener('click', () => { api('/api/map/reset', 'POST', {}).catch(() => {}); STATE.mapCentered = false; STATE.pendingGoal = null; });
EL.saveMapBtn  .addEventListener('click', () =>
  api('/api/map/save', 'POST', { name: EL.mapName.value || 'map' }).then(refreshMapList).catch(() => {}));
EL.loadMapBtn  .addEventListener('click', () => {
  const name = EL.mapSelect.value;
  if (name) api('/api/map/load', 'POST', { name }).catch(() => {});
});
EL.startMissionBtn.addEventListener('click', () => {
  if (!STATE.pendingGoal) { alert('Place a goal on the map first.'); return; }
  api('/api/navigate/goal', 'POST', { x: STATE.pendingGoal.x, y: STATE.pendingGoal.y }).catch(() => {});
});
EL.cancelNavBtn.addEventListener('click', () => { api('/api/navigate/cancel', 'POST', {}).catch(() => {}); STATE.pendingGoal = null; });

// ── Map zoom / pan buttons ────────────────────────────────────────────────────
EL.mapZoomInBtn   .addEventListener('click', () => applyZoom(1.15, null));
EL.mapZoomOutBtn  .addEventListener('click', () => applyZoom(0.87, null));
EL.mapZoomResetBtn.addEventListener('click', () => {
  STATE.mapZoom = 1;
  STATE.mapPanX = 0; STATE.mapPanY = 0;
  if (EL.zoomLevel) EL.zoomLevel.textContent = '1.0×';
});

function applyZoom(factor, clientX, clientY) {
  const dpr = window.devicePixelRatio || 1;
  const cw = EL.mapCanvas.width / dpr, ch = EL.mapCanvas.height / dpr;
  const rect = EL.mapCanvas.getBoundingClientRect();

  let mx = cw / 2, my = ch / 2;
  if (clientX !== null && clientY !== null) {
    mx = clientX - rect.left;
    my = clientY - rect.top;
  }

  const worldX = (mx - STATE.mapPanX / dpr) / STATE.mapZoom;
  const worldY = (my - STATE.mapPanY / dpr) / STATE.mapZoom;

  STATE.mapZoom = Math.max(0.3, Math.min(12, STATE.mapZoom * factor));

  STATE.mapPanX = (worldX * STATE.mapZoom - mx) * dpr;
  STATE.mapPanY = (worldY * STATE.mapZoom - my) * dpr;

  if (EL.zoomLevel) EL.zoomLevel.textContent = `${STATE.mapZoom.toFixed(1)}×`;
}

// ── Scroll-wheel zoom ─────────────────────────────────────────────────────────
EL.mapOuter && EL.mapOuter.addEventListener('wheel', e => {
  e.preventDefault();
  applyZoom(e.deltaY < 0 ? 1.10 : 0.91, e.clientX, e.clientY);
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
  STATE.dragStart = canvasToWorld(e, STATE.mapMeta);
});

EL.mapCanvas.addEventListener('pointermove', e => {
  if (STATE.mapTool === 'pan' && STATE.mapDragging && STATE.mapDragLast) {
    const dpr = window.devicePixelRatio || 1;
    STATE.mapPanX += (e.clientX - STATE.mapDragLast.x) * dpr;
    STATE.mapPanY += (e.clientY - STATE.mapDragLast.y) * dpr;
    STATE.mapDragLast = { x: e.clientX, y: e.clientY };
    return;
  }
  if (!STATE.dragStart || !STATE.mapMeta || STATE.mapTool !== 'start') return;
  const cur = canvasToWorld(e, STATE.mapMeta);
  STATE.headingPrev = Math.atan2(cur.y - STATE.dragStart.y, cur.x - STATE.dragStart.x);
});

EL.mapCanvas.addEventListener('pointerup', e => {
  if (STATE.mapTool === 'pan') {
    STATE.mapDragging = false;
    STATE.mapDragLast = null;
    EL.mapCanvas.style.cursor = 'grab';
    return;
  }
  if (!STATE.mapMeta) return;
  const pt = canvasToWorld(e, STATE.mapMeta);
  if (STATE.mapTool === 'goal') {
    STATE.pendingGoal = { x: pt.x, y: pt.y };
  } else if (STATE.mapTool === 'start') {
    const start = STATE.dragStart || pt;
    api('/api/pose', 'POST', { x: start.x, y: start.y, theta: STATE.headingPrev }).catch(() => {});
  } else if (STATE.mapTool === 'poi') {
    _openPoiAdd(pt.x, pt.y);
  }
  STATE.dragStart = null;
});

EL.mapCanvas.addEventListener('pointercancel', () => {
  STATE.mapDragging = false; STATE.mapDragLast = null;
});

function canvasToWorld(e, meta) {
  const dpr = window.devicePixelRatio || 1;
  const rect = EL.mapCanvas.getBoundingClientRect();
  const cx = (e.clientX - rect.left) * dpr;
  const cy = (e.clientY - rect.top)  * dpr;
  const worldX = (cx - STATE.mapPanX) / STATE.mapZoom;
  const worldY = (cy - STATE.mapPanY) / STATE.mapZoom;
  return {
    x: worldX * meta.resolution + meta.origin[0],
    y: worldY * meta.resolution + meta.origin[1],
  };
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
    // Bind buttons
    EL.poiSidebarList.querySelectorAll('.poi-nav-btn').forEach(btn => {
      btn.addEventListener('click', () => api(`/api/poi/${btn.dataset.id}/navigate`, 'POST', {}).catch(err => alert(err.message)));
    });
    EL.poiSidebarList.querySelectorAll('.poi-edit-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        const poi = pois.find(p => p.id === btn.dataset.id);
        if (poi) _openPoiEdit(poi);
      });
    });
    // Store pois for map canvas
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

// ── Map canvas render ─────────────────────────────────────────────────────────
function _fitCanvas() {
  const outer = EL.mapOuter;
  if (!outer) return;
  const w = outer.clientWidth  || 800;
  const h = outer.clientHeight || 800;
  const dpr = window.devicePixelRatio || 1;
  EL.mapCanvas.width  = Math.floor(w * dpr);
  EL.mapCanvas.height = Math.floor(h * dpr);
  EL.mapCanvas.style.width  = w + 'px';
  EL.mapCanvas.style.height = h + 'px';
}

async function refreshMap() {
  _fitCanvas();
  try {
    const meta = await api('/api/map/data');
    STATE.mapMeta = meta;

    if (!STATE.mapCentered && meta.pose) {
      const dpr = window.devicePixelRatio || 1;
      const cw = EL.mapCanvas.width / dpr, ch = EL.mapCanvas.height / dpr;
      const z = STATE.mapZoom;
      const visibleW = cw / z, visibleH = ch / z;
      STATE.mapPanX = (meta.pose.x - visibleW / 2) * dpr;
      STATE.mapPanY = (meta.pose.y - visibleH / 2) * dpr;
      STATE.mapCentered = true;
    }

    drawMap(meta);
  } catch (_) {}
}

async function drawMap(meta) {
  const dpr = window.devicePixelRatio || 1;
  const cw  = EL.mapCanvas.width  / dpr;
  const ch  = EL.mapCanvas.height / dpr;
  const z   = STATE.mapZoom;
  const panX = STATE.mapPanX / dpr;
  const panY = STATE.mapPanY / dpr;

  mapCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
  mapCtx.clearRect(0, 0, cw, ch);
  mapCtx.fillStyle = '#0a0f1e';
  mapCtx.fillRect(0, 0, cw, ch);

  mapCtx.save();
  mapCtx.scale(z, z);
  mapCtx.translate(-panX, -panY);

  mapCtx.beginPath();
  mapCtx.rect(panX, panY, cw / z, ch / z);
  mapCtx.clip();

  if (meta.image_png_b64) {
    const img = new Image();
    await new Promise(res => { img.onload = res; img.src = 'data:image/png;base64,' + meta.image_png_b64; });
    mapCtx.drawImage(img, 0, 0);
  } else {
    mapCtx.fillStyle = '#0a0f1e';
    mapCtx.fillRect(0, 0, meta.width, meta.height);
  }

  const toC = p => ({ x: p.x * z + panX, y: p.y * z + panY });

  const step = Math.max(5, Math.round(1.0 / meta.resolution));
  mapCtx.strokeStyle = 'rgba(255,255,255,0.05)'; mapCtx.lineWidth = 1 / z;
  for (let x = 0; x <= meta.width; x += step) {
    mapCtx.beginPath(); mapCtx.moveTo(x, 0); mapCtx.lineTo(x, meta.height); mapCtx.stroke();
  }
  for (let y = 0; y <= meta.height; y += step) {
    mapCtx.beginPath(); mapCtx.moveTo(0, y); mapCtx.lineTo(meta.width, y); mapCtx.stroke();
  }

  if (meta.scan?.length) {
    mapCtx.fillStyle = 'rgba(52,211,153,0.7)';
    const dot = 1.5 / z;
    meta.scan.forEach(p => {
      mapCtx.fillRect(p.x - dot/2, p.y - dot/2, dot, dot);
    });
  }

  if (meta.path?.length > 1) {
    mapCtx.save();
    mapCtx.strokeStyle = '#38bdf8'; mapCtx.lineWidth = 2 / z;
    mapCtx.shadowColor = 'rgba(56,189,248,0.5)'; mapCtx.shadowBlur = 6 / z;
    mapCtx.beginPath();
    meta.path.forEach((p, i) => { if (i === 0) mapCtx.moveTo(p.x, p.y); else mapCtx.lineTo(p.x, p.y); });
    mapCtx.stroke(); mapCtx.restore();
  }

  if (STATE.pendingGoal && STATE.mapMeta) {
    const res = meta.resolution || 0.02;
    const ox = meta.origin[0], oy = meta.origin[1];
    const gx = (STATE.pendingGoal.x - ox) / res;
    const gy = (STATE.pendingGoal.y - oy) / res;
    const c = toC({ x: gx, y: gy });
    mapCtx.save();
    mapCtx.strokeStyle = '#60a5fa'; mapCtx.lineWidth = 1.5 / z;
    mapCtx.setLineDash([3 / z, 2 / z]);
    mapCtx.beginPath(); mapCtx.arc(c.x, c.y, 3 / z, 0, Math.PI * 2); mapCtx.stroke();
    mapCtx.setLineDash([]);
    mapCtx.beginPath();
    mapCtx.moveTo(c.x, c.y - 8 / z); mapCtx.lineTo(c.x, c.y + 8 / z);
    mapCtx.moveTo(c.x - 8 / z, c.y); mapCtx.lineTo(c.x + 8 / z, c.y);
    mapCtx.stroke(); mapCtx.restore();
  }

  if (meta.goal) {
    const c = toC(meta.goal);
    mapCtx.save();
    mapCtx.strokeStyle = '#fbbf24'; mapCtx.lineWidth = 2 / z;
    mapCtx.shadowColor = '#fbbf24'; mapCtx.shadowBlur = 6 / z;
    mapCtx.beginPath(); mapCtx.arc(c.x, c.y, 3.5 / z, 0, Math.PI * 2); mapCtx.stroke();
    mapCtx.beginPath();
    mapCtx.moveTo(c.x, c.y - 10 / z); mapCtx.lineTo(c.x, c.y + 10 / z);
    mapCtx.moveTo(c.x - 10 / z, c.y); mapCtx.lineTo(c.x + 10 / z, c.y);
    mapCtx.stroke(); mapCtx.restore();
  }

  const pois = meta.pois || [];
  pois.forEach(poi => {
    const c = toC({ x: poi.px, y: poi.py });
    const r = 8 / z;
    mapCtx.save();
    mapCtx.fillStyle = 'rgba(251,191,36,0.2)';
    mapCtx.beginPath(); mapCtx.arc(c.x, c.y, r * 1.6, 0, Math.PI*2); mapCtx.fill();
    mapCtx.strokeStyle = '#fbbf24'; mapCtx.lineWidth = 1.5 / z;
    mapCtx.shadowColor = '#fbbf24'; mapCtx.shadowBlur = 4 / z;
    mapCtx.beginPath(); mapCtx.arc(c.x, c.y, r, 0, Math.PI*2); mapCtx.stroke();
    mapCtx.restore();
    mapCtx.save();
    mapCtx.font = `${11 / z}px sans-serif`;
    mapCtx.textAlign = 'center'; mapCtx.textBaseline = 'top';
    mapCtx.fillStyle = '#fbbf24';
    mapCtx.shadowColor = '#000'; mapCtx.shadowBlur = 2 / z;
    mapCtx.fillText(poi.label, c.x, c.y + r + 1 / z);
    mapCtx.restore();
  });

  if (meta.pose) {
    const c   = toC(meta.pose);
    const th  = meta.pose.theta;
    const ppm = 1 / meta.resolution;
    const rL  = 0.520 / 2 * ppm;
    const rW  = 0.500 / 2 * ppm;
    mapCtx.save();
    mapCtx.translate(c.x, c.y);
    mapCtx.rotate(th);
    mapCtx.fillStyle = 'rgba(56,189,248,0.15)';
    mapCtx.fillRect(-rW, -rL, rW * 2, rL * 2);
    mapCtx.strokeStyle = 'rgba(56,189,248,0.6)'; mapCtx.lineWidth = 1.5 / z;
    mapCtx.strokeRect(-rW, -rL, rW * 2, rL * 2);
    mapCtx.strokeStyle = '#34d399'; mapCtx.lineWidth = 2.5 / z;
    mapCtx.shadowColor = 'rgba(52,211,153,0.6)'; mapCtx.shadowBlur = 6 / z;
    mapCtx.beginPath(); mapCtx.moveTo(-rW, rL); mapCtx.lineTo(rW, rL); mapCtx.stroke();
    mapCtx.shadowBlur = 0;
    mapCtx.fillStyle = '#38bdf8';
    mapCtx.shadowColor = 'rgba(56,189,248,0.6)'; mapCtx.shadowBlur = 8 / z;
    const ar = Math.max(3 / z, rL * 0.5);
    mapCtx.beginPath();
    mapCtx.moveTo(0, rL - ar * 0.2); mapCtx.lineTo(-ar * 0.5, rL - ar); mapCtx.lineTo(ar * 0.5, rL - ar);
    mapCtx.closePath(); mapCtx.fill();
    mapCtx.restore();
  }

  mapCtx.restore();

  const vw = cw / z, vh = ch / z;
  const pixelsPerM_screen = vw / (meta.width * meta.resolution);
  const bar1m_s = pixelsPerM_screen;
  mapCtx.save();
  mapCtx.fillStyle = 'rgba(255,255,255,0.85)';
  mapCtx.fillRect(12, ch - 22, bar1m_s, 3);
  mapCtx.font = '8px monospace'; mapCtx.textAlign = 'left'; mapCtx.fillStyle = 'rgba(255,255,255,0.75)';
  mapCtx.fillText('1 m', 12 + bar1m_s + 3, ch - 18);
  const bar5m_s = 5 * pixelsPerM_screen;
  mapCtx.fillStyle = 'rgba(56,189,248,0.55)';
  mapCtx.fillRect(12, ch - 12, bar5m_s, 3);
  mapCtx.fillStyle = 'rgba(56,189,248,0.7)';
  mapCtx.fillText('5 m', 12 + bar5m_s + 3, ch - 10);
  mapCtx.textAlign = 'right'; mapCtx.fillStyle = 'rgba(99,130,180,0.5)';
  mapCtx.fillText(`${(meta.resolution * 100)|0} cm/px`, cw - 6, ch - 6);
  mapCtx.restore();
}
  } else {
    mapCtx.fillStyle = '#0f1729';
    mapCtx.fillRect(panX, panY, vw, vh);
  }

  const toC = p => ({
    x: p.x * scale + panX,
    y: p.y * scale + panY
  });

  const step = Math.max(5, Math.round(1.0 / meta.resolution));
  mapCtx.save();
  mapCtx.strokeStyle = 'rgba(255,255,255,0.05)'; mapCtx.lineWidth = 1 / scale;
  for (let x = 0; x <= meta.width; x += step) {
    const px = x + panX;
    mapCtx.beginPath(); mapCtx.moveTo(px, panY); mapCtx.lineTo(px, panY + vh); mapCtx.stroke();
  }
  for (let y = 0; y <= meta.height; y += step) {
    const py = y + panY;
    mapCtx.beginPath(); mapCtx.moveTo(panX, py); mapCtx.lineTo(panX + vw, py); mapCtx.stroke();
  }
  mapCtx.restore();

  if (meta.scan?.length) {
    mapCtx.fillStyle = 'rgba(52,211,153,0.7)';
    const dot = 1.5 / scale;
    meta.scan.forEach(p => {
      const c = toC(p);
      mapCtx.fillRect(c.x - dot/2, c.y - dot/2, dot, dot);
    });
  }

  if (meta.path?.length > 1) {
    mapCtx.save();
    mapCtx.strokeStyle = '#38bdf8'; mapCtx.lineWidth = 2 / scale;
    mapCtx.shadowColor = 'rgba(56,189,248,0.5)'; mapCtx.shadowBlur = 6 / scale;
    mapCtx.beginPath();
    meta.path.forEach((p, i) => { const c = toC(p); if (i === 0) mapCtx.moveTo(c.x, c.y); else mapCtx.lineTo(c.x, c.y); });
    mapCtx.stroke(); mapCtx.restore();
  }

  if (STATE.pendingGoal && STATE.mapMeta) {
    const c = toC(STATE.pendingGoal);
    mapCtx.save();
    mapCtx.strokeStyle = '#60a5fa'; mapCtx.lineWidth = 1.5 / scale;
    mapCtx.setLineDash([3 / scale, 2 / scale]);
    mapCtx.beginPath(); mapCtx.arc(c.x, c.y, 3 / scale, 0, Math.PI * 2); mapCtx.stroke();
    mapCtx.setLineDash([]);
    mapCtx.beginPath();
    mapCtx.moveTo(c.x, c.y - 8 / scale); mapCtx.lineTo(c.x, c.y + 8 / scale);
    mapCtx.moveTo(c.x - 8 / scale, c.y); mapCtx.lineTo(c.x + 8 / scale, c.y);
    mapCtx.stroke(); mapCtx.restore();
  }

  if (meta.goal) {
    const c = toC(meta.goal);
    mapCtx.save();
    mapCtx.strokeStyle = '#fbbf24'; mapCtx.lineWidth = 2 / scale;
    mapCtx.shadowColor = '#fbbf24'; mapCtx.shadowBlur = 6 / scale;
    mapCtx.beginPath(); mapCtx.arc(c.x, c.y, 3.5 / scale, 0, Math.PI * 2); mapCtx.stroke();
    mapCtx.beginPath();
    mapCtx.moveTo(c.x, c.y - 10 / scale); mapCtx.lineTo(c.x, c.y + 10 / scale);
    mapCtx.moveTo(c.x - 10 / scale, c.y); mapCtx.lineTo(c.x + 10 / scale, c.y);
    mapCtx.stroke(); mapCtx.restore();
  }

  // POIs
  const pois = meta.pois || [];
  pois.forEach(poi => {
    const c = toC({ x: poi.px, y: poi.py });
    const r = 8 / scale;
    mapCtx.save();
    mapCtx.fillStyle = 'rgba(251,191,36,0.2)';
    mapCtx.beginPath(); mapCtx.arc(c.x, c.y, r * 1.6, 0, Math.PI*2); mapCtx.fill();
    mapCtx.strokeStyle = '#fbbf24'; mapCtx.lineWidth = 1.5 / scale;
    mapCtx.shadowColor = '#fbbf24'; mapCtx.shadowBlur = 4 / scale;
    mapCtx.beginPath(); mapCtx.arc(c.x, c.y, r, 0, Math.PI*2); mapCtx.stroke();
    mapCtx.restore();
    mapCtx.save();
    mapCtx.font = `${11 / scale}px sans-serif`;
    mapCtx.textAlign = 'center'; mapCtx.textBaseline = 'top';
    mapCtx.fillStyle = '#fbbf24';
    mapCtx.shadowColor = '#000'; mapCtx.shadowBlur = 2 / scale;
    mapCtx.fillText(poi.label, c.x, c.y + r + 1 / scale);
    mapCtx.restore();
  });

  // Robot — scaled rectangle (520×500 mm)
  if (meta.pose) {
    const c   = toC(meta.pose);
    const th  = meta.pose.theta;
    const ppm = 1 / meta.resolution;  // map pixels per metre
    const rL  = 0.520 / 2 * ppm;   // half-length in map pixels
    const rW  = 0.500 / 2 * ppm;   // half-width  in map pixels
    mapCtx.save();
    mapCtx.translate(c.x, c.y);
    mapCtx.rotate(th);
    mapCtx.fillStyle = 'rgba(56,189,248,0.15)';
    mapCtx.fillRect(-rW, -rL, rW * 2, rL * 2);
    mapCtx.strokeStyle = 'rgba(56,189,248,0.6)'; mapCtx.lineWidth = 1.5 / scale;
    mapCtx.strokeRect(-rW, -rL, rW * 2, rL * 2);
    mapCtx.strokeStyle = '#34d399'; mapCtx.lineWidth = 2.5 / scale;
    mapCtx.shadowColor = 'rgba(52,211,153,0.6)'; mapCtx.shadowBlur = 6 / scale;
    mapCtx.beginPath(); mapCtx.moveTo(-rW, rL); mapCtx.lineTo(rW, rL); mapCtx.stroke();
    mapCtx.shadowBlur = 0;
    mapCtx.fillStyle = '#38bdf8';
    mapCtx.shadowColor = 'rgba(56,189,248,0.6)'; mapCtx.shadowBlur = 8 / scale;
    const ar = Math.max(3 / scale, rL * 0.5);
    mapCtx.beginPath();
    mapCtx.moveTo(0, rL - ar * 0.2); mapCtx.lineTo(-ar * 0.5, rL - ar); mapCtx.lineTo(ar * 0.5, rL - ar);
    mapCtx.closePath(); mapCtx.fill();
    mapCtx.restore();
  }

  mapCtx.restore(); // end world-space rendering

  // HUD: scale bar (screen space, bottom-left)
  const pixelsPerM_screen = vw / (meta.width * meta.resolution) * scale;
  const bar1m_s = pixelsPerM_screen;
  mapCtx.save();
  mapCtx.fillStyle = 'rgba(255,255,255,0.85)';
  mapCtx.fillRect(12, ch - 22, bar1m_s, 3);
  mapCtx.font = '8px monospace'; mapCtx.textAlign = 'left'; mapCtx.fillStyle = 'rgba(255,255,255,0.75)';
  mapCtx.fillText('1 m', 12 + bar1m_s + 3, ch - 18);
  const bar5m_s = 5 * pixelsPerM_screen;
  mapCtx.fillStyle = 'rgba(56,189,248,0.55)';
  mapCtx.fillRect(12, ch - 12, bar5m_s, 3);
  mapCtx.fillStyle = 'rgba(56,189,248,0.7)';
  mapCtx.fillText('5 m', 12 + bar5m_s + 3, ch - 10);
  mapCtx.textAlign = 'right'; mapCtx.fillStyle = 'rgba(99,130,180,0.5)';
  mapCtx.fillText(`${(meta.resolution * 100)|0} cm/px`, cw - 6, ch - 6);
  mapCtx.restore();
}

// ── Lidar render ──────────────────────────────────────────────────────────────
async function refreshLidar() {
  try {
    const data = await api('/api/lidar');
    drawRadar(data);
    drawScene(data);
    const hasRoute = data.obstacle_stop && data.avoidance_route?.length > 0;
    EL.avoidBadge.style.display = hasRoute ? '' : 'none';
  } catch (_) {}
}

function drawRadar(data) {
  const cw = EL.radarCanvas.width, ch = EL.radarCanvas.height;
  const cx = cw / 2, cy = ch / 2;
  const maxRange = data.max_range || 8.0;
  const scale = (Math.min(cw, ch) / 2 - 14) / maxRange;   // px per metre

  radarCtx.clearRect(0, 0, cw, ch);
  radarCtx.fillStyle = '#0d1422';
  radarCtx.fillRect(0, 0, cw, ch);

  // ── Distance rings ─────────────────────────────────────────────────────────
  // Inner rings: 50 cm, 100 cm, 150 cm  → labelled in centimetres
  // Outer rings: 2 m … 8 m              → labelled in metres
  [
    { r: 0.5,  lbl: '50 cm',  dash: [4,4], col: 'rgba(251,191,36,0.22)',   tc: 'rgba(251,191,36,0.75)' },
    { r: 1.0,  lbl: '100 cm', dash: [],    col: 'rgba(120,150,200,0.28)',  tc: 'rgba(140,175,230,0.75)' },
    { r: 1.5,  lbl: '150 cm', dash: [],    col: 'rgba(99,130,180,0.20)',   tc: 'rgba(140,175,230,0.60)' },
    { r: 2,    lbl: '2 m',    dash: [],    col: 'rgba(99,130,180,0.16)',   tc: 'rgba(99,130,180,0.50)' },
    { r: 3,    lbl: '3 m',    dash: [],    col: 'rgba(99,130,180,0.14)',   tc: 'rgba(99,130,180,0.45)' },
    { r: 4,    lbl: '4 m',    dash: [],    col: 'rgba(99,130,180,0.14)',   tc: 'rgba(99,130,180,0.45)' },
    { r: 5,    lbl: '5 m',    dash: [],    col: 'rgba(99,130,180,0.13)',   tc: 'rgba(99,130,180,0.40)' },
    { r: 6,    lbl: '6 m',    dash: [],    col: 'rgba(99,130,180,0.13)',   tc: 'rgba(99,130,180,0.40)' },
    { r: 8,    lbl: '8 m',    dash: [],    col: 'rgba(99,130,180,0.12)',   tc: 'rgba(99,130,180,0.38)' },
  ].forEach(({ r, lbl, dash, col, tc }) => {
    const rPx = r * scale;
    if (rPx > Math.min(cw, ch) / 2 - 4) return;
    radarCtx.setLineDash(dash);
    radarCtx.strokeStyle = col;
    radarCtx.lineWidth = 1;
    radarCtx.beginPath();
    radarCtx.arc(cx, cy, rPx, 0, Math.PI * 2);
    radarCtx.stroke();
    radarCtx.setLineDash([]);
    radarCtx.fillStyle = tc;
    radarCtx.font = r < 2 ? '8px monospace' : '9px sans-serif';
    radarCtx.textAlign = 'left';
    radarCtx.fillText(lbl, cx + rPx + 3, cy - 3);
  });

  // Crosshairs
  radarCtx.strokeStyle = 'rgba(99,130,180,0.18)';
  radarCtx.lineWidth = 1;
  radarCtx.beginPath();
  radarCtx.moveTo(cx, 4); radarCtx.lineTo(cx, ch - 4);
  radarCtx.moveTo(4, cy); radarCtx.lineTo(cw - 4, cy);
  radarCtx.stroke();

  // ── Avoidance route ────────────────────────────────────────────────────────
  const route = data.avoidance_route;
  if (data.obstacle_stop && route?.length >= 2) {
    radarCtx.save();
    radarCtx.strokeStyle = '#fbbf24'; radarCtx.lineWidth = 2.5; radarCtx.setLineDash([7, 4]);
    radarCtx.shadowColor = 'rgba(251,191,36,0.6)'; radarCtx.shadowBlur = 10;
    radarCtx.beginPath(); radarCtx.moveTo(cx, cy);
    route.forEach(wp => radarCtx.lineTo(cx - wp.y * scale, cy - wp.x * scale));
    radarCtx.stroke(); radarCtx.setLineDash([]); radarCtx.shadowBlur = 0;
    radarCtx.fillStyle = '#fbbf24';
    route.forEach((wp, i) => {
      const sx = cx - wp.y * scale, sy = cy - wp.x * scale;
      radarCtx.beginPath(); radarCtx.arc(sx, sy, i === route.length - 1 ? 5 : 3.5, 0, Math.PI * 2); radarCtx.fill();
    });
    const side = route[0]?.side || (route[0]?.y > 0 ? 'left' : 'right');
    radarCtx.font = 'bold 10px sans-serif'; radarCtx.textAlign = 'center';
    radarCtx.fillText(`DETOUR ${side.toUpperCase()}`, cx, 18);
    radarCtx.restore();
  }

  // ── Scan points ────────────────────────────────────────────────────────────
  if (data.points?.length) {
    radarCtx.fillStyle = data.obstacle_stop ? '#f87171' : '#34d399';
    data.points.forEach(p => {
      const sx = cx - p.y * scale, sy = cy - p.x * scale;
      radarCtx.fillRect(sx - 1.5, sy - 1.5, 3, 3);
    });
  }

  // Obstacle stop-zone arc
  if (data.obstacle_stop) {
    radarCtx.save();
    radarCtx.strokeStyle = 'rgba(248,113,113,0.50)'; radarCtx.lineWidth = 1.5; radarCtx.setLineDash([3, 3]);
    const stopR = 0.35 * scale;
    radarCtx.beginPath(); radarCtx.arc(cx, cy, stopR, -Math.PI * 0.75, -Math.PI * 0.25); radarCtx.stroke();
    radarCtx.setLineDash([]);
    radarCtx.fillStyle = '#f87171'; radarCtx.font = 'bold 9px sans-serif'; radarCtx.textAlign = 'center';
    radarCtx.fillText('BLOCKED', cx, cy - stopR - 5);
    radarCtx.restore();
  }

  // ── Rover physical footprint ───────────────────────────────────────────────
  // Rover: 520 × 500 × 300 mm  |  Wheels: 120 mm dia, 400 mm track
  // LiDAR: 240 mm forward of rover centre, laterally centred
  //
  // Radar screen convention (robot frame):
  //   screen_x = cx − robot_y × scale   (left=+y → left on screen)
  //   screen_y = cy − robot_x × scale   (fwd=+x  → up on screen)
  const ROVER_L   = 0.520;    // m, front–back
  const ROVER_W   = 0.500;    // m, left–right
  const WHEEL_D   = 0.120;    // m, wheel diameter
  const WHEEL_TRK = 0.400;    // m, wheel centre-to-centre
  const LIDAR_FWD = 0.240;    // m, lidar forward offset from rover centre

  const hL      = ROVER_L   / 2 * scale;    // half-length px
  const hW      = ROVER_W   / 2 * scale;    // half-width  px
  const wheelR  = WHEEL_D   / 2 * scale;    // wheel radius px
  const wheelOff = WHEEL_TRK / 2 * scale;  // lateral offset px

  radarCtx.save();

  // Body fill
  radarCtx.fillStyle = 'rgba(56,189,248,0.07)';
  radarCtx.fillRect(cx - hW, cy - hL, hW * 2, hL * 2);

  // Body border
  radarCtx.strokeStyle = 'rgba(56,189,248,0.48)';
  radarCtx.lineWidth = 1.5;
  radarCtx.strokeRect(cx - hW, cy - hL, hW * 2, hL * 2);

  // Front edge — bright green
  radarCtx.strokeStyle = '#34d399';
  radarCtx.lineWidth = 2.5;
  radarCtx.shadowColor = 'rgba(52,211,153,0.45)'; radarCtx.shadowBlur = 6;
  radarCtx.beginPath();
  radarCtx.moveTo(cx - hW, cy - hL); radarCtx.lineTo(cx + hW, cy - hL);
  radarCtx.stroke(); radarCtx.shadowBlur = 0;

  // Wheels (left = left on screen, right = right on screen)
  [['L', cx - wheelOff], ['R', cx + wheelOff]].forEach(([lbl, wx]) => {
    radarCtx.fillStyle   = 'rgba(100,116,139,0.22)';
    radarCtx.strokeStyle = 'rgba(148,163,184,0.70)';
    radarCtx.lineWidth   = 1.5;
    radarCtx.beginPath(); radarCtx.arc(wx, cy, wheelR, 0, Math.PI * 2);
    radarCtx.fill(); radarCtx.stroke();
    // tiny label
    radarCtx.fillStyle = 'rgba(148,163,184,0.55)';
    radarCtx.font = '6px sans-serif'; radarCtx.textAlign = 'center';
    radarCtx.fillText(lbl, wx, cy + 2);
    // wheel diameter callout (once, left wheel)
    if (lbl === 'L') {
      radarCtx.fillStyle = 'rgba(100,116,139,0.55)';
      radarCtx.font = '6px monospace'; radarCtx.textAlign = 'right';
      radarCtx.fillText(`⌀${Math.round(WHEEL_D*100)}cm`, wx - wheelR - 2, cy - 2);
    }
  });

  // Axle line
  radarCtx.strokeStyle = 'rgba(148,163,184,0.25)'; radarCtx.lineWidth = 1;
  radarCtx.setLineDash([2, 3]);
  radarCtx.beginPath(); radarCtx.moveTo(cx - wheelOff, cy); radarCtx.lineTo(cx + wheelOff, cy);
  radarCtx.stroke(); radarCtx.setLineDash([]);

  // LiDAR position dot
  const lidarSY = cy - LIDAR_FWD * scale;
  radarCtx.fillStyle = '#fbbf24'; radarCtx.shadowColor = '#fbbf24'; radarCtx.shadowBlur = 8;
  radarCtx.beginPath(); radarCtx.arc(cx, lidarSY, 3.5, 0, Math.PI * 2); radarCtx.fill();
  radarCtx.shadowBlur = 0;

  // LiDAR label
  radarCtx.fillStyle = 'rgba(251,191,36,0.65)';
  radarCtx.font = '6px monospace'; radarCtx.textAlign = 'left';
  radarCtx.fillText('LiDAR', cx + 5, lidarSY - 4);

  // Dimension callout — width
  radarCtx.fillStyle = 'rgba(56,189,248,0.50)';
  radarCtx.font = '7px monospace'; radarCtx.textAlign = 'center';
  radarCtx.fillText(`◀${Math.round(ROVER_W*100)} cm▶`, cx, cy + hL + 10);

  // Dimension callout — length
  radarCtx.textAlign = 'right';
  radarCtx.fillText(`${Math.round(ROVER_L*100)}cm`, cx - hW - 3, cy);

  // "FRONT" label
  radarCtx.fillStyle = '#34d399'; radarCtx.font = 'bold 7px sans-serif'; radarCtx.textAlign = 'center';
  radarCtx.fillText('▲ FRONT', cx, cy - hL - 5);

  radarCtx.restore();

  // Centre dot (rover origin)
  radarCtx.fillStyle = '#38bdf8'; radarCtx.shadowColor = '#38bdf8'; radarCtx.shadowBlur = 8;
  radarCtx.beginPath(); radarCtx.arc(cx, cy, 2.5, 0, Math.PI * 2); radarCtx.fill();
  radarCtx.shadowBlur = 0;

  radarCtx.fillStyle = 'rgba(99,130,180,0.45)';
  radarCtx.font = '8px sans-serif'; radarCtx.textAlign = 'left';
  radarCtx.fillText('▲ FWD', cx - 12, 10);
}

function drawScene(data) {
  const cw = EL.sceneCanvas.width, ch = EL.sceneCanvas.height;
  const maxRange = data.max_range || 8.0;
  sceneCtx.clearRect(0, 0, cw, ch);
  const grad = sceneCtx.createLinearGradient(0, 0, 0, ch);
  grad.addColorStop(0, '#0d1b2e'); grad.addColorStop(1, '#1a2a1a');
  sceneCtx.fillStyle = grad; sceneCtx.fillRect(0, 0, cw, ch);
  const horizon = ch * 0.55;
  sceneCtx.strokeStyle = 'rgba(56,189,248,0.15)'; sceneCtx.lineWidth = 1;
  sceneCtx.beginPath(); sceneCtx.moveTo(0, horizon); sceneCtx.lineTo(cw, horizon); sceneCtx.stroke();
  sceneCtx.strokeStyle = 'rgba(56,189,248,0.06)';
  for (let i = 0; i < 20; i++) {
    const y = horizon + (i / 20) * (ch - horizon);
    sceneCtx.beginPath(); sceneCtx.moveTo(0, y); sceneCtx.lineTo(cw, y); sceneCtx.stroke();
  }
  if (data.points?.length) {
    data.points.forEach(p => {
      const dist = Math.hypot(p.x, p.y);
      if (dist < 0.05 || dist > maxRange) return;
      const angle = Math.atan2(p.y, p.x);
      const normAngle = angle / (Math.PI / 2);
      const screenX = cw * (0.5 - normAngle * 0.55);
      const relDist  = dist / maxRange;
      const screenY  = horizon - (1 - relDist) * horizon * 0.9;
      const dotSize  = Math.max(1.5, (1 - relDist) * 8);
      const isObs    = data.obstacle_stop && Math.abs(angle) < 0.44 && dist < 0.45;
      sceneCtx.fillStyle   = isObs ? '#f87171' : '#34d399';
      sceneCtx.globalAlpha = 0.5 + relDist * 0.5;
      sceneCtx.fillRect(screenX - dotSize/2, screenY - dotSize/2, dotSize, dotSize);
    });
    sceneCtx.globalAlpha = 1;
  }
  if (data.obstacle_stop && data.avoidance_route?.length) {
    const side = data.avoidance_route[0]?.side || (data.avoidance_route[0]?.y > 0 ? 'left' : 'right');
    const arrow = side === 'left' ? '← DETOUR LEFT' : 'DETOUR RIGHT →';
    sceneCtx.fillStyle = '#fbbf24'; sceneCtx.font = 'bold 14px sans-serif';
    sceneCtx.globalAlpha = 0.90; sceneCtx.fillText(arrow, cw/2 - 64, horizon - 20);
    sceneCtx.globalAlpha = 1;
  }
}

// ── Poll loops (sequential debounce — no overlapping fetches) ─────────────────
// v4 fix: each loop awaits the previous call before scheduling the next tick,
// so a slow map payload render never blocks the faster status/lidar ticks.
function seqLoop(fn, ms) {
  async function run() {
    try { await fn(); } catch (_) {}
    setTimeout(run, ms);
  }
  run();
}

seqLoop(refreshStatus,       450);
seqLoop(refreshMap,          300);   // Faster map updates for responsive display
seqLoop(refreshLidar,        260);
seqLoop(refreshCameraStatus, 2000);
seqLoop(refreshPois,         5000);
refreshMapList();
setInterval(refreshMapList, 10000);

// Initialise map canvas cursor and tool hint
setTool('goal');

// ResizeObserver — re-fit canvas on container resize
if (typeof ResizeObserver !== 'undefined') {
  new ResizeObserver(() => { if (EL.mapOuter) refreshMap(); }).observe(EL.mapOuter);
}

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
