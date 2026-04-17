/**
 * Arespath Rover — Dashboard app.js v7
 *
 * V7 FEATURES
 * ───────────
 * • Mission map: stable static base image — walls/obstacles never flicker
 *   (separate canvas layers: static base + dynamic overlay on rAF)
 * • Google-Maps-like pan/zoom: scroll-to-zoom on cursor, inertia pan,
 *   pinch-to-zoom on touch, momentum on release
 * • Motor control: REPEAT_MS reduced to 40 ms; WebSocket path zero-copy
 * • canvasToWorld: fixed DPR/zoom coordinate transform (was double-scaling)
 * • Map PNG cache: static layer decoded once, reused; only dynamic overlay
 *   (robot pose, path, scan dots, goal, POIs) redrawn each frame
 * • Pinch-to-zoom on trackpad / touch
 * • SMOOTH TRAJECTORY: velocity-profiled path visualization with gradient colors
 *   and lookahead point indicator for pure pursuit tracking
 */

'use strict';

const STATE = {
  armed: false, mode: 'pilot', wsAlive: false, mapTool: 'goal',
  mapMeta: null, mapDirty: false, headingPrev: 0, activeCmd: null, snapping: false,
  mapZoom: 1.0, mapPanX: 0, mapPanY: 0,
  _panVX: 0, _panVY: 0, _inertiaRaf: null,
  mapDragging: false, _dragLastX: 0, _dragLastY: 0, _dragLastT: 0,
  _pinchDist: null, _pinchMidX: 0, _pinchMidY: 0,
  _pointerMoved: false, _pointerDownX: 0, _pointerDownY: 0,
  pendingPoi: null, pendingGoal: null, mapCentered: false, dragStart: null,
  pendingMission: [],
  _pois: [],
  _staticImg: null, _staticImgSrc: '', _loadingImgSrc: '', _renderPending: false,
};

const el = id => document.getElementById(id);
const EL = {
  armBtn: el('armBtn'), stopBtn: el('stopBtn'), pilotBtn: el('pilotBtn'),
  missionBtn: el('missionBtn'), speedSlider: el('speedSlider'), speedValue: el('speedValue'),
  toolGoalBtn: el('toolGoalBtn'), toolStartBtn: el('toolStartBtn'),
  toolPoiBtn: el('toolPoiBtn'), toolPanBtn: el('toolPanBtn'),
  mapStartBtn: el('mapStartBtn'), mapStopBtn: el('mapStopBtn'), mapResetBtn: el('mapResetBtn'),
  saveMapBtn: el('saveMapBtn'), loadMapBtn: el('loadMapBtn'),
  cancelNavBtn: el('cancelNavBtn'), startMissionBtn: el('startMissionBtn'),
  mapZoomInBtn: el('mapZoomInBtn'), mapZoomOutBtn: el('mapZoomOutBtn'), mapZoomResetBtn: el('mapZoomResetBtn'),
  zoomLevel: el('zoomLevel'), mapName: el('mapName'), mapSelect: el('mapSelect'),
  mapCanvas: el('mapCanvas'), mapOuter: el('mapOuter'),
  radarCanvas: el('radarCanvas'), sceneCanvas: el('sceneCanvas'),
  wsPill: el('ws-pill'), icpPill: el('icp-pill'), avoidBadge: el('avoidBadge'),
  snapToggleBtn: el('snapToggleBtn'), snapCountFront: el('snapCountFront'), snapCountRear: el('snapCountRear'),
  viewSnapshotsBtn: el('viewSnapshotsBtn'), takePhotoBtn: el('takePhotoBtn'),
  photoFeedback: el('photoFeedback'), viewPhotosBtn: el('viewPhotosBtn'),
  modalOverlay: el('modalOverlay'), modalClose: el('modalClose'), modalTitle: el('modalTitle'),
  modalSub: el('modalSub'), modalTabs: el('modalTabs'), modalGallery: el('modalGallery'),
  poiModal: el('poiModal'), poiModalClose: el('poiModalClose'), poiLabel: el('poiLabel'),
  poiKind: el('poiKind'), poiNote: el('poiNote'), poiCoordPreview: el('poiCoordPreview'),
  poiSaveBtn: el('poiSaveBtn'), poiEditModal: el('poiEditModal'), poiEditClose: el('poiEditClose'),
  poiEditId: el('poiEditId'), poiEditLabel: el('poiEditLabel'), poiEditKind: el('poiEditKind'),
  poiEditNote: el('poiEditNote'), poiEditNavBtn: el('poiEditNavBtn'), poiEditSaveBtn: el('poiEditSaveBtn'),
  poiEditDelBtn: el('poiEditDelBtn'), poiSidebarList: el('poi-sidebar-list'),
  toolHintInline: el('tool-hint-inline'),
};

const mapCtx   = EL.mapCanvas.getContext('2d');
const radarCtx = EL.radarCanvas.getContext('2d');
const sceneCtx = EL.sceneCanvas.getContext('2d');

/* ── API ─────────────────────────────────────────────────────────────────── */
async function api(path, method='GET', body=null) {
  const res = await fetch(path, {
    method,
    headers: {'Content-Type':'application/json'},
    body: body !== null ? JSON.stringify(body) : null,
  });
  if (!res.ok) { const t = await res.text().catch(()=>''); throw new Error(`${res.status} ${t}`.trim()); }
  return res.json();
}

/* ── Socket.IO ───────────────────────────────────────────────────────────── */
const socket = io({ transports: ['websocket','polling'], reconnectionDelay: 1000 });
socket.on('connect',    () => { STATE.wsAlive=true;  EL.wsPill.className='pill good'; EL.wsPill.textContent='WS'; refreshMap().catch(()=>{}); });
socket.on('disconnect', () => { STATE.wsAlive=false; EL.wsPill.className='pill bad';  EL.wsPill.textContent='WS'; _stopRepeat(); sendStop(); });
socket.on('status',     applyStatus);
socket.on('map_update', (payload) => {
  if (!payload) return;
  STATE.mapMeta = payload;
  if (payload.image_png_b64) _updateStaticImage(payload.image_png_b64);
  if (payload.pois) STATE._pois = payload.pois;
  STATE.mapDirty = true;
  scheduleMapRender();
});
setInterval(() => { if (STATE.wsAlive) socket.emit('heartbeat', {}); }, 1000);

/* ── Static map image (decoded once, reused) ─────────────────────────────── */
function _updateStaticImage(b64) {
  const src = 'data:image/png;base64,' + b64;
  if (src === STATE._staticImgSrc || src === STATE._loadingImgSrc) return;
  STATE._loadingImgSrc = src;
  const img = new Image();
  img.onload = () => {
    if (STATE._loadingImgSrc !== src) return;
    STATE._staticImg = img;
    STATE._staticImgSrc = src;
    STATE._loadingImgSrc = '';
    scheduleMapRender();
  };
  img.onerror = () => { if (STATE._loadingImgSrc === src) STATE._loadingImgSrc = ''; };
  img.src = src;
}

function scheduleMapRender() {
  if (STATE._renderPending) return;
  STATE._renderPending = true;
  requestAnimationFrame(() => { STATE._renderPending = false; renderMapFrame(); });
}

/* ── Speed ───────────────────────────────────────────────────────────────── */
function speed() { return Number(EL.speedSlider.value); }
EL.speedSlider.addEventListener('input', () => { EL.speedValue.textContent = `${EL.speedSlider.value}%`; });

/* ── Motor commands (low-latency, 25 Hz repeat) ──────────────────────────── */
function sendCommand(cmd) {
  if (cmd !== 'stop' && !STATE.armed) return;
  STATE.activeCmd = cmd; _highlightDpad(cmd);
  if (cmd === 'stop') { sendStop(); return; }
  const payload = { cmd, speed: speed() };
  if (STATE.wsAlive) socket.emit('manual_command', payload);
  else api('/api/manual','POST',_cmdToVector(cmd, speed()/100)).catch(()=>{});
}
function sendStop() {
  STATE.activeCmd = null; _highlightDpad(null); _stopRepeat();
  if (STATE.wsAlive) socket.volatile.emit('manual_command', { cmd:'stop', speed:0 });
  else api('/api/stop','POST',{}).catch(()=>{});
}
function _cmdToVector(cmd, sp) {
  return ({forward:{linear:sp,angular:0}, back:{linear:-sp,angular:0},
           left:{linear:0,angular:sp},   right:{linear:0,angular:-sp},
           stop:{linear:0,angular:0}})[cmd] || {linear:0,angular:0};
}

const REPEAT_MS = 40; // 25 Hz — was 100 ms (10 Hz), now 2.5× faster
let _repeatTimer = null;
function _startRepeat(cmd) { _stopRepeat(); _repeatTimer = setInterval(() => sendCommand(cmd), REPEAT_MS); }
function _stopRepeat()     { if (_repeatTimer !== null) { clearInterval(_repeatTimer); _repeatTimer = null; } }

document.querySelectorAll('[data-cmd]').forEach(btn => {
  const cmd = btn.dataset.cmd;
  const onDown = e => { e.preventDefault(); sendCommand(cmd); if (cmd !== 'stop') _startRepeat(cmd); };
  const onUp   = () => { _stopRepeat(); if (STATE.activeCmd === cmd || cmd === 'stop') sendStop(); };
  btn.addEventListener('pointerdown', onDown);
  btn.addEventListener('pointerup',   onUp);
  btn.addEventListener('pointercancel', onUp);
  btn.addEventListener('pointerleave', onUp);
  btn.addEventListener('contextmenu', e => e.preventDefault());
});
function _highlightDpad(activeCmd) {
  document.querySelectorAll('[data-cmd]').forEach(b => b.classList.toggle('active-cmd', b.dataset.cmd === activeCmd));
}

/* ── Keyboard ────────────────────────────────────────────────────────────── */
const KEY_CMD = { w:'forward', s:'back', a:'left', d:'right', ' ':'stop' };
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
window.addEventListener('blur', () => { _stopRepeat(); sendStop(); });
document.addEventListener('visibilitychange', () => { if (document.hidden) { _stopRepeat(); sendStop(); } });

/* ── Arm / mode ──────────────────────────────────────────────────────────── */
EL.armBtn.addEventListener('click', async () => {
  try { await api('/api/arm','POST',{armed:!STATE.armed}); await refreshStatus(); }
  catch(e) { setText('st-error', e.message); }
});
EL.stopBtn.addEventListener('click', () => sendStop());
EL.pilotBtn.addEventListener('click',   () => api('/api/mode','POST',{mode:'pilot'}).then(refreshStatus).catch(()=>{}));
EL.missionBtn.addEventListener('click', () => api('/api/mode','POST',{mode:'mission'}).then(refreshStatus).catch(()=>{}));

/* ── Map tools ───────────────────────────────────────────────────────────── */
EL.toolGoalBtn.addEventListener('click',  () => setTool('goal'));
EL.toolStartBtn.addEventListener('click', () => setTool('start'));
EL.toolPoiBtn.addEventListener('click',   () => setTool('poi'));
EL.toolPanBtn.addEventListener('click',   () => setTool('pan'));

function setTool(tool) {
  STATE.mapTool = tool;
  const labels = { goal:'GOAL', start:'START POSE', poi:'+POI', pan:'PAN' };
  setText('tool-label', labels[tool] || tool.toUpperCase());
  if (EL.toolHintInline) EL.toolHintInline.textContent = labels[tool] || tool.toUpperCase();
  EL.toolGoalBtn.classList.toggle('active',  tool==='goal');
  EL.toolStartBtn.classList.toggle('active', tool==='start');
  EL.toolPoiBtn.classList.toggle('active',   tool==='poi');
  EL.toolPanBtn.classList.toggle('active',   tool==='pan');
  EL.mapCanvas.style.cursor = tool === 'pan' ? 'grab' : 'crosshair';
}

/* ── Map controls ────────────────────────────────────────────────────────── */
EL.mapStartBtn.addEventListener('click', () => api('/api/map/start','POST',{clear:false}).catch(()=>{}));
EL.mapStopBtn.addEventListener('click',  () => api('/api/map/stop','POST',{}).catch(()=>{}));
EL.mapResetBtn.addEventListener('click', () => {
  api('/api/map/reset','POST',{}).catch(()=>{});
  STATE.mapCentered=false; STATE.pendingGoal=null; STATE.pendingMission=[];
  STATE._staticImg=null; STATE._staticImgSrc=''; STATE._loadingImgSrc='';
  updateMissionButton();
  scheduleMapRender();
});
EL.saveMapBtn.addEventListener('click', () =>
  api('/api/map/save','POST',{name:EL.mapName.value||'map'}).then(refreshMapList).catch(()=>{}));
EL.loadMapBtn.addEventListener('click', () => {
  const name = EL.mapSelect.value;
  if (name) api('/api/map/load','POST',{name}).then(()=>refreshMap()).catch(()=>{});
});
EL.startMissionBtn.addEventListener('click', () => {
  const waypoints = STATE.pendingMission.length ? STATE.pendingMission : (STATE.pendingGoal ? [STATE.pendingGoal] : []);
  if (!waypoints.length) { alert('Place one or more mission points on the map first.'); return; }
  api('/api/navigate/mission','POST',{waypoints}).then(() => {
    STATE.pendingGoal = null;
    STATE.pendingMission = [];
    updateMissionButton();
    scheduleMapRender();
  }).catch(()=>{});
});
EL.cancelNavBtn.addEventListener('click', () => {
  api('/api/navigate/cancel','POST',{}).catch(()=>{});
  STATE.pendingGoal=null; STATE.pendingMission=[]; updateMissionButton(); scheduleMapRender();
});

/* ── Google-Maps zoom/pan ────────────────────────────────────────────────── */
EL.mapZoomInBtn.addEventListener('click',    () => zoomAt(1.3, null, null));
EL.mapZoomOutBtn.addEventListener('click',   () => zoomAt(0.77, null, null));
EL.mapZoomResetBtn.addEventListener('click', () => {
  STATE.mapZoom=1; STATE.mapPanX=0; STATE.mapPanY=0; STATE._panVX=0; STATE._panVY=0;
  if (EL.zoomLevel) EL.zoomLevel.textContent='1.0×';
  scheduleMapRender();
});

/**
 * Zoom toward canvas position (cx, cy) in CSS pixels.
 * null = zoom toward center.
 */
function zoomAt(factor, cx, cy) {
  const rect = EL.mapCanvas.getBoundingClientRect();
  const mx = (cx !== null) ? cx : rect.width  / 2;
  const my = (cy !== null) ? cy : rect.height / 2;
  // World grid-pixel under cursor
  const worldX = (mx - STATE.mapPanX) / STATE.mapZoom;
  const worldY = (my - STATE.mapPanY) / STATE.mapZoom;
  STATE.mapZoom = Math.max(0.15, Math.min(20, STATE.mapZoom * factor));
  // Keep same world point under cursor
  STATE.mapPanX = mx - worldX * STATE.mapZoom;
  STATE.mapPanY = my - worldY * STATE.mapZoom;
  if (EL.zoomLevel) EL.zoomLevel.textContent = `${STATE.mapZoom.toFixed(1)}×`;
  scheduleMapRender();
}

/* Smooth scroll-to-zoom (mouse wheel + trackpad) */
EL.mapOuter && EL.mapOuter.addEventListener('wheel', e => {
  e.preventDefault();
  const rect = EL.mapCanvas.getBoundingClientRect();
  const cx = e.clientX - rect.left;
  const cy = e.clientY - rect.top;
  const delta = e.deltaMode === 1 ? e.deltaY * 30 : e.deltaY; // line vs pixel
  zoomAt(Math.pow(0.998, delta), cx, cy);
}, { passive: false });

/* ── Pointer events (pan, pinch, click) ──────────────────────────────────── */
const _activePointers = new Map();

EL.mapCanvas.addEventListener('pointerdown', e => {
  _activePointers.set(e.pointerId, {x: e.clientX, y: e.clientY});
  EL.mapCanvas.setPointerCapture(e.pointerId);

  // Cancel any running inertia scroll
  if (STATE._inertiaRaf) { cancelAnimationFrame(STATE._inertiaRaf); STATE._inertiaRaf = null; }

  if (_activePointers.size === 2) {
    // Begin pinch
    const pts = [..._activePointers.values()];
    STATE._pinchDist = Math.hypot(pts[1].x-pts[0].x, pts[1].y-pts[0].y);
    const rect = EL.mapCanvas.getBoundingClientRect();
    STATE._pinchMidX = ((pts[0].x+pts[1].x)/2) - rect.left;
    STATE._pinchMidY = ((pts[0].y+pts[1].y)/2) - rect.top;
    STATE.mapDragging = false;
    return;
  }

  STATE.mapDragging = true;
  STATE._dragLastX = e.clientX; STATE._dragLastY = e.clientY; STATE._dragLastT = performance.now();
  STATE._panVX = 0; STATE._panVY = 0;
  STATE._pointerMoved = false;
  STATE._pointerDownX = e.clientX; STATE._pointerDownY = e.clientY;
  STATE.dragStart = null;
  if (STATE.mapTool !== 'pan' && STATE.mapMeta) STATE.dragStart = canvasToWorld(e.clientX, e.clientY);
  if (STATE.mapTool === 'pan') EL.mapCanvas.style.cursor = 'grabbing';
});

EL.mapCanvas.addEventListener('pointermove', e => {
  _activePointers.set(e.pointerId, {x: e.clientX, y: e.clientY});

  // Pinch zoom
  if (_activePointers.size === 2) {
    const pts = [..._activePointers.values()];
    const dist = Math.hypot(pts[1].x-pts[0].x, pts[1].y-pts[0].y);
    if (STATE._pinchDist > 0) {
      const rect = EL.mapCanvas.getBoundingClientRect();
      zoomAt(dist/STATE._pinchDist,
             ((pts[0].x+pts[1].x)/2)-rect.left,
             ((pts[0].y+pts[1].y)/2)-rect.top);
    }
    STATE._pinchDist = dist;
    return;
  }

  if (!STATE.mapDragging) return;
  const dx = e.clientX - STATE._dragLastX;
  const dy = e.clientY - STATE._dragLastY;
  const dt = Math.max(1, performance.now() - STATE._dragLastT);
  STATE._panVX = dx / dt * 16;
  STATE._panVY = dy / dt * 16;

  const moved = Math.hypot(e.clientX - STATE._pointerDownX, e.clientY - STATE._pointerDownY) > 6;
  if (STATE.mapTool === 'pan' || moved) {
    STATE._pointerMoved = true;
    STATE.mapPanX += dx; STATE.mapPanY += dy;
    scheduleMapRender();
  }
  if (STATE.mapTool === 'start' && STATE.mapMeta && STATE.dragStart) {
    const cur = canvasToWorld(e.clientX, e.clientY);
    STATE.headingPrev = Math.atan2(cur.y-STATE.dragStart.y, cur.x-STATE.dragStart.x);
  }
  STATE._dragLastX = e.clientX; STATE._dragLastY = e.clientY; STATE._dragLastT = performance.now();
});

EL.mapCanvas.addEventListener('pointerup', e => {
  _activePointers.delete(e.pointerId);
  if (EL.mapCanvas.hasPointerCapture?.(e.pointerId)) EL.mapCanvas.releasePointerCapture(e.pointerId);
  if (_activePointers.size < 2) STATE._pinchDist = null;
  if (!STATE.mapDragging) return;
  STATE.mapDragging = false;
  EL.mapCanvas.style.cursor = STATE.mapTool === 'pan' ? 'grab' : 'crosshair';

  // Launch inertia if we were panning
  if (STATE._pointerMoved) { _startInertia(); STATE.dragStart = null; return; }

  // Click actions (pointer didn't move significantly)
  if (STATE.mapMeta) {
    const pt = canvasToWorld(e.clientX, e.clientY);
    if (STATE.mapTool === 'goal') {
      STATE.pendingGoal = {x: pt.x, y: pt.y};
      STATE.pendingMission.push(STATE.pendingGoal);
      updateMissionButton();
      scheduleMapRender();
    } else if (STATE.mapTool === 'start') {
      const start = STATE.dragStart || pt;
      api('/api/pose','POST',{x:start.x, y:start.y, theta:STATE.headingPrev}).catch(()=>{});
    } else if (STATE.mapTool === 'poi') {
      _openPoiAdd(pt.x, pt.y);
    }
  }
  STATE.dragStart = null;
});

EL.mapCanvas.addEventListener('pointercancel', e => {
  _activePointers.delete(e.pointerId);
  STATE.mapDragging = false; STATE._pinchDist = null;
  scheduleMapRender();
});

/* Inertia pan (exponential decay) */
function _startInertia() {
  if (STATE._inertiaRaf) cancelAnimationFrame(STATE._inertiaRaf);
  function step() {
    STATE._panVX *= 0.88; STATE._panVY *= 0.88;
    if (Math.abs(STATE._panVX) < 0.15 && Math.abs(STATE._panVY) < 0.15) {
      STATE._inertiaRaf = null; return;
    }
    STATE.mapPanX += STATE._panVX; STATE.mapPanY += STATE._panVY;
    scheduleMapRender();
    STATE._inertiaRaf = requestAnimationFrame(step);
  }
  STATE._inertiaRaf = requestAnimationFrame(step);
}

/* ── Coordinate transform ─────────────────────────────────────────────────── */
/**
 * Convert screen position to world metres.
 * Pan is in CSS pixels; zoom is unitless — no DPR needed since
 * getBoundingClientRect() returns CSS-pixel values.
 */
function canvasToWorld(clientX, clientY) {
  const meta = STATE.mapMeta;
  if (!meta) return {x:0, y:0};
  const rect = EL.mapCanvas.getBoundingClientRect();
  const cssx = clientX - rect.left;
  const cssy = clientY - rect.top;
  const gridX = (cssx - STATE.mapPanX) / STATE.mapZoom;
  const gridY = (cssy - STATE.mapPanY) / STATE.mapZoom;
  return {
    x: gridX * meta.resolution + meta.origin[0],
    y: gridY * meta.resolution + meta.origin[1],
  };
}

/* ── POI modals ──────────────────────────────────────────────────────────── */
function _openPoiAdd(wx, wy) {
  STATE.pendingPoi = {x:wx, y:wy};
  EL.poiLabel.value=''; EL.poiKind.value='waypoint'; EL.poiNote.value='';
  EL.poiCoordPreview.textContent = `(${wx.toFixed(2)} m, ${wy.toFixed(2)} m)`;
  EL.poiModal.style.display='flex'; EL.poiLabel.focus();
}
EL.poiModalClose.addEventListener('click', () => { EL.poiModal.style.display='none'; });
EL.poiSaveBtn.addEventListener('click', async () => {
  if (!STATE.pendingPoi) return;
  try {
    await api('/api/poi','POST',{label:EL.poiLabel.value.trim()||'Waypoint',
      kind:EL.poiKind.value, note:EL.poiNote.value.trim(), ...STATE.pendingPoi});
    EL.poiModal.style.display='none'; STATE.pendingPoi=null; await refreshPois();
  } catch(err) { alert('POI save failed: '+err.message); }
});
function _openPoiEdit(poi) {
  EL.poiEditId.value=poi.id; EL.poiEditLabel.value=poi.label;
  EL.poiEditKind.value=poi.kind; EL.poiEditNote.value=poi.note||'';
  EL.poiEditModal.style.display='flex';
}
EL.poiEditClose.addEventListener('click', () => { EL.poiEditModal.style.display='none'; });
EL.poiEditSaveBtn.addEventListener('click', async () => {
  const id = EL.poiEditId.value;
  try {
    await api(`/api/poi/${id}`,'PATCH',{label:EL.poiEditLabel.value.trim(),
      kind:EL.poiEditKind.value, note:EL.poiEditNote.value.trim()});
    EL.poiEditModal.style.display='none'; await refreshPois();
  } catch(err) { alert('Update failed: '+err.message); }
});
EL.poiEditNavBtn.addEventListener('click', async () => {
  try { await api(`/api/poi/${EL.poiEditId.value}/navigate`,'POST',{}); EL.poiEditModal.style.display='none'; }
  catch(err) { alert('Navigate failed: '+err.message); }
});
EL.poiEditDelBtn.addEventListener('click', async () => {
  if (!confirm('Delete this POI?')) return;
  try { await api(`/api/poi/${EL.poiEditId.value}`,'DELETE'); EL.poiEditModal.style.display='none'; await refreshPois(); }
  catch(err) { alert('Delete failed: '+err.message); }
});
const KIND_ICON = { gazebo:'⛺', waypoint:'📍', dock:'🔌', custom:'⭐' };
async function refreshPois() {
  try {
    const data = await api('/api/poi');
    const pois = data.pois || [];
    if (!pois.length) {
      EL.poiSidebarList.innerHTML='<p class="hint">No POIs yet. Select "+POI" tool and click the map.</p>'; return;
    }
    EL.poiSidebarList.innerHTML='';
    pois.forEach(poi => {
      const row = document.createElement('div'); row.className='poi-row';
      row.innerHTML = `<span class="poi-icon">${KIND_ICON[poi.kind]||'⭐'}</span>
        <span class="poi-name">${_esc(poi.label)}</span>
        <button class="poi-nav-btn secondary" data-id="${poi.id}" title="Navigate here">▶</button>
        <button class="poi-edit-btn secondary" data-id="${poi.id}" title="Edit">✎</button>`;
      EL.poiSidebarList.appendChild(row);
    });
    EL.poiSidebarList.querySelectorAll('.poi-nav-btn').forEach(btn =>
      btn.addEventListener('click', () => api(`/api/poi/${btn.dataset.id}/navigate`,'POST',{}).catch(e=>alert(e.message))));
    EL.poiSidebarList.querySelectorAll('.poi-edit-btn').forEach(btn =>
      btn.addEventListener('click', () => { const p=pois.find(p=>p.id===btn.dataset.id); if(p) _openPoiEdit(p); }));
    STATE._pois = pois;
  } catch(_) {}
}
function _esc(s) { return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;'); }

/* ── Camera / snapshots ──────────────────────────────────────────────────── */
EL.snapToggleBtn.addEventListener('click', async () => {
  try {
    if (STATE.snapping) { await api('/api/snapshot/stop','POST',{}); STATE.snapping=false; }
    else                { await api('/api/snapshot/start','POST',{}); STATE.snapping=true; }
    _updateSnapBtn();
  } catch(e) { console.warn('Snapshot toggle:',e); }
});
function _updateSnapBtn() {
  if (STATE.snapping) { EL.snapToggleBtn.textContent='⏹ Stop Snapshots'; EL.snapToggleBtn.classList.add('danger'); EL.snapToggleBtn.classList.remove('secondary'); }
  else                { EL.snapToggleBtn.textContent='📷 Start Snapshots'; EL.snapToggleBtn.classList.add('secondary'); EL.snapToggleBtn.classList.remove('danger'); }
}
async function refreshCameraStatus() {
  try {
    const data = await api('/api/cameras');
    STATE.snapping = data.snapping; _updateSnapBtn();
    const c = data.counts||{}; EL.snapCountFront.textContent=c.front??0; EL.snapCountRear.textContent=c.rear??0;
  } catch(_) {}
}
EL.takePhotoBtn.addEventListener('click', async () => {
  EL.takePhotoBtn.disabled=true; EL.photoFeedback.textContent='Saving…';
  try {
    const r = await api('/api/photo/take','POST',{});
    EL.photoFeedback.textContent = `✓ Saved: ${Object.keys(r.saved||{}).join(', ')||'none'}`;
    setTimeout(()=>{ EL.photoFeedback.textContent=''; }, 3000);
  } catch(e) { EL.photoFeedback.textContent=`✗ ${e.message}`; setTimeout(()=>{EL.photoFeedback.textContent='';},4000); }
  finally { EL.takePhotoBtn.disabled=false; }
});
let _galleryMode='snapshots', _galleryData={}, _galleryTab='front';
function _fmtTime(ts){ const d=new Date(ts*1000); return d.toLocaleString('en-GB',{day:'2-digit',month:'short',hour:'2-digit',minute:'2-digit',second:'2-digit'}); }
function _fmtSize(b){ if(b<1024) return b+' B'; if(b<1048576) return (b/1024).toFixed(1)+' KB'; return (b/1048576).toFixed(2)+' MB'; }
EL.viewSnapshotsBtn.addEventListener('click', async () => {
  _galleryMode='snapshots'; _galleryTab='front';
  EL.modalTitle.textContent='📷 Auto Snapshots'; EL.modalSub.textContent='Ring-buffer — newest first — max 10 per camera';
  EL.modalTabs.style.display=''; EL.modalGallery.innerHTML='<div class="modal-empty">Loading…</div>';
  EL.modalOverlay.style.display='flex';
  try { const data=await api('/api/snapshot/files'); _galleryData=data.files||{}; _setActiveTab(_galleryTab); _renderSnapshots(); }
  catch(e) { EL.modalGallery.innerHTML=`<div class="modal-empty">Error: ${e.message}</div>`; }
});
EL.viewPhotosBtn.addEventListener('click', async () => {
  _galleryMode='photos'; EL.modalTitle.textContent='🗂 Permanent Photos'; EL.modalSub.textContent='All captured photos';
  EL.modalTabs.style.display='none'; EL.modalGallery.innerHTML='<div class="modal-empty">Loading…</div>';
  EL.modalOverlay.style.display='flex';
  try { const data=await api('/api/photo/files'); _renderPhotos(data.files||[]); }
  catch(e) { EL.modalGallery.innerHTML=`<div class="modal-empty">Error: ${e.message}</div>`; }
});
EL.modalTabs.querySelectorAll('.modal-tab').forEach(tab=>tab.addEventListener('click',()=>{ _setActiveTab(tab.dataset.tab); _renderSnapshots(); }));
function _setActiveTab(tab) { _galleryTab=tab; EL.modalTabs.querySelectorAll('.modal-tab').forEach(t=>t.classList.toggle('active',t.dataset.tab===tab)); }
function _renderSnapshots() {
  let items=[];
  if (_galleryTab==='all') { const all=[]; Object.entries(_galleryData).forEach(([cam,fnames])=>fnames.forEach(f=>all.push({cam,f}))); all.sort((a,b)=>b.f.localeCompare(a.f)); items=all; }
  else items=(_galleryData[_galleryTab]||[]).map(f=>({cam:_galleryTab,f}));
  if (!items.length) { EL.modalGallery.innerHTML='<div class="modal-empty">No snapshots yet.</div>'; return; }
  const grid=document.createElement('div'); grid.className='gallery-grid';
  items.forEach(({cam,f})=>{ const ts=_parseTs(f); grid.appendChild(_makeCard({src:`/snapshots/${f}`,dlHref:`/snapshots/${f}?dl=1`,dlName:f,metaTop:`<strong>${cam}</strong>`,metaBot:ts?_fmtTime(ts):f})); });
  EL.modalGallery.innerHTML=''; EL.modalGallery.appendChild(grid);
}
function _renderPhotos(photos) {
  if (!photos.length) { EL.modalGallery.innerHTML='<div class="modal-empty">No photos taken yet.</div>'; return; }
  const grid=document.createElement('div'); grid.className='gallery-grid';
  photos.forEach(p=>grid.appendChild(_makeCard({src:`/photos/${p.filename}`,dlHref:`/photos/${p.filename}?dl=1`,dlName:p.filename,metaTop:`<strong>${p.cam}</strong> · ${_fmtSize(p.size)}`,metaBot:_fmtTime(p.ts)})));
  EL.modalGallery.innerHTML=''; EL.modalGallery.appendChild(grid);
}
function _makeCard({src,dlHref,dlName,metaTop,metaBot}) {
  const card=document.createElement('div'); card.className='gallery-card';
  const img=document.createElement('img'); img.className='gallery-img'; img.src=src; img.alt=dlName; img.loading='lazy';
  img.addEventListener('click',()=>_openLightbox(src));
  const foot=document.createElement('div'); foot.className='gallery-card-foot';
  const meta=document.createElement('div'); meta.className='gallery-meta'; meta.innerHTML=`${metaTop}<br>${metaBot}`;
  const dlLink=document.createElement('a'); dlLink.className='gallery-dl'; dlLink.href=dlHref; dlLink.download=dlName; dlLink.textContent='⬇ Download';
  foot.appendChild(meta); foot.appendChild(dlLink); card.appendChild(img); card.appendChild(foot); return card;
}
function _parseTs(f) { const m=f.match(/_(\d{9,13})\.jpg$/); return m?parseInt(m[1],10):null; }
EL.modalClose.addEventListener('click', ()=>{ EL.modalOverlay.style.display='none'; });
EL.modalOverlay.addEventListener('click', e=>{ if(e.target===EL.modalOverlay) EL.modalOverlay.style.display='none'; });
document.addEventListener('keydown', e=>{ if(e.key==='Escape'){ EL.modalOverlay.style.display='none'; EL.poiModal.style.display='none'; EL.poiEditModal.style.display='none'; } });
function _openLightbox(src) {
  const lb=document.createElement('div'); lb.className='lightbox';
  const img=document.createElement('img'); img.src=src;
  lb.appendChild(img); lb.addEventListener('click',()=>lb.remove());
  document.addEventListener('keydown', function esc(e){ if(e.key==='Escape'){ lb.remove(); document.removeEventListener('keydown',esc); } });
  document.body.appendChild(lb);
}

/* ── Status display ──────────────────────────────────────────────────────── */
function setText(id,value){ const n=document.getElementById(id); if(n) n.textContent=value; }
function applyStatus(s) {
  STATE.armed=s.armed; STATE.mode=s.mode;
  EL.armBtn.textContent = s.armed ? 'Disarm' : 'Arm Motors';
  EL.pilotBtn.classList.toggle('active',  s.mode==='pilot');
  EL.missionBtn.classList.toggle('active', s.mode==='mission');
  setText('st-connected', s.connected?'ONLINE':'OFFLINE');
  setText('st-lidar', s.lidar_connected?'READY':'OFF');
  setText('st-mode', s.mode.toUpperCase());
  setText('st-battery', s.battery_v!=null?`${s.battery_v.toFixed(2)} V`:'--');
  setText('st-age',   `${(s.telemetry_age_s??0).toFixed(1)} s`);
  setText('st-error', s.last_error||'--');
  setText('pose-x', (s.pose?.x??0).toFixed(2));
  setText('pose-y', (s.pose?.y??0).toFixed(2));
  setText('pose-theta', ((s.pose?.theta??0)*180/Math.PI).toFixed(1));
  setText('left-rpm',  (s.left_rpm??0).toFixed(1));
  setText('right-rpm', (s.right_rpm??0).toFixed(1));
  setText('left-pwm',  String(s.left_pwm??0));
  setText('right-pwm', String(s.right_pwm??0));
  setText('scan-pts',  String(s.latest_scan_points??0));
  setText('path-pts',  String(s.nav?.path?.length??0));
  const rem = s.obstacle_wait_remaining_s;
  if (s.obstacle_stop && rem>0) { setText('st-safety',`⏳ ${rem.toFixed(0)} s`); _setObstacleCountdown(rem,5.0); }
  else if (s.obstacle_stop)     { setText('st-safety','BLOCKED'); _clearObstacleCountdown(); }
  else                          { setText('st-safety','CLEAR');   _clearObstacleCountdown(); }
  const navSt = s.nav?.status||'--'; setText('st-nav', navSt);
  const navEl = document.getElementById('st-nav');
  if (navEl) navEl.style.color = navSt.includes('waiting')?'#fbbf24':navSt.includes('replan')?'#f87171':navSt.includes('reached')?'#34d399':'';
  EL.icpPill.className = s.lidar_connected ? 'pill good' : 'pill dim';
}
function _setObstacleCountdown(remaining, total) {
  let ring = document.getElementById('obs-countdown-ring');
  if (!ring) {
    const wrap=document.createElement('span'); wrap.id='obs-countdown-wrap'; wrap.title='Obstacle patience timer';
    wrap.innerHTML=`<svg id="obs-countdown-ring" width="22" height="22" viewBox="0 0 22 22">
      <circle cx="11" cy="11" r="9" fill="none" stroke="rgba(251,191,36,0.2)" stroke-width="2.5"/>
      <circle id="obs-cd-arc" cx="11" cy="11" r="9" fill="none" stroke="#fbbf24" stroke-width="2.5"
        stroke-dasharray="56.55" stroke-dashoffset="0" stroke-linecap="round" transform="rotate(-90 11 11)"/>
      <text id="obs-cd-txt" x="11" y="14.5" text-anchor="middle" font-size="7" fill="#fbbf24" font-family="monospace"></text>
    </svg>`;
    document.getElementById('conn-bar').appendChild(wrap);
    ring = document.getElementById('obs-countdown-ring');
  }
  const arc=document.getElementById('obs-cd-arc'), txt=document.getElementById('obs-cd-txt');
  arc.setAttribute('stroke-dashoffset', String(2*Math.PI*9*(1-Math.max(0,remaining/total))));
  txt.textContent = Math.ceil(remaining)+'s';
}
function _clearObstacleCountdown() { const w=document.getElementById('obs-countdown-wrap'); if(w) w.remove(); }
async function refreshStatus() {
  try { applyStatus(await api('/api/status')); }
  catch(e) { setText('st-connected','ERROR'); setText('st-error',e.message); }
}
async function refreshMapList() {
  try {
    const data = await api('/api/map/list'); const cur=EL.mapSelect.value;
    EL.mapSelect.innerHTML='';
    data.maps.forEach(name=>{ const o=document.createElement('option'); o.value=name; o.textContent=name; EL.mapSelect.appendChild(o); });
    if (data.maps.includes(cur)) EL.mapSelect.value=cur; else if (data.maps.length) EL.mapSelect.value=data.maps[0];
  } catch(_) {}
}

/* ── Canvas sizing ───────────────────────────────────────────────────────── */
function _fitCanvas() {
  const outer = EL.mapOuter; if (!outer) return;
  const w=outer.clientWidth||800, h=outer.clientHeight||800;
  const dpr = window.devicePixelRatio||1;
  if (EL.mapCanvas.width!==Math.floor(w*dpr)||EL.mapCanvas.height!==Math.floor(h*dpr)) {
    EL.mapCanvas.width=Math.floor(w*dpr); EL.mapCanvas.height=Math.floor(h*dpr);
    EL.mapCanvas.style.width=w+'px'; EL.mapCanvas.style.height=h+'px';
  }
}
async function refreshMap() {
  _fitCanvas();
  try {
    const meta = await api('/api/map/data');
    STATE.mapMeta = meta;
    if (meta.image_png_b64) _updateStaticImage(meta.image_png_b64);
    if (!STATE.mapCentered && meta.pose) {
      const rect = EL.mapCanvas.getBoundingClientRect();
      STATE.mapPanX = rect.width  / 2 - meta.pose.x * STATE.mapZoom;
      STATE.mapPanY = rect.height / 2 - meta.pose.y * STATE.mapZoom;
      STATE.mapCentered = true;
    }
    scheduleMapRender();
  } catch(_) {}
}

function updateMissionButton() {
  const count = STATE.pendingMission.length;
  EL.startMissionBtn.textContent = count > 1 ? `Start Mission (${count})` : 'Start Mission';
}

/* ── Main map render ─────────────────────────────────────────────────────── */
/**
 * Layered rendering:
 *   1. Static SLAM base (PNG decoded once) — walls never re-flicker
 *   2. Grid overlay
 *   3. Dynamic overlay (scan, path, goal, POIs, robot)
 *   4. HUD (scale bar)
 *
 * Transform stack:
 *   Physical canvas = CSS canvas × DPR
 *   setTransform(dpr*z, 0, 0, dpr*z,  dpr*panX, dpr*panY)
 *   → draw in grid-pixels, pan/zoom in CSS pixels
 */
function renderMapFrame() {
  const meta = STATE.mapMeta; if (!meta) return;
  _fitCanvas();

  const dpr = window.devicePixelRatio||1;
  const cw = EL.mapCanvas.width, ch = EL.mapCanvas.height;
  const cssW = cw/dpr, cssH = ch/dpr;
  const z = STATE.mapZoom;

  mapCtx.setTransform(1,0,0,1,0,0);
  mapCtx.clearRect(0,0,cw,ch);
  mapCtx.fillStyle='#0a0f1e'; mapCtx.fillRect(0,0,cw,ch);

  // Unified transform: DPR × zoom, pan in CSS pixels × DPR
  mapCtx.setTransform(dpr*z, 0, 0, dpr*z, dpr*STATE.mapPanX, dpr*STATE.mapPanY);

  // Layer 1: static SLAM base
  if (STATE._staticImg) {
    mapCtx.drawImage(STATE._staticImg, 0, 0);
  } else {
    mapCtx.fillStyle='#111827'; mapCtx.fillRect(0,0,meta.width||2400,meta.height||2400);
  }

  // Layer 2: faint grid (culled to visible area)
  const invZ = 1/z;
  const visX = -STATE.mapPanX*invZ, visY = -STATE.mapPanY*invZ;
  const visW = cssW*invZ,           visH = cssH*invZ;
  const step = Math.max(5, Math.round(1/meta.resolution));
  mapCtx.strokeStyle='rgba(255,255,255,0.04)'; mapCtx.lineWidth=invZ;
  const x0=Math.floor(visX/step)*step, x1=Math.ceil((visX+visW)/step)*step;
  const y0=Math.floor(visY/step)*step, y1=Math.ceil((visY+visH)/step)*step;
  for (let x=x0;x<=x1;x+=step){ mapCtx.beginPath(); mapCtx.moveTo(x,visY); mapCtx.lineTo(x,visY+visH); mapCtx.stroke(); }
  for (let y=y0;y<=y1;y+=step){ mapCtx.beginPath(); mapCtx.moveTo(visX,y); mapCtx.lineTo(visX+visW,y); mapCtx.stroke(); }

  // Layer 3: live scan dots
  if (meta.scan?.length) {
    mapCtx.fillStyle='rgba(52,211,153,0.65)'; const dot=1.5*invZ;
    meta.scan.forEach(p=>mapCtx.fillRect(p.x-dot/2,p.y-dot/2,dot,dot));
  }

  // Path with velocity-based color gradient (v7: smooth trajectory visualization)
  if (meta.path?.length>1) {
    mapCtx.save();
    mapCtx.lineCap='round';
    mapCtx.lineJoin='round';

    const pathLen=meta.path.length;
    for(let i=1;i<pathLen;i++){
      const p0=meta.path[i-1], p1=meta.path[i];
      const t=i/pathLen;
      const r=Math.round(56*(1-t)+56*t);
      const g=Math.round(189*(1-t)+100*t);
      const b=Math.round(248*(1-t)+71*t);
      mapCtx.strokeStyle=`rgba(${r},${g},${b},0.9)`;
      mapCtx.lineWidth=(1.5+1.5*((1-t)*0.8+0.2))*invZ;
      mapCtx.shadowColor=`rgba(${r},${g},${b},0.4)`;
      mapCtx.shadowBlur=4*invZ;
      mapCtx.beginPath();
      mapCtx.moveTo(p0.x,p0.y);
      mapCtx.lineTo(p1.x,p1.y);
      mapCtx.stroke();
    }

    // Draw lookahead point indicator
    if(meta.path?.length>3){
      const idx=Math.floor(pathLen*0.3);
      const lp=meta.path[idx];
      mapCtx.fillStyle='rgba(251,191,36,0.8)';
      mapCtx.shadowColor='#fbbf24';
      mapCtx.shadowBlur=8*invZ;
      mapCtx.beginPath();
      mapCtx.arc(lp.x,lp.y,3*invZ,0,Math.PI*2);
      mapCtx.fill();
    }
    mapCtx.restore();
  }

  // Pending goal (blue crosshair)
  if (STATE.pendingGoal && meta.origin && meta.resolution) {
    const gx=(STATE.pendingGoal.x-meta.origin[0])/meta.resolution;
    const gy=(STATE.pendingGoal.y-meta.origin[1])/meta.resolution;
    mapCtx.save(); mapCtx.strokeStyle='#60a5fa'; mapCtx.lineWidth=1.5*invZ;
    mapCtx.setLineDash([3*invZ,2*invZ]);
    mapCtx.beginPath(); mapCtx.arc(gx,gy,3*invZ,0,Math.PI*2); mapCtx.stroke();
    mapCtx.setLineDash([]);
    mapCtx.beginPath();
    mapCtx.moveTo(gx,gy-8*invZ); mapCtx.lineTo(gx,gy+8*invZ);
    mapCtx.moveTo(gx-8*invZ,gy); mapCtx.lineTo(gx+8*invZ,gy);
    mapCtx.stroke(); mapCtx.restore();
  }

  if (STATE.pendingMission?.length) {
    mapCtx.save();
    mapCtx.lineWidth=1.5*invZ;
    mapCtx.strokeStyle='rgba(96,165,250,0.85)';
    mapCtx.fillStyle='rgba(96,165,250,0.95)';
    mapCtx.setLineDash([4*invZ,4*invZ]);
    for (let i=0;i<STATE.pendingMission.length;i++) {
      const wp = STATE.pendingMission[i];
      const gx=(wp.x-meta.origin[0])/meta.resolution;
      const gy=(wp.y-meta.origin[1])/meta.resolution;
      if (i===0) {
        mapCtx.beginPath();
        mapCtx.moveTo(gx,gy);
      } else {
        mapCtx.lineTo(gx,gy);
      }
    }
    if (STATE.pendingMission.length > 1) mapCtx.stroke();
    mapCtx.setLineDash([]);
    STATE.pendingMission.forEach((wp, idx) => {
      const gx=(wp.x-meta.origin[0])/meta.resolution;
      const gy=(wp.y-meta.origin[1])/meta.resolution;
      mapCtx.beginPath(); mapCtx.arc(gx,gy,5*invZ,0,Math.PI*2); mapCtx.fill();
      mapCtx.fillStyle='#0f172a';
      mapCtx.font=`${10*invZ}px sans-serif`;
      mapCtx.textAlign='center'; mapCtx.textBaseline='middle';
      mapCtx.fillText(String(idx+1),gx,gy);
      mapCtx.fillStyle='rgba(96,165,250,0.95)';
    });
    mapCtx.restore();
  }

  // Active goal (gold)
  if (meta.goal) {
    const gx=meta.goal.x, gy=meta.goal.y;
    mapCtx.save(); mapCtx.strokeStyle='#fbbf24'; mapCtx.lineWidth=2*invZ;
    mapCtx.shadowColor='#fbbf24'; mapCtx.shadowBlur=6*invZ;
    mapCtx.beginPath(); mapCtx.arc(gx,gy,3.5*invZ,0,Math.PI*2); mapCtx.stroke();
    mapCtx.beginPath();
    mapCtx.moveTo(gx,gy-10*invZ); mapCtx.lineTo(gx,gy+10*invZ);
    mapCtx.moveTo(gx-10*invZ,gy); mapCtx.lineTo(gx+10*invZ,gy);
    mapCtx.stroke(); mapCtx.restore();
  }

  if (meta.mission_waypoints?.length) {
    mapCtx.save();
    mapCtx.strokeStyle='rgba(251,191,36,0.8)';
    mapCtx.lineWidth=1.5*invZ;
    mapCtx.setLineDash([6*invZ,4*invZ]);
    meta.mission_waypoints.forEach((wp, idx) => {
      if (idx===0) {
        mapCtx.beginPath();
        mapCtx.moveTo(wp.x, wp.y);
      } else {
        mapCtx.lineTo(wp.x, wp.y);
      }
    });
    if (meta.mission_waypoints.length > 1) mapCtx.stroke();
    mapCtx.setLineDash([]);
    meta.mission_waypoints.forEach((wp, idx) => {
      mapCtx.fillStyle='rgba(251,191,36,0.92)';
      mapCtx.beginPath(); mapCtx.arc(wp.x, wp.y, 4.5*invZ, 0, Math.PI*2); mapCtx.fill();
      mapCtx.fillStyle='#111827';
      mapCtx.font=`${9*invZ}px sans-serif`;
      mapCtx.textAlign='center'; mapCtx.textBaseline='middle';
      mapCtx.fillText(String(idx+1), wp.x, wp.y);
    });
    mapCtx.restore();
  }

  // POIs
  (meta.pois||[]).forEach(poi=>{
    const gx=poi.px, gy=poi.py, r=8*invZ;
    mapCtx.save(); mapCtx.fillStyle='rgba(251,191,36,0.18)';
    mapCtx.beginPath(); mapCtx.arc(gx,gy,r*1.6,0,Math.PI*2); mapCtx.fill();
    mapCtx.strokeStyle='#fbbf24'; mapCtx.lineWidth=1.5*invZ;
    mapCtx.shadowColor='#fbbf24'; mapCtx.shadowBlur=4*invZ;
    mapCtx.beginPath(); mapCtx.arc(gx,gy,r,0,Math.PI*2); mapCtx.stroke();
    mapCtx.restore();
    mapCtx.save(); mapCtx.font=`${11*invZ}px sans-serif`;
    mapCtx.textAlign='center'; mapCtx.textBaseline='top'; mapCtx.fillStyle='#fbbf24';
    mapCtx.shadowColor='#000'; mapCtx.shadowBlur=2*invZ;
    mapCtx.fillText(poi.label,gx,gy+r+invZ); mapCtx.restore();
  });

  // Robot body
  if (meta.pose) {
    const gx=meta.pose.x, gy=meta.pose.y, th=meta.pose.theta;
    const ppm=1/meta.resolution, rL=0.520/2*ppm, rW=0.500/2*ppm;
    mapCtx.save(); mapCtx.translate(gx,gy); mapCtx.rotate(th);
    mapCtx.fillStyle='rgba(56,189,248,0.12)'; mapCtx.fillRect(-rW,-rL,rW*2,rL*2);
    mapCtx.strokeStyle='rgba(56,189,248,0.52)'; mapCtx.lineWidth=1.5*invZ; mapCtx.strokeRect(-rW,-rL,rW*2,rL*2);
    mapCtx.strokeStyle='#34d399'; mapCtx.lineWidth=2.5*invZ;
    mapCtx.shadowColor='rgba(52,211,153,0.5)'; mapCtx.shadowBlur=6*invZ;
    mapCtx.beginPath(); mapCtx.moveTo(-rW,rL); mapCtx.lineTo(rW,rL); mapCtx.stroke();
    mapCtx.shadowBlur=0;
    mapCtx.fillStyle='#38bdf8';
    mapCtx.shadowColor='rgba(56,189,248,0.5)'; mapCtx.shadowBlur=8*invZ;
    const ar=Math.max(3*invZ,rL*0.5);
    mapCtx.beginPath(); mapCtx.moveTo(0,rL-ar*0.2); mapCtx.lineTo(-ar*0.5,rL-ar); mapCtx.lineTo(ar*0.5,rL-ar);
    mapCtx.closePath(); mapCtx.fill(); mapCtx.restore();
  }

  // Layer 4: HUD (CSS pixel coords)
  mapCtx.setTransform(dpr,0,0,dpr,0,0);
  const ppm=z/meta.resolution, bar1m=ppm, bar5m=5*ppm;
  mapCtx.save();
  mapCtx.fillStyle='rgba(255,255,255,0.80)'; mapCtx.fillRect(12,cssH-22,bar1m,3);
  mapCtx.font='8px monospace'; mapCtx.textAlign='left'; mapCtx.fillStyle='rgba(255,255,255,0.70)';
  mapCtx.fillText('1 m',12+bar1m+3,cssH-18);
  mapCtx.fillStyle='rgba(56,189,248,0.48)'; mapCtx.fillRect(12,cssH-12,bar5m,3);
  mapCtx.fillStyle='rgba(56,189,248,0.62)'; mapCtx.fillText('5 m',12+bar5m+3,cssH-10);
  mapCtx.textAlign='right'; mapCtx.fillStyle='rgba(99,130,180,0.40)';
  mapCtx.fillText(`${(meta.resolution*100)|0} cm/px  z×${z.toFixed(1)}`,cssW-6,cssH-6);
  mapCtx.restore();
}

/* ── Lidar radar ─────────────────────────────────────────────────────────── */
async function refreshLidar() {
  try {
    const data = await api('/api/lidar');
    drawRadar(data); drawScene(data);
    EL.avoidBadge.style.display = (data.obstacle_stop&&data.avoidance_route?.length>0)?'':'none';
  } catch(_) {}
}
function drawRadar(data) {
  const cw=EL.radarCanvas.width, ch=EL.radarCanvas.height, cx=cw/2, cy=ch/2;
  const maxRange=data.max_range||8.0, scale=(Math.min(cw,ch)/2-14)/maxRange;
  radarCtx.clearRect(0,0,cw,ch); radarCtx.fillStyle='#0d1422'; radarCtx.fillRect(0,0,cw,ch);
  [{r:0.5,lbl:'50 cm',dash:[4,4],col:'rgba(251,191,36,0.22)',tc:'rgba(251,191,36,0.75)'},
   {r:1.0,lbl:'100 cm',dash:[],col:'rgba(120,150,200,0.28)',tc:'rgba(140,175,230,0.75)'},
   {r:1.5,lbl:'150 cm',dash:[],col:'rgba(99,130,180,0.20)',tc:'rgba(140,175,230,0.60)'},
   {r:2,lbl:'2 m',dash:[],col:'rgba(99,130,180,0.16)',tc:'rgba(99,130,180,0.50)'},
   {r:3,lbl:'3 m',dash:[],col:'rgba(99,130,180,0.14)',tc:'rgba(99,130,180,0.45)'},
   {r:5,lbl:'5 m',dash:[],col:'rgba(99,130,180,0.13)',tc:'rgba(99,130,180,0.40)'},
   {r:8,lbl:'8 m',dash:[],col:'rgba(99,130,180,0.12)',tc:'rgba(99,130,180,0.38)'}].forEach(({r,lbl,dash,col,tc})=>{
    const rPx=r*scale; if(rPx>Math.min(cw,ch)/2-4) return;
    radarCtx.setLineDash(dash); radarCtx.strokeStyle=col; radarCtx.lineWidth=1;
    radarCtx.beginPath(); radarCtx.arc(cx,cy,rPx,0,Math.PI*2); radarCtx.stroke();
    radarCtx.setLineDash([]); radarCtx.fillStyle=tc;
    radarCtx.font=r<2?'8px monospace':'9px sans-serif'; radarCtx.textAlign='left';
    radarCtx.fillText(lbl,cx+rPx+3,cy-3);
  });
  radarCtx.strokeStyle='rgba(99,130,180,0.18)'; radarCtx.lineWidth=1;
  radarCtx.beginPath(); radarCtx.moveTo(cx,4); radarCtx.lineTo(cx,ch-4); radarCtx.moveTo(4,cy); radarCtx.lineTo(cw-4,cy); radarCtx.stroke();
  const route=data.avoidance_route;
  if (data.obstacle_stop&&route?.length>=2) {
    radarCtx.save(); radarCtx.strokeStyle='#fbbf24'; radarCtx.lineWidth=2.5; radarCtx.setLineDash([7,4]);
    radarCtx.shadowColor='rgba(251,191,36,0.6)'; radarCtx.shadowBlur=10;
    radarCtx.beginPath(); radarCtx.moveTo(cx,cy);
    route.forEach(wp=>radarCtx.lineTo(cx-wp.y*scale,cy-wp.x*scale));
    radarCtx.stroke(); radarCtx.setLineDash([]); radarCtx.shadowBlur=0; radarCtx.fillStyle='#fbbf24';
    route.forEach((wp,i)=>{ const sx=cx-wp.y*scale,sy=cy-wp.x*scale; radarCtx.beginPath(); radarCtx.arc(sx,sy,i===route.length-1?5:3.5,0,Math.PI*2); radarCtx.fill(); });
    const side=route[0]?.side||(route[0]?.y>0?'left':'right');
    radarCtx.font='bold 10px sans-serif'; radarCtx.textAlign='center'; radarCtx.fillText(`DETOUR ${side.toUpperCase()}`,cx,18); radarCtx.restore();
  }
  if (data.points?.length) {
    radarCtx.fillStyle=data.obstacle_stop?'#f87171':'#34d399';
    data.points.forEach(p=>{ const sx=cx-p.y*scale,sy=cy-p.x*scale; radarCtx.fillRect(sx-1.5,sy-1.5,3,3); });
  }
  if (data.obstacle_stop) {
    radarCtx.save(); radarCtx.strokeStyle='rgba(248,113,113,0.50)'; radarCtx.lineWidth=1.5; radarCtx.setLineDash([3,3]);
    const stopR=0.35*scale; radarCtx.beginPath(); radarCtx.arc(cx,cy,stopR,-Math.PI*0.75,-Math.PI*0.25); radarCtx.stroke();
    radarCtx.setLineDash([]); radarCtx.fillStyle='#f87171'; radarCtx.font='bold 9px sans-serif'; radarCtx.textAlign='center';
    radarCtx.fillText('BLOCKED',cx,cy-stopR-5); radarCtx.restore();
  }
  const ROVER_L=0.520,ROVER_W=0.500,WHEEL_D=0.120,WHEEL_TRK=0.400,LIDAR_FWD=0.240;
  const hL=ROVER_L/2*scale,hW=ROVER_W/2*scale,wheelR=WHEEL_D/2*scale,wheelOff=WHEEL_TRK/2*scale;
  radarCtx.save();
  radarCtx.fillStyle='rgba(56,189,248,0.07)'; radarCtx.fillRect(cx-hW,cy-hL,hW*2,hL*2);
  radarCtx.strokeStyle='rgba(56,189,248,0.48)'; radarCtx.lineWidth=1.5; radarCtx.strokeRect(cx-hW,cy-hL,hW*2,hL*2);
  radarCtx.strokeStyle='#34d399'; radarCtx.lineWidth=2.5;
  radarCtx.shadowColor='rgba(52,211,153,0.45)'; radarCtx.shadowBlur=6;
  radarCtx.beginPath(); radarCtx.moveTo(cx-hW,cy-hL); radarCtx.lineTo(cx+hW,cy-hL); radarCtx.stroke(); radarCtx.shadowBlur=0;
  [['L',cx-wheelOff],['R',cx+wheelOff]].forEach(([lbl,wx])=>{
    radarCtx.fillStyle='rgba(100,116,139,0.22)'; radarCtx.strokeStyle='rgba(148,163,184,0.70)'; radarCtx.lineWidth=1.5;
    radarCtx.beginPath(); radarCtx.arc(wx,cy,wheelR,0,Math.PI*2); radarCtx.fill(); radarCtx.stroke();
    radarCtx.fillStyle='rgba(148,163,184,0.55)'; radarCtx.font='6px sans-serif'; radarCtx.textAlign='center'; radarCtx.fillText(lbl,wx,cy+2);
  });
  const lidarSY=cy-LIDAR_FWD*scale;
  radarCtx.fillStyle='#fbbf24'; radarCtx.shadowColor='#fbbf24'; radarCtx.shadowBlur=8;
  radarCtx.beginPath(); radarCtx.arc(cx,lidarSY,3.5,0,Math.PI*2); radarCtx.fill(); radarCtx.shadowBlur=0;
  radarCtx.fillStyle='rgba(251,191,36,0.65)'; radarCtx.font='6px monospace'; radarCtx.textAlign='left'; radarCtx.fillText('LiDAR',cx+5,lidarSY-4);
  radarCtx.fillStyle='#34d399'; radarCtx.font='bold 7px sans-serif'; radarCtx.textAlign='center'; radarCtx.fillText('▲ FRONT',cx,cy-hL-5);
  radarCtx.restore();
  radarCtx.fillStyle='#38bdf8'; radarCtx.shadowColor='#38bdf8'; radarCtx.shadowBlur=8;
  radarCtx.beginPath(); radarCtx.arc(cx,cy,2.5,0,Math.PI*2); radarCtx.fill(); radarCtx.shadowBlur=0;
}
function drawScene(data) {
  const cw=EL.sceneCanvas.width, ch=EL.sceneCanvas.height, maxRange=data.max_range||8.0;
  sceneCtx.clearRect(0,0,cw,ch);
  const grad=sceneCtx.createLinearGradient(0,0,0,ch); grad.addColorStop(0,'#0d1b2e'); grad.addColorStop(1,'#1a2a1a');
  sceneCtx.fillStyle=grad; sceneCtx.fillRect(0,0,cw,ch);
  const horizon=ch*0.55;
  sceneCtx.strokeStyle='rgba(56,189,248,0.15)'; sceneCtx.lineWidth=1;
  sceneCtx.beginPath(); sceneCtx.moveTo(0,horizon); sceneCtx.lineTo(cw,horizon); sceneCtx.stroke();
  sceneCtx.strokeStyle='rgba(56,189,248,0.06)';
  for(let i=0;i<20;i++){ const y=horizon+(i/20)*(ch-horizon); sceneCtx.beginPath(); sceneCtx.moveTo(0,y); sceneCtx.lineTo(cw,y); sceneCtx.stroke(); }
  if(data.points?.length){
    data.points.forEach(p=>{
      const dist=Math.hypot(p.x,p.y); if(dist<0.05||dist>maxRange) return;
      const angle=Math.atan2(p.y,p.x), screenX=cw*(0.5-angle/(Math.PI/2)*0.55);
      const relDist=dist/maxRange, screenY=horizon-(1-relDist)*horizon*0.9;
      const dotSize=Math.max(1.5,(1-relDist)*8), isObs=data.obstacle_stop&&Math.abs(angle)<0.44&&dist<0.45;
      sceneCtx.fillStyle=isObs?'#f87171':'#34d399';
      sceneCtx.globalAlpha=0.5+relDist*0.5;
      sceneCtx.fillRect(screenX-dotSize/2,screenY-dotSize/2,dotSize,dotSize);
    }); sceneCtx.globalAlpha=1;
  }
  if(data.obstacle_stop&&data.avoidance_route?.length){
    const side=data.avoidance_route[0]?.side||(data.avoidance_route[0]?.y>0?'left':'right');
    sceneCtx.fillStyle='#fbbf24'; sceneCtx.font='bold 14px sans-serif';
    sceneCtx.globalAlpha=0.90; sceneCtx.fillText(side==='left'?'← DETOUR LEFT':'DETOUR RIGHT →',cw/2-64,horizon-20); sceneCtx.globalAlpha=1;
  }
}

/* ── Tuning panel ────────────────────────────────────────────────────────── */
const EL_tuningRows=document.getElementById('tuning-rows'), EL_tuningFeedback=document.getElementById('tuning-feedback'), EL_tuningReset=document.getElementById('tuningResetBtn');
let _tuningSettings={};
async function loadTuningSettings() {
  try { const data=await api('/api/settings'); _tuningSettings=data.settings||{}; renderTuningRows(); }
  catch(e) { if(EL_tuningRows) EL_tuningRows.innerHTML=`<div class="hint">Error: ${e.message}</div>`; }
}
function renderTuningRows() {
  if(!EL_tuningRows) return; EL_tuningRows.innerHTML='';
  Object.entries(_tuningSettings).forEach(([key,spec])=>{
    const row=document.createElement('div'); row.className='tuning-row';
    const lbl=document.createElement('div'); lbl.className='tuning-label'; lbl.title=spec.description||''; lbl.textContent=spec.label;
    const unit=document.createElement('span'); unit.className='tuning-unit'; unit.textContent=spec.unit?` ${spec.unit}`:'';
    const inp=document.createElement('input'); inp.type='number'; inp.className='tuning-input';
    inp.min=spec.min; inp.max=spec.max;
    inp.step=spec.unit==='m'||spec.unit==='m/s'?'0.01':spec.unit==='s'?'0.5':'0.05';
    inp.value=Number(spec.value).toFixed(spec.unit==='s'?1:2); inp.dataset.key=key; inp.dataset.default=spec.default;
    const dot=document.createElement('span'); dot.className='tuning-dot';
    dot.style.visibility=Math.abs(spec.value-spec.default)>0.0001?'visible':'hidden'; dot.title=`Default: ${spec.default} ${spec.unit}`;
    const applyFn=async()=>{
      const val=parseFloat(inp.value); if(isNaN(val)) return;
      try {
        const r=await api('/api/settings','PATCH',{key,value:val});
        if(r.ok){ _tuningSettings[key].value=r.value; inp.value=Number(r.value).toFixed(inp.step.includes('0.5')?1:2); dot.style.visibility=Math.abs(r.value-spec.default)>0.0001?'visible':'hidden'; _flashFeedback(`✓ ${spec.label} = ${r.value} ${spec.unit}`,'ok'); }
      } catch(err){ _flashFeedback(`✗ ${err.message}`,'err'); inp.value=Number(_tuningSettings[key].value).toFixed(inp.step.includes('0.5')?1:2); }
    };
    inp.addEventListener('keydown',e=>{ if(e.key==='Enter'){ e.preventDefault(); applyFn(); } });
    inp.addEventListener('blur',applyFn);
    const ctrl=document.createElement('div'); ctrl.className='tuning-ctrl'; ctrl.appendChild(inp); ctrl.appendChild(unit); ctrl.appendChild(dot);
    row.appendChild(lbl); row.appendChild(ctrl); EL_tuningRows.appendChild(row);
  });
}
function _flashFeedback(msg,type) {
  if(!EL_tuningFeedback) return; EL_tuningFeedback.textContent=msg; EL_tuningFeedback.className=`tuning-feedback ${type}`;
  clearTimeout(EL_tuningFeedback._t); EL_tuningFeedback._t=setTimeout(()=>{ EL_tuningFeedback.textContent=''; EL_tuningFeedback.className='tuning-feedback'; },3000);
}
EL_tuningReset&&EL_tuningReset.addEventListener('click',async()=>{
  if(!confirm('Reset all tuning values to defaults?')) return;
  try { const r=await api('/api/settings','DELETE'); if(r.ok){ _tuningSettings=r.settings; renderTuningRows(); _flashFeedback('✓ Reset to defaults','ok'); } }
  catch(e){ _flashFeedback(`✗ ${e.message}`,'err'); }
});

/* ── Sequential polling loops ────────────────────────────────────────────── */
function seqLoop(fn,ms){ async function run(){ try{ await fn(); }catch(_){} setTimeout(run,ms); } run(); }

/* ── Init ────────────────────────────────────────────────────────────────── */
_fitCanvas(); scheduleMapRender();
seqLoop(refreshStatus, 450);
seqLoop(refreshMap, 8000);
seqLoop(refreshLidar, 260);
seqLoop(refreshCameraStatus, 2000);
seqLoop(refreshPois, 5000);
refreshMapList(); setInterval(refreshMapList, 10000);
setTool('goal'); updateMissionButton(); loadTuningSettings();
if (typeof ResizeObserver !== 'undefined')
  new ResizeObserver(()=>{ _fitCanvas(); scheduleMapRender(); }).observe(EL.mapOuter);
