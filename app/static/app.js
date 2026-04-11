/**
 * Arespath Rover — Dashboard app.js  v3
 *
 * Features
 * ────────
 * • D-pad + keyboard hold-to-drive with 100 ms repeat; release → STOP
 * • Lidar radar draws obstacle-free detour route when blocked
 * • Ring-buffer snapshot toggle (auto, 10 s interval, max 10/cam)
 * • Permanent photo capture (both cameras, stored forever)
 * • Gallery modals for snapshots (tabbed by camera) and photos (all)
 * • Lightbox on image click
 */

'use strict';

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
  mapStartBtn:     el('mapStartBtn'),
  mapStopBtn:      el('mapStopBtn'),
  mapResetBtn:     el('mapResetBtn'),
  saveMapBtn:      el('saveMapBtn'),
  loadMapBtn:      el('loadMapBtn'),
  cancelNavBtn:    el('cancelNavBtn'),
  mapName:         el('mapName'),
  mapSelect:       el('mapSelect'),
  mapCanvas:       el('mapCanvas'),
  radarCanvas:     el('radarCanvas'),
  sceneCanvas:     el('sceneCanvas'),
  wsPill:          el('ws-pill'),
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

// ── Speed helper ──────────────────────────────────────────────────────────────
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
    left:    { linear:  0,  angular: -sp },
    right:   { linear:  0,  angular:  sp },
    stop:    { linear:  0,  angular:  0 },
  };
  return map[cmd] || { linear: 0, angular: 0 };
}

// ── D-pad hold-to-drive with 100 ms repeat ────────────────────────────────────
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

window.addEventListener('blur', () => { _stopRepeat(); sendStop(); });
document.addEventListener('visibilitychange', () => {
  if (document.hidden) { _stopRepeat(); sendStop(); }
});

// ── Arm / Stop ────────────────────────────────────────────────────────────────
EL.armBtn.addEventListener('click', async () => {
  try {
    await api('/api/arm', 'POST', { armed: !STATE.armed });
    await refreshStatus();
  } catch (e) { setText('st-error', e.message); }
});
EL.stopBtn.addEventListener('click', () => sendStop());

// ── Mode buttons ──────────────────────────────────────────────────────────────
EL.pilotBtn.addEventListener('click', () =>
  api('/api/mode', 'POST', { mode: 'pilot' }).then(refreshStatus).catch(() => {}));
EL.missionBtn.addEventListener('click', () =>
  api('/api/mode', 'POST', { mode: 'mission' }).then(refreshStatus).catch(() => {}));

// ── Map tool buttons ──────────────────────────────────────────────────────────
EL.toolGoalBtn.addEventListener('click', () => setTool('goal'));
EL.toolStartBtn.addEventListener('click', () => setTool('start'));

function setTool(tool) {
  STATE.mapTool = tool;
  setText('tool-label', tool.toUpperCase());
  EL.toolGoalBtn.classList.toggle('active', tool === 'goal');
  EL.toolStartBtn.classList.toggle('active', tool === 'start');
}

// ── Map controls ──────────────────────────────────────────────────────────────
EL.mapStartBtn.addEventListener('click', () =>
  api('/api/map/start', 'POST', { clear: false }).catch(() => {}));
EL.mapStopBtn.addEventListener('click', () =>
  api('/api/map/stop', 'POST', {}).catch(() => {}));
EL.mapResetBtn.addEventListener('click', () =>
  api('/api/map/reset', 'POST', {}).catch(() => {}));
EL.saveMapBtn.addEventListener('click', () =>
  api('/api/map/save', 'POST', { name: EL.mapName.value || 'map' })
    .then(refreshMapList).catch(() => {}));
EL.loadMapBtn.addEventListener('click', () => {
  const name = EL.mapSelect.value;
  if (name) api('/api/map/load', 'POST', { name }).catch(() => {});
});
EL.cancelNavBtn.addEventListener('click', () =>
  api('/api/navigate/cancel', 'POST', {}).catch(() => {}));

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
let _galleryMode   = 'snapshots'; // 'snapshots' | 'photos'
let _galleryData   = {};          // raw API response
let _galleryTab    = 'front';     // active tab for snapshots

function _fmtTime(ts) {
  const d = new Date(ts * 1000);
  return d.toLocaleString('en-GB', {
    day: '2-digit', month: 'short', hour: '2-digit', minute: '2-digit', second: '2-digit'
  });
}

function _fmtSize(bytes) {
  if (bytes < 1024) return `${bytes} B`;
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
  return `${(bytes / 1024 / 1024).toFixed(2)} MB`;
}

// Open snapshot gallery
EL.viewSnapshotsBtn.addEventListener('click', async () => {
  _galleryMode = 'snapshots';
  _galleryTab  = 'front';
  EL.modalTitle.textContent = '📷 Auto Snapshots';
  EL.modalSub.textContent   = 'Ring-buffer — newest first — max 10 per camera';
  EL.modalTabs.style.display = '';
  EL.modalGallery.innerHTML  = '<div class="modal-empty">Loading…</div>';
  EL.modalOverlay.style.display = 'flex';
  try {
    const data = await api('/api/snapshot/files');
    _galleryData = data.files || {};
    _setActiveTab(_galleryTab);
    _renderSnapshots();
  } catch (e) {
    EL.modalGallery.innerHTML = `<div class="modal-empty">Error: ${e.message}</div>`;
  }
});

// Open photo gallery
EL.viewPhotosBtn.addEventListener('click', async () => {
  _galleryMode = 'photos';
  EL.modalTitle.textContent  = '🗂 Permanent Photos';
  EL.modalSub.textContent    = 'All captured photos — stored permanently';
  EL.modalTabs.style.display = 'none';
  EL.modalGallery.innerHTML  = '<div class="modal-empty">Loading…</div>';
  EL.modalOverlay.style.display = 'flex';
  try {
    const data = await api('/api/photo/files');
    _renderPhotos(data.files || []);
  } catch (e) {
    EL.modalGallery.innerHTML = `<div class="modal-empty">Error: ${e.message}</div>`;
  }
});

// Tab switching
EL.modalTabs.querySelectorAll('.modal-tab').forEach(tab => {
  tab.addEventListener('click', () => {
    _setActiveTab(tab.dataset.tab);
    _renderSnapshots();
  });
});

function _setActiveTab(tab) {
  _galleryTab = tab;
  EL.modalTabs.querySelectorAll('.modal-tab').forEach(t => {
    t.classList.toggle('active', t.dataset.tab === tab);
  });
}

function _renderSnapshots() {
  const files = _galleryData;
  let items = [];
  if (_galleryTab === 'all') {
    // Merge all cameras, sort by filename (timestamp embedded)
    const all = [];
    Object.entries(files).forEach(([cam, fnames]) => {
      fnames.forEach(f => all.push({ cam, f }));
    });
    all.sort((a, b) => b.f.localeCompare(a.f));
    items = all;
  } else {
    items = (files[_galleryTab] || []).map(f => ({ cam: _galleryTab, f }));
  }

  if (!items.length) {
    EL.modalGallery.innerHTML = '<div class="modal-empty">No snapshots yet. Start snapshot capture to begin.</div>';
    return;
  }
  EL.modalGallery.innerHTML = '';
  const grid = document.createElement('div');
  grid.className = 'gallery-grid';
  items.forEach(({ cam, f }) => {
    const ts = _parseTs(f);
    grid.appendChild(_makeCard({
      src:      `/snapshots/${f}`,
      dlHref:   `/snapshots/${f}?dl=1`,
      dlName:   f,
      metaTop:  `<strong>${cam}</strong>`,
      metaBot:  ts ? _fmtTime(ts) : f,
    }));
  });
  EL.modalGallery.appendChild(grid);
}

function _renderPhotos(photos) {
  if (!photos.length) {
    EL.modalGallery.innerHTML = '<div class="modal-empty">No photos taken yet. Use "Take Photo" to capture.</div>';
    return;
  }
  EL.modalGallery.innerHTML = '';
  const grid = document.createElement('div');
  grid.className = 'gallery-grid';
  photos.forEach(p => {
    grid.appendChild(_makeCard({
      src:      `/photos/${p.filename}`,
      dlHref:   `/photos/${p.filename}?dl=1`,
      dlName:   p.filename,
      metaTop:  `<strong>${p.cam}</strong> · ${_fmtSize(p.size)}`,
      metaBot:  _fmtTime(p.ts),
    }));
  });
  EL.modalGallery.appendChild(grid);
}

function _makeCard({ src, dlHref, dlName, metaTop, metaBot }) {
  const card = document.createElement('div');
  card.className = 'gallery-card';

  const img = document.createElement('img');
  img.className = 'gallery-img';
  img.src = src;
  img.alt = dlName;
  img.loading = 'lazy';
  img.addEventListener('click', () => _openLightbox(src));

  const foot = document.createElement('div');
  foot.className = 'gallery-card-foot';

  const meta = document.createElement('div');
  meta.className = 'gallery-meta';
  meta.innerHTML = `${metaTop}<br>${metaBot}`;

  const dlLink = document.createElement('a');
  dlLink.className = 'gallery-dl';
  dlLink.href = dlHref;
  dlLink.download = dlName;
  dlLink.textContent = '⬇ Download';

  foot.appendChild(meta);
  foot.appendChild(dlLink);
  card.appendChild(img);
  card.appendChild(foot);
  return card;
}

function _parseTs(filename) {
  // filename pattern: <cam>_<ts>.jpg or photo_<cam>_<ts>.jpg
  const m = filename.match(/_(\d{9,13})\.jpg$/);
  return m ? parseInt(m[1], 10) : null;
}

// Close modal
EL.modalClose.addEventListener('click', () => { EL.modalOverlay.style.display = 'none'; });
EL.modalOverlay.addEventListener('click', e => {
  if (e.target === EL.modalOverlay) EL.modalOverlay.style.display = 'none';
});
document.addEventListener('keydown', e => {
  if (e.key === 'Escape') EL.modalOverlay.style.display = 'none';
});

// ── Lightbox ──────────────────────────────────────────────────────────────────
function _openLightbox(src) {
  const lb = document.createElement('div');
  lb.className = 'lightbox';
  const img = document.createElement('img');
  img.src = src;
  lb.appendChild(img);
  lb.addEventListener('click', () => lb.remove());
  document.addEventListener('keydown', function esc(e) {
    if (e.key === 'Escape') { lb.remove(); document.removeEventListener('keydown', esc); }
  });
  document.body.appendChild(lb);
}

// ── Map canvas interactions ────────────────────────────────────────────────────
EL.mapCanvas.addEventListener('pointerdown', e => {
  if (!STATE.mapMeta) return;
  STATE.dragStart = canvasToWorld(e, STATE.mapMeta);
});
EL.mapCanvas.addEventListener('pointermove', e => {
  if (!STATE.dragStart || !STATE.mapMeta || STATE.mapTool !== 'start') return;
  const cur = canvasToWorld(e, STATE.mapMeta);
  STATE.headingPrev = Math.atan2(cur.y - STATE.dragStart.y, cur.x - STATE.dragStart.x);
});
EL.mapCanvas.addEventListener('pointerup', e => {
  if (!STATE.mapMeta) return;
  const pt = canvasToWorld(e, STATE.mapMeta);
  if (STATE.mapTool === 'goal') {
    api('/api/navigate/goal', 'POST', { x: pt.x, y: pt.y }).catch(() => {});
  } else {
    const start = STATE.dragStart || pt;
    api('/api/pose', 'POST', { x: start.x, y: start.y, theta: STATE.headingPrev }).catch(() => {});
  }
  STATE.dragStart = null;
});

function canvasToWorld(e, meta) {
  const rect = EL.mapCanvas.getBoundingClientRect();
  const cx = (e.clientX - rect.left) / rect.width;
  const cy = (e.clientY - rect.top)  / rect.height;
  const gx = Math.round(cx * meta.width);
  const gy = Math.round(cy * meta.height);
  return {
    x: gx * meta.resolution + meta.origin[0],
    y: gy * meta.resolution + meta.origin[1],
  };
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
  setText('st-safety',    s.obstacle_stop ? 'BLOCKED' : 'CLEAR');
  setText('st-battery',   s.battery_v != null ? `${s.battery_v.toFixed(2)} V` : '--');
  setText('st-nav',       s.nav?.status || '--');
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
async function refreshMap() {
  try {
    const meta = await api('/api/map/data');
    STATE.mapMeta = meta;
    drawMap(meta);
  } catch (_) {}
}

async function drawMap(meta) {
  const cw = EL.mapCanvas.width, ch = EL.mapCanvas.height;
  mapCtx.clearRect(0, 0, cw, ch);
  mapCtx.fillStyle = '#1a2236';
  mapCtx.fillRect(0, 0, cw, ch);

  if (meta.image_png_b64) {
    const img = new Image();
    await new Promise(res => { img.onload = res; img.src = 'data:image/png;base64,' + meta.image_png_b64; });
    mapCtx.drawImage(img, 0, 0, cw, ch);
  }

  const toC = p => ({ x: (p.x / meta.width) * cw, y: (p.y / meta.height) * ch });

  // Grid
  const step = Math.max(10, Math.round(1.0 / meta.resolution));
  mapCtx.save();
  mapCtx.strokeStyle = 'rgba(255,255,255,0.05)'; mapCtx.lineWidth = 1;
  for (let x = 0; x <= meta.width; x += step) {
    const px = (x / meta.width) * cw;
    mapCtx.beginPath(); mapCtx.moveTo(px, 0); mapCtx.lineTo(px, ch); mapCtx.stroke();
  }
  for (let y = 0; y <= meta.height; y += step) {
    const py = (y / meta.height) * ch;
    mapCtx.beginPath(); mapCtx.moveTo(0, py); mapCtx.lineTo(cw, py); mapCtx.stroke();
  }
  mapCtx.restore();

  // Scan points
  if (meta.scan?.length) {
    mapCtx.fillStyle = 'rgba(52,211,153,0.85)';
    meta.scan.forEach(p => { const c = toC(p); mapCtx.fillRect(c.x-1.5, c.y-1.5, 3, 3); });
  }

  // Path
  if (meta.path?.length > 1) {
    mapCtx.save();
    mapCtx.strokeStyle = '#38bdf8'; mapCtx.lineWidth = 3;
    mapCtx.shadowColor = 'rgba(56,189,248,0.4)'; mapCtx.shadowBlur = 12;
    mapCtx.beginPath();
    meta.path.forEach((p, i) => {
      const c = toC(p); if (i === 0) mapCtx.moveTo(c.x, c.y); else mapCtx.lineTo(c.x, c.y);
    });
    mapCtx.stroke(); mapCtx.restore();
  }

  // Goal
  if (meta.goal) {
    const c = toC(meta.goal);
    mapCtx.save();
    mapCtx.strokeStyle = '#fbbf24'; mapCtx.lineWidth = 2.5;
    mapCtx.shadowColor = '#fbbf24'; mapCtx.shadowBlur = 10;
    mapCtx.beginPath(); mapCtx.arc(c.x, c.y, 8, 0, Math.PI * 2); mapCtx.stroke();
    mapCtx.beginPath();
    mapCtx.moveTo(c.x, c.y-14); mapCtx.lineTo(c.x, c.y+14);
    mapCtx.moveTo(c.x-14, c.y); mapCtx.lineTo(c.x+14, c.y);
    mapCtx.stroke(); mapCtx.restore();
  }

  // Robot
  if (meta.pose) {
    const c = toC(meta.pose); const th = meta.pose.theta; const r = 10;
    mapCtx.save();
    mapCtx.translate(c.x, c.y); mapCtx.rotate(th);
    mapCtx.fillStyle = '#38bdf8';
    mapCtx.shadowColor = 'rgba(56,189,248,0.6)'; mapCtx.shadowBlur = 14;
    mapCtx.beginPath();
    mapCtx.moveTo(r, 0); mapCtx.lineTo(-r*0.7, -r*0.6); mapCtx.lineTo(-r*0.7, r*0.6);
    mapCtx.closePath(); mapCtx.fill(); mapCtx.restore();
  }
}

// ── Lidar render ──────────────────────────────────────────────────────────────
async function refreshLidar() {
  try {
    const data = await api('/api/lidar');
    drawRadar(data);
    drawScene(data);
    // Avoidance badge
    const hasRoute = data.obstacle_stop && data.avoidance_route?.length > 0;
    EL.avoidBadge.style.display = hasRoute ? '' : 'none';
  } catch (_) {}
}

function drawRadar(data) {
  const cw = EL.radarCanvas.width, ch = EL.radarCanvas.height;
  const cx = cw / 2, cy = ch / 2;
  const maxRange = data.max_range || 8.0;
  const scale = (Math.min(cw, ch) / 2 - 14) / maxRange;

  radarCtx.clearRect(0, 0, cw, ch);
  radarCtx.fillStyle = '#0d1422';
  radarCtx.fillRect(0, 0, cw, ch);

  // Range rings
  radarCtx.lineWidth = 1;
  [2, 4, 6, 8].forEach(r => {
    radarCtx.strokeStyle = 'rgba(99,130,180,0.18)';
    radarCtx.beginPath(); radarCtx.arc(cx, cy, r * scale, 0, Math.PI * 2); radarCtx.stroke();
    radarCtx.fillStyle = 'rgba(99,130,180,0.50)';
    radarCtx.font = '9px sans-serif';
    radarCtx.fillText(`${r}m`, cx + r * scale + 2, cy - 2);
  });

  // Crosshairs
  radarCtx.strokeStyle = 'rgba(99,130,180,0.22)';
  radarCtx.beginPath();
  radarCtx.moveTo(cx, 5);    radarCtx.lineTo(cx, ch-5);
  radarCtx.moveTo(5, cy);    radarCtx.lineTo(cw-5, cy);
  radarCtx.stroke();

  // ── Avoidance route ────────────────────────────────────────────────────────
  // Robot frame: x=forward, y=left (+y = left of robot)
  // On radar canvas: forward = UP (−y on screen), left = LEFT (−x on screen)
  //   screen_x = cx  − robot_y * scale   (left on robot → left on screen)
  //   screen_y = cy  − robot_x * scale   (forward on robot → up on screen)
  const route = data.avoidance_route;
  if (data.obstacle_stop && route?.length >= 2) {
    radarCtx.save();
    radarCtx.strokeStyle = '#fbbf24';
    radarCtx.lineWidth = 2.5;
    radarCtx.setLineDash([7, 4]);
    radarCtx.shadowColor = 'rgba(251,191,36,0.6)';
    radarCtx.shadowBlur  = 10;
    radarCtx.beginPath();
    radarCtx.moveTo(cx, cy);  // robot origin
    route.forEach(wp => {
      const sx = cx - wp.y * scale;
      const sy = cy - wp.x * scale;
      radarCtx.lineTo(sx, sy);
    });
    radarCtx.stroke();
    radarCtx.setLineDash([]);
    radarCtx.shadowBlur = 0;

    // Waypoint dots
    radarCtx.fillStyle = '#fbbf24';
    route.forEach((wp, i) => {
      const sx = cx - wp.y * scale;
      const sy = cy - wp.x * scale;
      radarCtx.beginPath();
      radarCtx.arc(sx, sy, i === route.length - 1 ? 5 : 3.5, 0, Math.PI * 2);
      radarCtx.fill();
    });

    // Direction label
    const side = route[0]?.side || (route[0]?.y > 0 ? 'left' : 'right');
    radarCtx.fillStyle = '#fbbf24';
    radarCtx.font = 'bold 10px sans-serif';
    radarCtx.fillText(`DETOUR ${side.toUpperCase()}`, cx - 36, 18);
    radarCtx.restore();
  }

  // ── Scan points ────────────────────────────────────────────────────────────
  if (data.points?.length) {
    // Scan points from API are cartesian {x, y} in robot frame already
    // (x=forward, y=left from lidar.scan_cartesian)
    radarCtx.fillStyle = data.obstacle_stop ? '#f87171' : '#34d399';
    data.points.forEach(p => {
      const sx = cx - p.y * scale;   // y=left → left on screen
      const sy = cy - p.x * scale;   // x=forward → up on screen
      radarCtx.fillRect(sx - 1.5, sy - 1.5, 3, 3);
    });
  }

  // Obstacle stop zone arc
  if (data.obstacle_stop) {
    radarCtx.save();
    radarCtx.strokeStyle = 'rgba(248,113,113,0.5)';
    radarCtx.lineWidth = 1.5;
    radarCtx.setLineDash([3, 3]);
    const stopR = 0.35 * scale;
    // Arc in front of robot (upward on canvas = forward direction)
    radarCtx.beginPath();
    radarCtx.arc(cx, cy, stopR, -Math.PI * 0.75, -Math.PI * 0.25);
    radarCtx.stroke();
    radarCtx.setLineDash([]);
    radarCtx.fillStyle = '#f87171';
    radarCtx.font = 'bold 9px sans-serif';
    radarCtx.fillText('BLOCKED', cx - 22, cy - stopR - 5);
    radarCtx.restore();
  }

  // Robot dot
  radarCtx.fillStyle = '#38bdf8';
  radarCtx.shadowColor = '#38bdf8'; radarCtx.shadowBlur = 8;
  radarCtx.beginPath(); radarCtx.arc(cx, cy, 5, 0, Math.PI * 2); radarCtx.fill();
  radarCtx.shadowBlur = 0;

  // "UP = FORWARD" label
  radarCtx.fillStyle = 'rgba(99,130,180,0.45)';
  radarCtx.font = '8px sans-serif';
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
      // p.x = forward, p.y = left (robot frame)
      const dist = Math.hypot(p.x, p.y);
      if (dist < 0.05 || dist > maxRange) return;
      // angle from forward axis
      const angle = Math.atan2(p.y, p.x);
      const normAngle = angle / (Math.PI / 2);
      const screenX = cw * (0.5 - normAngle * 0.55); // left=+y → left on screen
      const relDist  = dist / maxRange;
      const screenY  = horizon - (1 - relDist) * horizon * 0.9;
      const dotSize  = Math.max(1.5, (1 - relDist) * 8);
      const isObs    = data.obstacle_stop && Math.abs(angle) < 0.44 && dist < 0.45;
      sceneCtx.fillStyle    = isObs ? '#f87171' : '#34d399';
      sceneCtx.globalAlpha  = 0.5 + relDist * 0.5;
      sceneCtx.fillRect(screenX - dotSize/2, screenY - dotSize/2, dotSize, dotSize);
    });
    sceneCtx.globalAlpha = 1;
  }

  // Detour label on scene
  if (data.obstacle_stop && data.avoidance_route?.length) {
    const side = data.avoidance_route[0]?.side || (data.avoidance_route[0]?.y > 0 ? 'left' : 'right');
    const arrow = side === 'left' ? '← DETOUR LEFT' : 'DETOUR RIGHT →';
    sceneCtx.fillStyle    = '#fbbf24';
    sceneCtx.font         = 'bold 14px sans-serif';
    sceneCtx.globalAlpha  = 0.90;
    sceneCtx.fillText(arrow, cw / 2 - 64, horizon - 20);
    sceneCtx.globalAlpha = 1;
  }
}

// ── Poll loops ────────────────────────────────────────────────────────────────
function debounced(fn, ms) {
  let timer = null;
  const run = async () => {
    try { await fn(); } catch (_) {}
    timer = setTimeout(run, ms);
  };
  run();
}

debounced(refreshStatus,       450);
debounced(refreshMap,          900);
debounced(refreshLidar,        260);
debounced(refreshCameraStatus, 2000);
refreshMapList();
setInterval(refreshMapList, 10000);
