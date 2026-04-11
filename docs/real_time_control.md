# Real-Time Control Architecture

## System Architecture Diagram

```
Browser (Socket.IO / REST)
        │
        │  manual_command {cmd, speed}
        │  heartbeat {}
        ▼
┌─────────────────────────────────────────┐
│          Flask + Flask-SocketIO         │  ← 0.0.0.0:8080
│  /api/manual  /api/arm  /api/stop  …   │
└───────────────┬─────────────────────────┘
                │ set_manual_command()
                │ force_stop()
                ▼
┌─────────────────────────────────────────┐
│           RobotRuntime                  │
│  ┌─────────────────────┐               │
│  │  Control loop 20 Hz │               │
│  │  odometry update    │               │
│  │  map update         │               │
│  │  obstacle check     │               │
│  │  nav step / manual  │               │
│  └────────┬────────────┘               │
│           │ push_manual / push_nav     │
│           ▼                            │
│  ┌─────────────────────┐               │
│  │  DriveCommandQueue  │               │
│  │  cmd-worker 50 Hz   │               │
│  │  manual priority    │               │
│  │  idle watchdog      │               │
│  └────────┬────────────┘               │
└───────────┼─────────────────────────────┘
            │ set_pwm / stop (serial)
            ▼
┌─────────────────────────────────────────┐
│  ArduinoBridge (serial-reader thread)   │
│  non-blocking reads / locked writes     │
└───────────────────────┬─────────────────┘
                        │  USB serial 115200
                        ▼
┌─────────────────────────────────────────┐
│  Arduino Uno + BTS7960                  │
│  firmware watchdog 350 ms               │
│  encoder ISRs + 120 ms telemetry        │
└─────────────────────────────────────────┘
```

## Timing Table

| Layer | Rate | Thread | Purpose |
|-------|------|--------|---------|
| Frontend heartbeat | 1 Hz | Browser | Server watchdog reset |
| Frontend status poll | ~2.2 Hz | Browser | UI telemetry update |
| Frontend lidar poll | ~3.8 Hz | Browser | Radar/scene render |
| Frontend map poll | ~1.1 Hz | Browser | Occupancy map render |
| Control loop | 20 Hz | `robot-ctrl` | Odometry, nav, obstacle |
| Command worker | 50 Hz | `cmd-worker` | Serial flush + rate limit |
| Arduino telemetry | ~8 Hz | Arduino | TEL frame on serial |
| Arduino watchdog | 350 ms timeout | Arduino | Failsafe stop |
| Server heartbeat watchdog | 3 s timeout | `robot-monitor` | Force stop if client lost |
| Serial reconnect | 0.5 s poll | `robot-monitor` | Auto-reconnect on drop |

## Fail-Safe Layers

1. **Frontend**: key/pointer release → `sendStop()` immediately; page blur → `sendStop()`.
2. **Socket.IO**: disconnect event → server calls `force_stop()`.
3. **Heartbeat watchdog**: if no heartbeat for 3 s → server calls `push_stop()`.
4. **Manual command timeout**: manual command auto-expires in 400 ms without refresh.
5. **DriveCommandQueue idle watchdog**: if nothing pushed for 500 ms → STOP sent.
6. **Arduino watchdog**: 350 ms with no command → `FAILSAFE_STOP`.

## Code Snippets

### Hold-to-drive pattern (frontend)

```javascript
// D-pad buttons: send command on pointerdown, stop on pointerup
btn.addEventListener('pointerdown',  () => sendCommand(cmd));
btn.addEventListener('pointerup',    () => sendStop());
btn.addEventListener('pointercancel',() => sendStop());
btn.addEventListener('pointerleave', () => sendStop());

// Keyboard: keydown starts, keyup stops. e.repeat ignored.
window.addEventListener('keydown', e => {
  if (e.repeat) return;
  const cmd = KEY_CMD[e.key.toLowerCase()];
  if (cmd) sendCommand(cmd);
});
window.addEventListener('keyup', e => {
  const cmd = KEY_CMD[e.key.toLowerCase()];
  if (cmd) sendStop();
});
```

### Socket.IO manual command handler (server)

```python
@socketio.on("manual_command")
def on_manual_command(data):
    cmd   = str(data.get("cmd", "stop")).lower()
    speed = max(0.0, min(100.0, float(data.get("speed", 55)))) / 100.0
    if cmd == "stop":
        runtime.force_stop()
    else:
        lin, ang = SPEED_MAP[cmd]
        runtime.set_manual_command(lin * speed, ang * speed)
```

### DriveCommandQueue worker loop

```python
def _work_loop(self):
    while self._running:
        with self._cond:
            self._cond.wait(timeout=WATCHDOG_IDLE_S)
            stop_req = self._stop_requested
            manual   = self._manual_cmd
            nav      = self._nav_cmd
            idle     = (time.time() - self._last_push_time) >= WATCHDOG_IDLE_S
            # consume
            self._stop_requested = False
            self._manual_cmd = None
            self._nav_cmd    = None

        # rate limit
        elapsed = time.time() - self._last_send_time
        if elapsed < MIN_SEND_INTERVAL:
            time.sleep(MIN_SEND_INTERVAL - elapsed)

        if stop_req or idle:
            self._bridge.stop()
        elif manual is not None:
            self._bridge.set_pwm(*manual)   # manual always wins
        elif nav is not None:
            self._bridge.set_pwm(*nav)
```

### Arduino watchdog

```cpp
void checkWatchdog() {
    if (failsafeActive) return;
    if ((currentLeftPWM != 0 || currentRightPWM != 0) &&
        (millis() - lastCmdMs > WATCHDOG_MS)) {
        stopAll();
        failsafeActive = true;
        Serial.println(F("FAILSAFE_STOP"));
    }
}
```

### Serial command handling (Arduino)

```cpp
// Every command updates lastCmdMs to reset watchdog
lastCmdMs = millis();
failsafeActive = false;

if (strcmp(cmd, "SET") == 0) {
    setLeft(constrain(atoi(ls), -255, 255));
    setRight(constrain(atoi(rs), -255, 255));
    Serial.println(F("ACK SET"));
}
```
