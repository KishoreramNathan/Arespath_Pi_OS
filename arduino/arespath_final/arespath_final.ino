/*
  Arespath Rover — Arduino Uno firmware
  ══════════════════════════════════════
  Hardware:  Arduino Uno R3 + dual BTS7960 motor driver

  Serial protocol (115200 baud, newline-terminated):
    PING           → PONG ARESPATH
    SET <l> <r>    → set left/right PWM  (-255 … 255)
    STOP           → stop both motors immediately
    RESET_ODOM     → zero encoder tick counters
    F              → forward  at DEFAULT_SPEED
    B              → backward at DEFAULT_SPEED
    L              → spin left  at DEFAULT_SPEED
    R              → spin right at DEFAULT_SPEED
    S              → stop (alias)

  Telemetry every 120 ms:
    TEL,<ms>,<left_ticks>,<right_ticks>,<left_rpm>,<right_rpm>,<left_pwm>,<right_pwm>,<batt_v>

  Watchdog:
    If no command is received for 350 ms → FAILSAFE_STOP (motors halted).

  Pinout:
    Left  BTS7960  RPWM=5  LPWM=6  R_EN=7  L_EN=8
    Right BTS7960  RPWM=9  LPWM=10 R_EN=11 L_EN=12
    Left  encoder  A=2 (INT0), B=A0
    Right encoder  A=3 (INT1), B=A1
    Battery sense  A2 (optional, 4:1 divider → 0-20 V range)
*/

// ── Pin definitions ───────────────────────────────────────────────────────────
const uint8_t L_RPWM = 5, L_LPWM = 6, L_REN = 7,  L_LEN = 8;
const uint8_t R_RPWM = 9, R_LPWM = 10, R_REN = 11, R_LEN = 12;
const uint8_t ENC_L_A = 2, ENC_L_B = A0;
const uint8_t ENC_R_A = 3, ENC_R_B = A1;
const uint8_t BATT_PIN = A2;

// ── Tuneable constants ────────────────────────────────────────────────────────
const float   TICKS_PER_REV     = 420.0f;
const uint16_t TELEMETRY_MS     = 120;
const uint16_t WATCHDOG_MS      = 350;
const uint8_t  DEFAULT_SPEED    = 140;
// Battery divider: 10K + 3.3K → scale = (10+3.3)/3.3 * (5.0/1023)
const float   BATT_SCALE        = (13.3f / 3.3f) * (5.0f / 1023.0f);

// ── Runtime state ─────────────────────────────────────────────────────────────
volatile long leftCount  = 0;
volatile long rightCount = 0;

int currentLeftPWM  = 0;
int currentRightPWM = 0;

unsigned long lastTelMs  = 0;
unsigned long lastCmdMs  = 0;    // watchdog reference
long prevLeftCount  = 0;
long prevRightCount = 0;
bool failsafeActive = false;

char   serialBuf[72];
uint8_t serialIdx = 0;

// ── Motor helpers ─────────────────────────────────────────────────────────────
void enableDrivers() {
  digitalWrite(L_REN, HIGH); digitalWrite(L_LEN, HIGH);
  digitalWrite(R_REN, HIGH); digitalWrite(R_LEN, HIGH);
}

void setLeft(int v) {
  v = constrain(v, -255, 255);
  currentLeftPWM = v;
  if (v > 0)       { analogWrite(L_RPWM, v);   analogWrite(L_LPWM, 0); }
  else if (v < 0)  { analogWrite(L_RPWM, 0);   analogWrite(L_LPWM, -v); }
  else             { analogWrite(L_RPWM, 0);   analogWrite(L_LPWM, 0); }
}

void setRight(int v) {
  v = constrain(v, -255, 255);
  currentRightPWM = v;
  if (v > 0)       { analogWrite(R_RPWM, v);   analogWrite(R_LPWM, 0); }
  else if (v < 0)  { analogWrite(R_RPWM, 0);   analogWrite(R_LPWM, -v); }
  else             { analogWrite(R_RPWM, 0);   analogWrite(R_LPWM, 0); }
}

void stopAll() {
  setLeft(0);
  setRight(0);
}

void resetOdometry() {
  noInterrupts();
  leftCount = rightCount = 0;
  interrupts();
  prevLeftCount = prevRightCount = 0;
}

// ── Encoder ISRs ──────────────────────────────────────────────────────────────
void isrLeftA() {
  bool a = digitalRead(ENC_L_A), b = digitalRead(ENC_L_B);
  if (a == b) leftCount++; else leftCount--;
}
void isrRightA() {
  bool a = digitalRead(ENC_R_A), b = digitalRead(ENC_R_B);
  if (a == b) rightCount++; else rightCount--;
}

// ── Command parser ────────────────────────────────────────────────────────────
static void toUpperInPlace(char *s) { while (*s) { *s = toupper(*s); s++; } }

void handleLine(char *line) {
  while (*line == ' ' || *line == '\t') line++;
  if (*line == '\0') return;

  // Touch watchdog whenever any command is parsed
  lastCmdMs   = millis();
  failsafeActive = false;

  char *cmd = strtok(line, " ");
  if (!cmd) return;
  toUpperInPlace(cmd);

  // ── PING ──────────────────────────────────────────────────────────────────
  if (strcmp(cmd, "PING") == 0) {
    Serial.println(F("PONG ARESPATH"));
    return;
  }

  // ── STOP / S ──────────────────────────────────────────────────────────────
  if (strcmp(cmd, "STOP") == 0 || strcmp(cmd, "S") == 0) {
    stopAll();
    Serial.println(F("ACK STOP"));
    return;
  }

  // ── RESET_ODOM ────────────────────────────────────────────────────────────
  if (strcmp(cmd, "RESET_ODOM") == 0 || strcmp(cmd, "RESET") == 0) {
    resetOdometry();
    Serial.println(F("ACK RESET_ODOM"));
    return;
  }

  // ── SET <l> <r> ───────────────────────────────────────────────────────────
  if (strcmp(cmd, "SET") == 0) {
    char *ls = strtok(NULL, " ");
    char *rs = strtok(NULL, " ");
    if (!ls || !rs) { Serial.println(F("ERR SET missing args")); return; }
    setLeft(constrain(atoi(ls), -255, 255));
    setRight(constrain(atoi(rs), -255, 255));
    Serial.println(F("ACK SET"));
    return;
  }

  // ── Simple direction shortcuts ────────────────────────────────────────────
  if (strcmp(cmd, "F") == 0) {
    setLeft(DEFAULT_SPEED); setRight(DEFAULT_SPEED);
    Serial.println(F("ACK F"));
    return;
  }
  if (strcmp(cmd, "B") == 0) {
    setLeft(-DEFAULT_SPEED); setRight(-DEFAULT_SPEED);
    Serial.println(F("ACK B"));
    return;
  }
  if (strcmp(cmd, "R") == 0) {
    setLeft(-DEFAULT_SPEED); setRight(DEFAULT_SPEED);
    Serial.println(F("ACK R"));
    return;
  }
  if (strcmp(cmd, "L") == 0) {
    setLeft(DEFAULT_SPEED); setRight(-DEFAULT_SPEED);
    Serial.println(F("ACK L"));
    return;
  }

  Serial.print(F("ERR unknown cmd: "));
  Serial.println(cmd);
}

// ── Serial reader (non-blocking) ──────────────────────────────────────────────
void readSerial() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      serialBuf[serialIdx] = '\0';
      handleLine(serialBuf);
      serialIdx = 0;
    } else if (serialIdx < sizeof(serialBuf) - 1) {
      serialBuf[serialIdx++] = ch;
    }
  }
}

// ── Telemetry ─────────────────────────────────────────────────────────────────
void sendTelemetry() {
  unsigned long now = millis();
  if (now - lastTelMs < TELEMETRY_MS) return;

  long lc, rc;
  noInterrupts(); lc = leftCount; rc = rightCount; interrupts();

  float leftRpm = 0, rightRpm = 0;
  if (lastTelMs > 0) {
    float dtMin = (now - lastTelMs) / 60000.0f;
    if (dtMin > 0) {
      leftRpm  = (lc - prevLeftCount)  / (TICKS_PER_REV * dtMin);
      rightRpm = (rc - prevRightCount) / (TICKS_PER_REV * dtMin);
    }
  }
  prevLeftCount  = lc;
  prevRightCount = rc;
  lastTelMs = now;

  // Battery voltage (0-20 V range with 4:1 divider on A2)
  float batt = analogRead(BATT_PIN) * BATT_SCALE;

  Serial.print(F("TEL,"));
  Serial.print(now);         Serial.print(',');
  Serial.print(lc);          Serial.print(',');
  Serial.print(rc);          Serial.print(',');
  Serial.print(leftRpm, 2);  Serial.print(',');
  Serial.print(rightRpm, 2); Serial.print(',');
  Serial.print(currentLeftPWM);  Serial.print(',');
  Serial.print(currentRightPWM); Serial.print(',');
  Serial.println(batt, 2);
}

// ── Watchdog ──────────────────────────────────────────────────────────────────
void checkWatchdog() {
  if (failsafeActive) return;
  if ((currentLeftPWM != 0 || currentRightPWM != 0) &&
      (millis() - lastCmdMs > WATCHDOG_MS)) {
    stopAll();
    failsafeActive = true;
    Serial.println(F("FAILSAFE_STOP"));
  }
}

// ── Setup / Loop ──────────────────────────────────────────────────────────────
void setup() {
  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT);
  pinMode(L_REN,  OUTPUT); pinMode(L_LEN,  OUTPUT);
  pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT);
  pinMode(R_REN,  OUTPUT); pinMode(R_LEN,  OUTPUT);

  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRightA, CHANGE);

  Serial.begin(115200);
  enableDrivers();
  stopAll();
  resetOdometry();
  lastCmdMs = millis();

  Serial.println(F("PONG ARESPATH"));   // auto-announce on reset
}

void loop() {
  readSerial();
  checkWatchdog();
  sendTelemetry();
}
