// Scavenge Bot firmware v0.6 — calibration in software, so the horns never need re-seating.
// Serial 115200, one command per line (Newline line ending).
//
// Everyday commands (use the calibration):
//   home                 every joint to its reference pose (base forward, shoulder upright, elbow 90, gripper closed)
//   park                 arm straight up (needs each joint's direction calibrated, otherwise same as home)
//   move B S E G         raw servo degrees, clamped to each joint's calibrated limits
//   base|shoulder|elbow|gripper N  (or b|s|e|g N)   one servo, raw degrees, clamped
//   open  close  relax  speed N  status  limits  help
//
// Calibration (do once; saved in the Uno's EEPROM, survives power-off):
//   cal start            calibration mode: limits ignored (only 0-180), so you can explore
//   nudge J N            move joint J by N degrees (e.g. nudge shoulder -5), wakes it if asleep
//   cal J min / cal J max   this position is a safe end of travel. Find it by WATCHING the joint in small steps;
//                        stop before anything touches. Never drive into a stop (that broke the shoulder servo).
//   cal J ref            optional: this position is the joint's reference pose. Without it, home = middle of the range
//   cal J dir 1|-1       1 if increasing the servo value moves the joint "forward" (base left, shoulder out,
//                        elbow bends forward/down, gripper opens), otherwise -1
//   cal show   cal save   cal reset   cal end
//
// Supervised wake (for a joint whose real position is unknown; watch it on camera):
//   creep J T [N]        joint asleep: send N pulses (default 2, max 10) toward raw value T, then let go. The joint twitches
//                        a few degrees toward T and stops. The direction of the twitch tells which side of T it really is.
//   hold J T             joint asleep: soft-wake it AT raw value T and hold. Use once creep shows T is close to where it is.
//   relax J              let one joint go (relax alone = all joints)
//
// Safety: boots relaxed. Each servo wakes only when first commanded, with a SOFT WAKE: pulses start at ~5 per second
// and ramp to the normal 50 over ~1.5 s, so an analog servo creeps to its target instead of lunging (a full-speed
// wake jump broke the shoulder servo's end stop in v0.4). `relax` and `park` store each joint's last position in
// EEPROM, and the next wake starts there. After waking, every move ramps at `speed` deg/s. Wiring: base D3, shoulder D5,
// elbow D6, gripper D9; servos on 4xAA; grounds shared; nothing on the Uno's 5V pin.
#include <Servo.h>
#include <EEPROM.h>

const uint8_t N = 4;
const char* NAMES[N] = {"base", "shoulder", "elbow", "gripper"};
const uint8_t PINS[N] = {3, 5, 6, 9};
const int US_MIN = 544, US_MAX = 2400;      // Servo library defaults

struct Cal {                                  // stored in EEPROM
  uint16_t magic;
  int16_t ref[N], lo[N], hi[N];
  int8_t dir[N];                              // 0 = not calibrated yet
  uint8_t refSet[N];                          // 1 once `cal J ref` was used
  int16_t last[N];                            // last commanded position, saved on relax/park
};
const uint16_t MAGIC = 0x5B05;
Cal cal;

Servo servo[N];
float pos[N], target[N];
bool live[N] = {false, false, false, false};
bool calMode = false;
float speedDps = 60;
bool moving = false;
bool saveWhenDone = false;                    // set by park: store positions once it arrives
unsigned long moveStart = 0, lastTick = 0;
char line[64]; uint8_t len = 0;

void defaults() {
  cal.magic = MAGIC;
  for (uint8_t k = 0; k < N; k++) { cal.ref[k] = 90; cal.lo[k] = 0; cal.hi[k] = 180; cal.dir[k] = 0; cal.refSet[k] = 0; cal.last[k] = 90; }
}
bool calibrated(uint8_t k) { return !(cal.lo[k] == 0 && cal.hi[k] == 180); }

void writeServo(uint8_t k) { servo[k].writeMicroseconds(US_MIN + (int)(pos[k] * (US_MAX - US_MIN) / 180.0 + 0.5)); }
bool anyLive() { for (uint8_t k = 0; k < N; k++) if (live[k]) return true; return false; }

void wake(uint8_t k) {
  if (live[k]) return;
  // soft wake at the last known position: few pulses = little drive, so the servo creeps there
  float start = cal.last[k];
  int us = US_MIN + (int)(start * (US_MAX - US_MIN) / 180.0 + 0.5);
  pinMode(PINS[k], OUTPUT);
  for (uint16_t gap = 200; gap > 20; gap -= 6) {            // ~30 pulses, 5 Hz rising to ~50 Hz, ~1.6 s
    digitalWrite(PINS[k], HIGH); delayMicroseconds(us); digitalWrite(PINS[k], LOW);
    delay(gap);
  }
  pos[k] = start; writeServo(k);
  servo[k].attach(PINS[k], US_MIN, US_MAX); live[k] = true; // now ramps from start to target at `speed`
}
void creep(uint8_t k, int deg, uint8_t n) {                // a few sparse pulses, then silence: twitch and let go
  int us = US_MIN + (int)((long)deg * (US_MAX - US_MIN) / 180);
  pinMode(PINS[k], OUTPUT);
  for (uint8_t i = 0; i < n; i++) { digitalWrite(PINS[k], HIGH); delayMicroseconds(us); digitalWrite(PINS[k], LOW); delay(60); }
}
void saveLast() {
  for (uint8_t k = 0; k < N; k++) if (live[k]) cal.last[k] = (int)(pos[k] + 0.5);
  EEPROM.put(0, cal);
}
void relaxAll() { saveLast(); for (uint8_t k = 0; k < N; k++) { servo[k].detach(); live[k] = false; } moving = false; }

void setTarget(uint8_t k, long v) {
  long lo = calMode ? 0 : cal.lo[k], hi = calMode ? 180 : cal.hi[k];
  long c = constrain(v, lo, hi);
  target[k] = c;
  if (c != v) {
    Serial.print(F("warn: ")); Serial.print(NAMES[k]); Serial.print(F(" clamped to ")); Serial.print(c);
    Serial.print(F(" (limit ")); Serial.print(lo); Serial.print('-'); Serial.print(hi); Serial.println(')');
  }
}
void startMove(int only = -1) {
  for (uint8_t k = 0; k < N; k++) if (only < 0 || only == k) wake(k);
  moving = true; moveStart = millis(); Serial.println(F("ok"));
}

int keyOf(const char* w) {
  if (!w) return -1;
  if (!strcmp(w, "base") || !strcmp(w, "b")) return 0;
  if (!strcmp(w, "shoulder") || !strcmp(w, "s")) return 1;
  if (!strcmp(w, "elbow") || !strcmp(w, "e")) return 2;
  if (!strcmp(w, "gripper") || !strcmp(w, "g")) return 3;
  return -1;
}
bool parseNum(const char* t, long& out) {
  if (!t || !*t) return false;
  char* end; out = strtol(t, &end, 10); return *end == '\0';
}

void printJoint(uint8_t k) {
  Serial.print(NAMES[k]); Serial.print(F(": ref ")); Serial.print(cal.ref[k]);
  Serial.print(F(", limits ")); Serial.print(cal.lo[k]); Serial.print('-'); Serial.print(cal.hi[k]);
  Serial.print(cal.refSet[k] ? F(" (set)") : F(" (middle)"));
  Serial.print(F(", dir ")); Serial.print(cal.dir[k] ? (cal.dir[k] > 0 ? "+1" : "-1") : "?");
  Serial.println(calibrated(k) ? F("") : F("  (limits not set)"));
}
void printStatus() {
  Serial.print(F("pos"));
  for (uint8_t k = 0; k < N; k++) { Serial.print(' '); Serial.print((int)(pos[k] + 0.5)); }
  Serial.print(F("  target"));
  for (uint8_t k = 0; k < N; k++) { Serial.print(' '); Serial.print((int)target[k]); }
  Serial.print(F("  speed ")); Serial.print((int)speedDps);
  Serial.print(calMode ? F("  CAL MODE") : F(""));
  Serial.print(F("  holding:")); bool none = true;
  for (uint8_t k = 0; k < N; k++) if (live[k]) { Serial.print(' '); Serial.print(NAMES[k]); none = false; }
  Serial.println(none ? F(" none") : F(""));
}

// joint-angle pose relative to the reference pose (degrees of joint motion); needs dir
bool posePark(int out[N]) {
  const int delta[N] = {0, 0, -80, 0};       // straight up: elbow unbends 80 deg from its right-angle reference
  for (uint8_t k = 0; k < N; k++) {
    if (delta[k] && !cal.dir[k]) return false;
    out[k] = cal.ref[k] + cal.dir[k] * delta[k];
  }
  return true;
}

void handleCal(char** a, uint8_t na) {
  if (na == 0 || !strcmp(a[0], "show")) { for (uint8_t k = 0; k < N; k++) printJoint(k); return; }
  if (!strcmp(a[0], "start")) { calMode = true; Serial.println(F("ok calibration mode: limits ignored. Nudge each joint, then cal J ref / min / max / dir.")); return; }
  if (!strcmp(a[0], "end")) { calMode = false; Serial.println(F("ok calibration mode off (unsaved changes stay until power-off; cal save keeps them)")); return; }
  if (!strcmp(a[0], "save")) {
    for (uint8_t k = 0; k < N; k++) if (cal.lo[k] > cal.hi[k]) { int t = cal.lo[k]; cal.lo[k] = cal.hi[k]; cal.hi[k] = t; }
    EEPROM.put(0, cal); Serial.println(F("ok saved")); return;
  }
  if (!strcmp(a[0], "reset")) { defaults(); Serial.println(F("ok calibration reset to defaults (not saved)")); return; }
  int k = keyOf(a[0]);
  if (k < 0 || na < 2) { Serial.println(F("error: try cal show | cal start | cal J ref|min|max | cal J dir 1|-1 | cal save | cal end")); return; }
  if (!live[k]) { Serial.print(F("error: ")); Serial.print(NAMES[k]); Serial.println(F(" is asleep; nudge or move it first so its position is known")); return; }
  int p = (int)(pos[k] + 0.5);
  bool lim = false;
  if (!strcmp(a[1], "ref")) { cal.ref[k] = p; cal.refSet[k] = 1; }
  else if (!strcmp(a[1], "min")) { cal.lo[k] = p; lim = true; }
  else if (!strcmp(a[1], "max")) { cal.hi[k] = p; lim = true; }
  else if (!strcmp(a[1], "dir")) {
    long d; if (na < 3 || !parseNum(a[2], d) || (d != 1 && d != -1)) { Serial.println(F("error: cal J dir 1 or -1")); return; }
    cal.dir[k] = d;
  } else { Serial.println(F("error: cal J ref|min|max|dir")); return; }
  if (lim && !cal.refSet[k]) cal.ref[k] = (cal.lo[k] + cal.hi[k]) / 2;  // home = middle until a ref is set
  Serial.print(F("ok ")); printJoint(k);
}

void handle(char* s) {
  for (char* p = s; *p; p++) *p = tolower(*p);
  char* cmd = strtok(s, " \t"); if (!cmd) return;
  char* a[5]; uint8_t na = 0;
  while (na < 5 && (a[na] = strtok(NULL, " \t"))) na++;
  long v[4];

  if (!strcmp(cmd, "help")) {
    Serial.println(F("home | park | move B S E G | base|shoulder|elbow|gripper N | open | close | relax | speed N | status | limits"));
    Serial.println(F("calibration: cal start | nudge J N | cal J ref|min|max | cal J dir 1|-1 | cal show | cal save | cal end"));
    Serial.println(F("supervised wake: creep J T [N] | hold J T | relax J"));
  } else if (!strcmp(cmd, "cal")) handleCal(a, na);
  else if (!strcmp(cmd, "nudge")) {
    int k = keyOf(na > 0 ? a[0] : NULL);
    if (k < 0 || na != 2 || !parseNum(a[1], v[0])) { Serial.println(F("error: nudge J N, e.g. nudge shoulder -5")); return; }
    setTarget(k, (long)(live[k] ? pos[k] + 0.5 : cal.last[k]) + v[0]);   // asleep: relative to last known position
    startMove(k);
  } else if (!strcmp(cmd, "move")) {
    if (na != 4 || !parseNum(a[0], v[0]) || !parseNum(a[1], v[1]) || !parseNum(a[2], v[2]) || !parseNum(a[3], v[3])) {
      Serial.println(F("error: move needs 4 numbers, e.g. move 90 120 100 70")); return;
    }
    for (uint8_t k = 0; k < N; k++) setTarget(k, v[k]);
    startMove();
  } else if (!strcmp(cmd, "home")) {
    for (uint8_t k = 0; k < N; k++) setTarget(k, cal.ref[k]);
    startMove();
  } else if (!strcmp(cmd, "park")) {
    int p[N];
    if (!posePark(p)) { Serial.println(F("warn: elbow direction not calibrated, parking at home instead")); for (uint8_t k = 0; k < N; k++) p[k] = cal.ref[k]; }
    for (uint8_t k = 0; k < N; k++) setTarget(k, p[k]);
    startMove(); saveWhenDone = true;
  } else if (!strcmp(cmd, "open") || !strcmp(cmd, "close")) {
    bool open = !strcmp(cmd, "open");
    if (!cal.dir[3]) { Serial.println(F("error: calibrate the gripper first (cal gripper ref while closed, cal gripper dir)")); return; }
    int closedV = cal.ref[3], openV = cal.dir[3] > 0 ? cal.hi[3] : cal.lo[3];
    setTarget(3, open ? openV : closedV); startMove(3);
  } else if (!strcmp(cmd, "relax")) {
    if (na == 0) { relaxAll(); Serial.println(F("ok relaxed")); return; }
    int k = keyOf(a[0]);
    if (k < 0) { Serial.println(F("error: relax or relax J")); return; }
    if (live[k]) { cal.last[k] = (int)(pos[k] + 0.5); target[k] = pos[k]; EEPROM.put(0, cal); servo[k].detach(); live[k] = false; }
    Serial.print(F("ok relaxed ")); Serial.println(NAMES[k]);
  } else if (!strcmp(cmd, "creep") || !strcmp(cmd, "hold")) {
    bool hold = cmd[0] == 'h';
    int k = keyOf(na > 0 ? a[0] : NULL);
    v[1] = 2;
    if (k < 0 || na < 2 || !parseNum(a[1], v[0]) || v[0] < 0 || v[0] > 180 || (na > 2 && (hold || !parseNum(a[2], v[1]))) || v[1] < 1 || v[1] > 10) {
      Serial.println(F("error: creep J T [N]  or  hold J T   (T 0-180, N 1-10)")); return;
    }
    if (live[k]) { Serial.print(F("error: ")); Serial.print(NAMES[k]); Serial.println(F(" is awake; use nudge, or relax it first")); return; }
    if (hold) { cal.last[k] = v[0]; target[k] = v[0]; wake(k); Serial.print(F("ok holding ")); Serial.print(NAMES[k]); Serial.print(F(" at ")); Serial.println(v[0]); }
    else { creep(k, v[0], v[1]); Serial.print(F("ok crept ")); Serial.print(NAMES[k]); Serial.print(F(" toward ")); Serial.println(v[0]); }
  }
  else if (!strcmp(cmd, "speed")) {
    if (na != 1 || !parseNum(a[0], v[0]) || v[0] <= 0) { Serial.println(F("error: speed needs a positive number")); return; }
    speedDps = constrain(v[0], 5, 400); Serial.print(F("ok speed ")); Serial.println((int)speedDps);
  } else if (!strcmp(cmd, "status")) printStatus();
  else if (!strcmp(cmd, "limits")) { for (uint8_t k = 0; k < N; k++) printJoint(k); }
  else {
    int k = keyOf(cmd);
    if (k >= 0 && na == 1 && parseNum(a[0], v[0])) { setTarget(k, v[0]); startMove(k); }
    else { Serial.print(F("error: unknown command \"")); Serial.print(cmd); Serial.println(F("\" (try help)")); }
  }
}

void setup() {
  Serial.begin(115200);
  EEPROM.get(0, cal);
  if (cal.magic == 0x5B04) {                  // v0.4 data: same layout minus last[]; keep the calibration
    for (uint8_t k = 0; k < N; k++) cal.last[k] = cal.ref[k];
    cal.magic = MAGIC; EEPROM.put(0, cal);
  }
  bool fresh = cal.magic != MAGIC;
  if (fresh) defaults();
  for (uint8_t k = 0; k < N; k++) pos[k] = target[k] = cal.last[k];
  Serial.println(F("Scavenge Bot firmware v0.6 ready (relaxed). Type help. Joints soft-wake from their last saved position."));
  Serial.println(fresh ? F("Not calibrated: cal start, then per joint nudge in small steps while watching, cal J min / cal J max. cal save.") : F("Calibration loaded."));
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (len) { line[len] = 0; handle(line); len = 0; } }
    else if (len < sizeof(line) - 1) line[len++] = c;
  }
  unsigned long now = millis();
  if (now - lastTick >= 20) {
    float dt = (now - lastTick) / 1000.0; lastTick = now; if (dt > 0.1) dt = 0.1;
    if (anyLive() && moving) {
      bool any = false;
      for (uint8_t k = 0; k < N; k++) {
        if (!live[k]) continue;
        float d = target[k] - pos[k], step = speedDps * dt;
        if (fabs(d) > 0.01) { pos[k] = fabs(d) <= step ? target[k] : pos[k] + (d > 0 ? step : -step); writeServo(k); any = true; }
      }
      if (!any) {
        moving = false; Serial.print(F("done in ")); Serial.print((now - moveStart) / 1000.0, 1); Serial.println(F(" s"));
        if (saveWhenDone) { saveLast(); saveWhenDone = false; Serial.println(F("parked: positions saved, safe to power off")); }
      }
    }
  }
}
