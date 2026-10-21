/**
 * 5-DOF Robotic Arm — Arduino UNO Firmware
 * ==========================================
 * Command Format: W090S090E090P090R090G050\n
 *   W = Waist   (0-180°)
 *   S = Shoulder (0-180°)
 *   E = Elbow    (0-180°)
 *   P = Wrist Pitch (0-180°)
 *   R = Wrist Roll  (0-180°)
 *   G = Gripper  (0-100 → mapped 0-90°)
 *
 * Servo Pin Mapping:
 *   Waist      → Pin 3
 *   Shoulder   → Pin 5
 *   Elbow      → Pin 6
 *   Wrist Pitch→ Pin 9
 *   Wrist Roll → Pin 10
 *   Gripper    → Pin 11
 *
 * Upload via: Arduino IDE → Board: Arduino UNO → Port: COMX
 */

#include <Servo.h>

Servo waistServo;
Servo shoulderServo;
Servo elbowServo;
Servo wristPitchServo;
Servo wristRollServo;
Servo gripperServo;

// Current positions for smooth interpolation
int curWaist = 90, curShoulder = 90, curElbow = 90;
int curWristP = 90, curWristR = 90, curGripper = 0;

// Target positions
int tgtWaist = 90, tgtShoulder = 90, tgtElbow = 90;
int tgtWristP = 90, tgtWristR = 90, tgtGripper = 0;

#define STEP_DELAY 10  // ms between interpolation steps
#define MAX_STEP   3   // max degrees per step for smooth motion

void setup() {
  Serial.begin(9600);

  waistServo.attach(3);
  shoulderServo.attach(5);
  elbowServo.attach(6);
  wristPitchServo.attach(9);
  wristRollServo.attach(10);
  gripperServo.attach(11);

  // Home position
  setAllServos(90, 90, 90, 90, 90, 0);
  Serial.println("READY:RoboArm_v1.0");
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 10) {
      parseCommand(cmd);
    }
  }
  smoothMove();
  delay(STEP_DELAY);
}

void parseCommand(String cmd) {
  // Expected: W090S090E090P090R090G050
  int wIdx = cmd.indexOf('W');
  int sIdx = cmd.indexOf('S');
  int eIdx = cmd.indexOf('E');
  int pIdx = cmd.indexOf('P');
  int rIdx = cmd.indexOf('R');
  int gIdx = cmd.indexOf('G');

  if (wIdx < 0 || sIdx < 0 || eIdx < 0) {
    Serial.println("ERR:INVALID_CMD");
    return;
  }

  tgtWaist    = constrain(cmd.substring(wIdx+1, sIdx).toInt(), 0, 180);
  tgtShoulder = constrain(cmd.substring(sIdx+1, eIdx).toInt(), 0, 180);
  tgtElbow    = constrain(cmd.substring(eIdx+1, pIdx > 0 ? pIdx : cmd.length()).toInt(), 0, 180);
  if (pIdx >= 0) tgtWristP = constrain(cmd.substring(pIdx+1, rIdx > 0 ? rIdx : cmd.length()).toInt(), 0, 180);
  if (rIdx >= 0) tgtWristR = constrain(cmd.substring(rIdx+1, gIdx > 0 ? gIdx : cmd.length()).toInt(), 0, 180);
  if (gIdx >= 0) tgtGripper = constrain(cmd.substring(gIdx+1).toInt(), 0, 100);

  Serial.print("OK:");
  Serial.println(cmd);
}

void smoothMove() {
  bool moved = false;
  curWaist    = stepToward(curWaist,    tgtWaist,    waistServo,      moved);
  curShoulder = stepToward(curShoulder, tgtShoulder, shoulderServo,   moved);
  curElbow    = stepToward(curElbow,    tgtElbow,    elbowServo,      moved);
  curWristP   = stepToward(curWristP,   tgtWristP,   wristPitchServo, moved);
  curWristR   = stepToward(curWristR,   tgtWristR,   wristRollServo,  moved);

  // Gripper: map 0-100 → 0-90 degrees
  int gripTarget = map(tgtGripper, 0, 100, 0, 90);
  int gripCurrent = map(curGripper, 0, 100, 0, 90);
  int newGripAngle = stepTowardVal(gripCurrent, gripTarget, moved);
  curGripper = map(newGripAngle, 0, 90, 0, 100);
  gripperServo.write(newGripAngle);
}

int stepToward(int cur, int tgt, Servo &servo, bool &moved) {
  if (cur == tgt) return cur;
  int step = (tgt > cur) ? min(MAX_STEP, tgt - cur) : -min(MAX_STEP, cur - tgt);
  cur += step;
  servo.write(cur);
  moved = true;
  return cur;
}

int stepTowardVal(int cur, int tgt, bool &moved) {
  if (cur == tgt) return cur;
  int step = (tgt > cur) ? min(MAX_STEP, tgt - cur) : -min(MAX_STEP, cur - tgt);
  moved = true;
  return cur + step;
}

void setAllServos(int w, int s, int e, int p, int r, int g) {
  waistServo.write(w);
  shoulderServo.write(s);
  elbowServo.write(e);
  wristPitchServo.write(p);
  wristRollServo.write(r);
  gripperServo.write(map(g, 0, 100, 0, 90));
  curWaist = w; curShoulder = s; curElbow = e;
  curWristP = p; curWristR = r; curGripper = g;
  tgtWaist = w; tgtShoulder = s; tgtElbow = e;
  tgtWristP = p; tgtWristR = r; tgtGripper = g;
}
