# gesture_server.py — Standalone Gesture Recognition Server
# ===========================================================
# Run alongside Streamlit: python gesture_server.py
#
# pip install opencv-python mediapipe pyserial
#
# Usage:
#   python gesture_server.py                    # USB webcam, no serial
#   python gesture_server.py COM3               # USB webcam + Arduino on COM3
#   python gesture_server.py COM3 1             # Webcam index 1
#   python gesture_server.py COM3 http://IP:81/stream   # ESP32-CAM stream

import sys
import time
import cv2
import mediapipe as mp
import serial

# ── Config ─────────────────────────────────────────────────────────────────────
SERIAL_PORT  = sys.argv[1] if len(sys.argv) > 1 else None
CAM_SOURCE   = sys.argv[2] if len(sys.argv) > 2 else 0
BAUD_RATE    = 9600
SEND_DELAY   = 0.5   # seconds between commands

# ── MediaPipe ──────────────────────────────────────────────────────────────────
mp_hands = mp.solutions.hands
mp_draw  = mp.solutions.drawing_utils

# ── Joints state ───────────────────────────────────────────────────────────────
joints = {
    "waist": 90, "shoulder": 90, "elbow": 90,
    "wrist_pitch": 90, "wrist_roll": 90, "gripper": 0
}

# ── Gesture → command map ──────────────────────────────────────────────────────
GESTURE_CMDS = {
    "FIST":       ("G100",   "Close gripper"),
    "OPEN":       ("G000",   "Open gripper"),
    "INDEX_UP":   ("S+05",   "Shoulder up"),
    "INDEX_DOWN": ("S-05",   "Shoulder down"),
    "PEACE":      ("E+10",   "Elbow extend"),
    "THREE":      ("E-10",   "Elbow retract"),
    "HANG_LOOSE": ("R+10",   "Wrist roll right"),
    "CALL_ME":    ("R-10",   "Wrist roll left"),
    "THUMBS_UP":  ("HOME",   "Home position"),
    "POINT_RIGHT":("W+05",   "Waist right"),
    "POINT_LEFT": ("W-05",   "Waist left"),
    "FIST_UP":    ("STOP",   "Emergency stop"),
}


def count_fingers(lm):
    """Return [thumb, index, middle, ring, pinky] — 1=up, 0=down"""
    tips = [8, 12, 16, 20]
    fingers = []
    # Thumb: compare x (for right hand)
    fingers.append(1 if lm.landmark[4].x < lm.landmark[3].x else 0)
    # Other fingers: tip y < pip y
    for tip in tips:
        fingers.append(1 if lm.landmark[tip].y < lm.landmark[tip - 2].y else 0)
    return fingers


def classify_gesture(fingers, lm):
    total = sum(fingers)
    th, ix, mi, ri, pi = fingers

    if total == 0:   return "FIST"
    if total == 5:   return "OPEN"
    if fingers == [0, 1, 0, 0, 0]: return "INDEX_UP"
    if fingers == [0, 1, 1, 0, 0]: return "PEACE"
    if fingers == [0, 1, 1, 1, 0]: return "THREE"
    if fingers == [1, 0, 0, 0, 1]: return "HANG_LOOSE"
    if fingers == [1, 0, 0, 0, 0]: return "THUMBS_UP"
    if fingers == [0, 0, 0, 0, 1]: return "CALL_ME"
    # Point direction by index tip x vs wrist x
    if ix == 1 and total == 1:
        if lm.landmark[8].x > lm.landmark[0].x + 0.1:
            return "POINT_RIGHT"
        elif lm.landmark[8].x < lm.landmark[0].x - 0.1:
            return "POINT_LEFT"
        return "INDEX_UP"
    return f"GESTURE_{total}"


def apply_cmd(cmd):
    global joints
    if cmd == "HOME":
        joints = {k: 90 for k in joints}
        joints["gripper"] = 0
    elif cmd == "STOP":
        joints = {k: 90 for k in joints}
        joints["gripper"] = 0
    elif cmd.startswith("G"):
        joints["gripper"] = min(100, max(0, int(cmd[1:])))
    elif cmd.startswith("S+"):
        joints["shoulder"] = min(180, joints["shoulder"] + int(cmd[2:]))
    elif cmd.startswith("S-"):
        joints["shoulder"] = max(0,   joints["shoulder"] - int(cmd[2:]))
    elif cmd.startswith("E+"):
        joints["elbow"]    = min(180, joints["elbow"]    + int(cmd[2:]))
    elif cmd.startswith("E-"):
        joints["elbow"]    = max(0,   joints["elbow"]    - int(cmd[2:]))
    elif cmd.startswith("W+"):
        joints["waist"]    = min(180, joints["waist"]    + int(cmd[2:]))
    elif cmd.startswith("W-"):
        joints["waist"]    = max(0,   joints["waist"]    - int(cmd[2:]))
    elif cmd.startswith("R+"):
        joints["wrist_roll"] = min(180, joints["wrist_roll"] + int(cmd[2:]))
    elif cmd.startswith("R-"):
        joints["wrist_roll"] = max(0,   joints["wrist_roll"] - int(cmd[2:]))


def build_serial_cmd():
    j = joints
    return (f"W{j['waist']:03d}S{j['shoulder']:03d}E{j['elbow']:03d}"
            f"P{j['wrist_pitch']:03d}R{j['wrist_roll']:03d}G{j['gripper']:03d}\n")


# ── Serial connection ──────────────────────────────────────────────────────────
ser = None
if SERIAL_PORT:
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"✓ Connected to Arduino on {SERIAL_PORT}")
    except Exception as e:
        print(f"⚠ Serial not available: {e} — simulation mode")

# ── Camera ────────────────────────────────────────────────────────────────────
try:
    cam_source = int(CAM_SOURCE)
except ValueError:
    cam_source = CAM_SOURCE  # URL string

cap = cv2.VideoCapture(cam_source)
if not cap.isOpened():
    print(f"❌ Cannot open camera source: {cam_source}")
    sys.exit(1)

print(f"✓ Camera opened: {cam_source}")
print("Press Q to quit")

# ── Main loop ─────────────────────────────────────────────────────────────────
with mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7,
                    min_tracking_confidence=0.6) as hands:
    last_gesture = None
    last_sent    = 0

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result    = hands.process(frame_rgb)

        gesture = "None"
        if result.multi_hand_landmarks:
            for lm in result.multi_hand_landmarks:
                mp_draw.draw_landmarks(frame, lm, mp_hands.HAND_CONNECTIONS)
                fingers = count_fingers(lm)
                gesture = classify_gesture(fingers, lm)

        now = time.time()
        if (gesture != last_gesture and gesture in GESTURE_CMDS
                and (now - last_sent) > SEND_DELAY):
            cmd, desc = GESTURE_CMDS[gesture]
            apply_cmd(cmd)
            serial_cmd = build_serial_cmd()
            if ser:
                ser.write(serial_cmd.encode())
            print(f"[{time.strftime('%H:%M:%S')}] {gesture:15s} → {cmd:6s}  {desc}")
            print(f"  Serial: {serial_cmd.strip()}")
            last_gesture = gesture
            last_sent    = now

        # Overlay
        h, w = frame.shape[:2]
        cv2.rectangle(frame, (0, 0), (w, 50), (10, 20, 40), -1)
        cv2.putText(frame, f"Gesture: {gesture}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 212, 255), 2)
        cv2.putText(frame, f"W:{joints['waist']} S:{joints['shoulder']} E:{joints['elbow']}",
                    (10, h - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.putText(frame, f"WP:{joints['wrist_pitch']} WR:{joints['wrist_roll']} G:{joints['gripper']}%",
                    (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        cv2.imshow("Gesture Control — RoboArm (Press Q to quit)", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
if ser:
    ser.close()
print("Gesture server stopped.")
