# 🦾 6-DOF Robotic Arm Master Controller V3

**Real-Time Gesture AI, Gemini Voice Control & Auto-Velocity Kinematic Suite**

[![Python](https://img.shields.io/badge/Python-3.9%2B-blue?logo=python&logoColor=white)](https://python.org)
[![Streamlit](https://img.shields.io/badge/Streamlit-1.28%2B-FF4B4B?logo=streamlit&logoColor=white)](https://streamlit.io)
[![MediaPipe](https://img.shields.io/badge/MediaPipe-Hand%20Tracking-green)](https://mediapipe.dev)
[![OpenCV](https://img.shields.io/badge/OpenCV-Computer%20Vision-5C3EE8)](https://opencv.org)
[![Gemini](https://img.shields.io/badge/Google-Gemini%20Voice-4285F4)](https://ai.google.dev)
[![Status](https://img.shields.io/badge/Status-Complete-brightgreen)]()

---

## 📖 Description

This project is a complete **professional-grade control suite** for a 6-DOF robotic arm. It unifies classical robotics kinematics, real-time computer vision gesture control, natural-language voice commands (Google Gemini), and live hardware serial communication with an ESP32.

**What it does**
- Full forward & inverse kinematics using Denavit–Hartenberg parameters
- Real-time Jacobian, end-effector velocity, manipulability index and singularity detection
- **Gesture Control** – MediaPipe hand tracking maps hand pose → joint angles with low-pass filtering
- **Voice Control** – Speak natural language commands (“Move shoulder up by 25 degrees”, “Close gripper”) processed by Gemini AI
- **Manual Control** – Smooth joint sliders with automatic velocity calculation and trajectory waypoint saving
- Live 3-D kinematic visualization (Plotly) + optional GLB model viewer
- Hardware link to ESP32 via serial (115200 baud) with rate-limiting and dead-band filtering

**Problem it solves**  
Building a modern robotic arm interface usually requires separate tools for kinematics, vision, voice and hardware. This single Streamlit application brings everything together in one dark-cyber themed dashboard, making advanced robotic control accessible for students, makers and researchers.

**Target audience**  
Robotics students, mechatronics engineers, makers working with servo arms / ESP32, and educators teaching kinematics, computer vision and human-robot interaction.

---

## 📑 Table of Contents

- [Description](#-description)
- [Tech Stack](#-tech-stack--badges)
- [Hardware Photos](#-hardware--wiring)
- [Getting Started](#️-getting-started--technical-setup)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation-instructions)
  - [Usage](#usage-examples)
- [Key Features](#-key-features)
- [Kinematics & Control](#-kinematics--control)
- [Architecture](#-software-architecture)
- [Contributing](#-collaboration--support)
- [Project Status](#-project-status--roadmap)
- [Author](#-author--credits)

---

## 🛠️ Tech Stack / Badges

| Category              | Technology                                      |
|-----------------------|-------------------------------------------------|
| Language              | Python 3.9+                                     |
| UI Framework          | Streamlit (wide layout, custom cyber CSS)       |
| Computer Vision       | OpenCV + MediaPipe Hand Landmarker              |
| Voice AI              | Google Gemini + SpeechRecognition + PyAudio     |
| Kinematics            | NumPy + SciPy (Rotation, pinv)                  |
| Visualization         | Plotly 3-D + model-viewer (GLB)                 |
| Hardware              | pyserial → ESP32 (115200 baud)                  |
| Math                  | DH parameters, Jacobian, manipulability         |

**Install command**
```bash
pip install streamlit pyserial opencv-python mediapipe numpy pandas plotly scipy sounddevice google-generativeai SpeechRecognition pyaudio
```

---

## 📸 Hardware & Wiring

### Physical 6-DOF Robotic Arm
<img width="460" height="626" alt="image" src="https://github.com/user-attachments/assets/cedcc40b-f7ae-49ad-8792-1587ae116bb0" />


*6-DOF servo arm with gripper, driven by an ESP32 and PCA9685 servo driver. The white base houses the power distribution and control electronics.*

### Electronics Wiring Diagram
<img width="945" height="518" alt="image" src="https://github.com/user-attachments/assets/16888e53-828f-4b00-a320-d7e4237930f5" />


*Complete schematic: ESP32 DevKit ↔ PCA9685 16-channel PWM driver ↔ 6× SG90/MG996R servos + power supply (Li-ion pack or external 5–6 V).*

---

## 🛠️ Getting Started & Technical Setup

### Prerequisites

- Python 3.9 or higher
- Webcam (for gesture mode)
- Microphone (for voice mode)
- ESP32 + PCA9685 servo driver + 6-DOF arm (optional for hardware)
- Google Gemini API key (optional, only for voice)

### Installation Instructions

1. **Create a new GitHub repository** (recommended settings below)
2. **Clone & enter**
   ```bash
   git clone https://github.com/YOUR_USERNAME/6DOF-Robotic-Arm-Controller.git
   cd 6DOF-Robotic-Arm-Controller
   ```

3. **Virtual environment**
   ```bash
   python -m venv venv
   # Windows
   venv\Scripts\activate
   # macOS / Linux
   source venv/bin/activate
   ```

4. **Install dependencies**
   ```bash
   pip install streamlit pyserial opencv-python mediapipe numpy pandas plotly scipy sounddevice google-generativeai SpeechRecognition pyaudio
   ```

5. **(Optional) Set Gemini API key**
   ```powershell
   # Windows PowerShell
   $env:GEMINI_API_KEY="your-api-key-here"
   ```

6. **Run the application**
   ```bash
   streamlit run newrr.py
   ```
   The dashboard opens at `http://localhost:8501`.

### Usage Examples

#### Manual Control
- Use the six joint sliders (Waist → Gripper)
- Enable “Auto-send commands to hardware”
- Save poses and replay sequences

#### Gesture Control
1. Select **Gesture Control** mode
2. Click **▶ START CAMERA TRACKING**
3. Move your hand in front of the webcam:
   - Left/Right → Waist
   - Up/Down → Shoulder
   - Reach → Elbow
   - Tilt/Roll → Wrist
   - Pinch → Gripper

#### Voice Control
1. Select **Voice Control** mode
2. Click **🎤 LISTEN & EXECUTE COMMAND**
3. Speak naturally, e.g.:
   - “Move shoulder up by 25 degrees”
   - “Rotate waist left”
   - “Close the gripper”
   - “Reset robot position”

#### Inverse Kinematics
- Enter a target (X, Y, Z) in metres
- Click **⚡ Solve & Move to Pose**
- The arm will move to the computed joint configuration if reachable

---

## ✨ Key Features

- **Full 6-DOF DH Kinematics** with live forward & inverse solutions
- **Real-time Jacobian** + end-effector linear/angular velocity
- **Manipulability index** and singularity warning
- **MediaPipe Gesture AI** with EMA smoothing and scale-invariant mapping
- **Gemini Natural Language** voice commands with command history
- **Auto-velocity engine** (EMA-filtered joint velocities + clamp)
- **Serial rate limiting + dead-band** to protect the ESP32
- **Trajectory waypoint system** (save / run sequences)
- **3-D Plotly visualization** + optional GLB model viewer
- **Dark cyber UI** with bento cards, neon accents and live metric badges

---

## 📐 Kinematics & Control

### Denavit–Hartenberg Parameters
```
Joint  θ_offset   d (m)    a (m)   α (rad)
0      0         0.15     0       -π/2
1      0         0        0.30     0
2      0         0        0.25     0
3      0         0        0       -π/2
4      0         0.10     0        π/2
5      0         0.05     0        0
```

### Joint Limits (degrees)
| Joint          | Min | Max |
|----------------|-----|-----|
| Waist          | 0   | 180 |
| Shoulder       | 0   | 180 |
| Elbow          | 0   | 180 |
| Wrist Pitch    | 0   | 180 |
| Wrist Roll     | 0   | 180 |
| Gripper        | 0   | 90  |

Servo-to-kinematic offset: 90° (joints 0-4), 45° (gripper).

### Control Modes
1. **Manual** – Direct joint angle sliders + auto velocity
2. **Gesture** – MediaPipe hand landmarks → joint mapping
3. **Voice** – Gemini LLM parses natural language → joint deltas

---

## 🏗️ Software Architecture

```
newrr.py
├── RobotKinematics          # DH, FK, IK, Jacobian, manipulability
├── RobotController          # Serial ESP32 link (rate-limited)
├── GestureController        # MediaPipe Tasks / legacy + HUD
├── GeminiVoiceController    # Speech → Gemini → joint commands
├── create_3d_arm_visualization
├── create_workspace_visualization
└── main()                   # Streamlit UI + session state
```

All mutable state lives in `st.session_state` for deterministic Streamlit behaviour.

---

## 🤝 Collaboration & Support

### Contributing Guidelines

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/ImprovedGesture`)
3. Commit with clear messages
4. Open a Pull Request

Please keep the cyber dark theme and document any new kinematic or control features.

### Project Status / Roadmap

- [x] Full DH forward & inverse kinematics
- [x] Real-time Jacobian, velocity & manipulability
- [x] MediaPipe gesture control with smoothing
- [x] Gemini voice natural-language control
- [x] ESP32 serial hardware link
- [x] Trajectory waypoint system
- [x] 3-D Plotly + GLB visualization
- [x] Dark cyber UI
- [ ] Collision detection / self-collision avoidance
- [ ] Path planning (RRT / Cartesian trajectory)
- [ ] Multi-arm support
- [ ] ROS 2 bridge (optional)

**Current Status**: Feature-complete V3 release ready for educational and maker use.

---

## 👤 Author & Credits

**Student / Developer**: Udit Kumar Panigrahi 
**Institute**: Lovely Professional University (LPU)  
**Roll No.**: 12508609  
 

**Programme context**:  Advanced Robotics projects (2026)

---

## 📄 License

Released for educational and non-commercial use.  
Attribution appreciated.

---

*Built to make advanced robotic control — kinematics, vision and voice — accessible in a single elegant dashboard.*
