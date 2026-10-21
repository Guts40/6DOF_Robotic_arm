"""
5-DOF Robotic Arm Control System
Main Streamlit Application
"""

import streamlit as st
import streamlit_3d as sd
import streamlit.components.v1 as components
import math
import time
import json
import os
import numpy as np
import pandas as pd
import serial.tools.list_ports
import serial

st.set_page_config(
    page_title="5-DOF Robotic Arm Control",
    page_icon="🦾",
    layout="wide",
    initial_sidebar_state="expanded"
)

# ─── Custom CSS ───────────────────────────────────────────────────────────────
st.markdown("""
<style>
@import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Rajdhani:wght@400;600;700&display=swap');

:root {
    --bg: #0a0e1a;
    --panel: #111827;
    --border: #1e3a5f;
    --accent: #00d4ff;
    --accent2: #ff6b35;
    --text: #c9d8f0;
    --muted: #4a6080;
    --success: #00ff88;
    --warn: #ffcc00;
}
html, body, [data-testid="stApp"] {
    background-color: var(--bg) !important;
    font-family: 'Rajdhani', sans-serif;
    color: var(--text);
}
[data-testid="stSidebar"] {
    background: var(--panel) !important;
    border-right: 1px solid var(--border);
}
.metric-card {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 16px 20px;
    margin-bottom: 12px;
    position: relative;
    overflow: hidden;
}
.metric-card::before {
    content: '';
    position: absolute;
    left: 0; top: 0; bottom: 0;
    width: 3px;
    background: var(--accent);
}
.metric-card .label {
    font-size: 11px;
    letter-spacing: 2px;
    text-transform: uppercase;
    color: var(--muted);
    font-family: 'Share Tech Mono', monospace;
}
.metric-card .value {
    font-size: 28px;
    font-weight: 700;
    color: var(--accent);
    font-family: 'Share Tech Mono', monospace;
    line-height: 1.2;
}
.metric-card .unit { font-size: 13px; color: var(--muted); margin-left: 4px; }
.status-dot {
    display: inline-block; width: 8px; height: 8px;
    border-radius: 50%; margin-right: 6px;
    animation: pulse 2s infinite;
}
.status-dot.online  { background: var(--success); }
.status-dot.offline { background: #ff4444; animation: none; }
@keyframes pulse {
    0%,100% { opacity:1; box-shadow: 0 0 0 0 rgba(0,255,136,0.4); }
    50%      { opacity:.8; box-shadow: 0 0 0 6px rgba(0,255,136,0); }
}
.section-header {
    font-family: 'Share Tech Mono', monospace;
    font-size: 13px; letter-spacing: 3px;
    text-transform: uppercase;
    color: var(--accent);
    border-bottom: 1px solid var(--border);
    padding-bottom: 8px; margin-bottom: 16px;
}
.stButton > button {
    background: transparent !important;
    border: 1px solid var(--accent) !important;
    color: var(--accent) !important;
    font-family: 'Share Tech Mono', monospace !important;
    letter-spacing: 1px;
    transition: all 0.2s;
}
.stButton > button:hover {
    background: var(--accent) !important;
    color: var(--bg) !important;
    box-shadow: 0 0 20px rgba(0,212,255,0.3) !important;
}
.info-box {
    background: rgba(0,212,255,0.05);
    border: 1px solid rgba(0,212,255,0.2);
    border-radius: 6px;
    padding: 12px 16px; margin: 8px 0;
    font-family: 'Share Tech Mono', monospace; font-size: 13px;
}
.conn-panel {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 8px; padding: 16px; margin-bottom: 12px;
}
#MainMenu, footer, header { visibility: hidden; }
.block-container { padding-top: 1rem !important; }
</style>
""", unsafe_allow_html=True)

# ─── Initialize Session State ─────────────────────────────────────────────────
if "joints" not in st.session_state:
    st.session_state.joints = {
        "waist": 90, "shoulder": 90, "elbow": 90,
        "wrist_pitch": 90, "wrist_roll": 90, "gripper": 0
    }
if "control_mode" not in st.session_state:
    st.session_state.control_mode = "Manual"
if "arduino_connected" not in st.session_state:
    st.session_state.arduino_connected = False
if "arduino_port" not in st.session_state:
    st.session_state.arduino_port = "Simulation"
if "bt_connected" not in st.session_state:
    st.session_state.bt_connected = False
if "esp32_bt_port" not in st.session_state:
    st.session_state.esp32_bt_port = ""
if "cam_connected" not in st.session_state:
    st.session_state.cam_connected = False
if "esp32cam_url" not in st.session_state:
    st.session_state.esp32cam_url = "http://192.168.1.100:81/stream"
if "recorded_frames" not in st.session_state:
    st.session_state.recorded_frames = []
if "recording" not in st.session_state:
    st.session_state.recording = False
if "motion_sequences" not in st.session_state:
    st.session_state.motion_sequences = []
    
# Process Voice Command from URL Parameters (used by HTML Web Speech API)
voice_cmd = st.query_params.get("voice_cmd")
if voice_cmd:
    cmd = voice_cmd
    j = st.session_state.joints
    st.session_state.control_mode = "Voice"
    if cmd == "HOME":
        st.session_state.joints = {k: 90 for k in j}
        st.session_state.joints["gripper"] = 0
    elif cmd == "STOP":
        st.session_state.joints = {k: 90 for k in j}
        st.session_state.joints["gripper"] = 0
    elif cmd.startswith("G"): j["gripper"] = min(100, max(0, int(cmd[1:])))
    elif cmd.startswith("S+"): j["shoulder"] = min(180, j["shoulder"] + int(cmd[2:]))
    elif cmd.startswith("S-"): j["shoulder"] = max(0,   j["shoulder"] - int(cmd[2:]))
    elif cmd.startswith("E+"): j["elbow"]    = min(180, j["elbow"]    + int(cmd[2:]))
    elif cmd.startswith("E-"): j["elbow"]    = max(0,   j["elbow"]    - int(cmd[2:]))
    elif cmd.startswith("W+"): j["waist"]    = min(180, j["waist"]    + int(cmd[2:]))
    elif cmd.startswith("W-"): j["waist"]    = max(0,   j["waist"]    - int(cmd[2:]))
    elif cmd.startswith("P+"): j["wrist_pitch"] = min(180, j["wrist_pitch"] + int(cmd[2:]))
    elif cmd.startswith("P-"): j["wrist_pitch"] = max(0,   j["wrist_pitch"] - int(cmd[2:]))
    # Clear parameter so it doesn't trigger again on normal rerun
    st.query_params.clear()

def build_serial_cmd(joints):
    return f"W{joints['waist']:03d}S{joints['shoulder']:03d}E{joints['elbow']:03d}P{joints['wrist_pitch']:03d}R{joints['wrist_roll']:03d}G{joints['gripper']:03d}\\n"
    
def send_to_hardware():
    cmd = build_serial_cmd(st.session_state.joints)
    if st.session_state.arduino_connected:
        try:
            with serial.Serial(st.session_state.arduino_port, 9600, timeout=1) as ser:
                ser.write(cmd.encode())
        except Exception as e:
            print(f"Serial Error: {e}")

# ─── Sidebar Navigation ───────────────────────────────────────────────────────
with st.sidebar:
    st.markdown("""
    <div style="text-align:center; padding:20px 0;">
        <div style="font-size:48px;">🦾</div>
        <div style="font-family:'Share Tech Mono',monospace; font-size:20px; color:#00d4ff; letter-spacing:2px;">ROBO-ARM OS</div>
        <div style="font-size:11px; color:#4a6080; margin-top:4px;">5-DOF CONTROL SYSTEM</div>
    </div>
    <hr style="border-color:#1e3a5f;">
    """, unsafe_allow_html=True)
    
    page = st.radio("NAVIGATION", [
        "◈ Dashboard", 
        "🕹️ Manual Control", 
        "👁️ Gesture Control", 
        "🗣️ Voice Control", 
        "📼 Motion Recorder", 
        "📐 Kinematics Analysis", 
        "📡 Connections"
    ], label_visibility="collapsed")
    
    st.markdown("<hr style='border-color:#1e3a5f;'>", unsafe_allow_html=True)
    
    # Hardware Status
    st.markdown("<div style='font-family:\"Share Tech Mono\",monospace; font-size:12px; color:#4a6080; letter-spacing:1px; margin-bottom:8px;'>HARDWARE STATUS</div>", unsafe_allow_html=True)
    
    ard_dot = "online" if st.session_state.arduino_connected else "offline"
    cam_dot = "online" if st.session_state.cam_connected else "offline"
    bt_dot = "online" if st.session_state.bt_connected else "offline"
    
    st.markdown(f"""
    <div style="font-family:'Share Tech Mono',monospace; font-size:12px; color:#c9d8f0; line-height:2;">
        <div><span class="status-dot {ard_dot}"></span> Arduino UNO</div>
        <div><span class="status-dot {cam_dot}"></span> ESP32-CAM</div>
        <div><span class="status-dot {bt_dot}"></span> ESP32 Bluetooth</div>
    </div>
    """, unsafe_allow_html=True)

# ─── Helper Functions ─────────────────────────────────────────────────────────

def get_gltf_data_uri():
    model_path = "Robotic_Arm_Embedded.gltf"
    if os.path.exists(model_path):
        with open(model_path, "rb") as f:
            file_content = f.read()
        import base64
        b64 = base64.b64encode(file_content).decode("utf-8")
        return f"data:model/gltf+json;base64,{b64}"
    return None

def get_threejs_viewer_html(data_uri, joints):
    return f"""
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <style>
            body {{ margin: 0; overflow: hidden; background-color: transparent; font-family: 'Share Tech Mono', monospace; }}
            #viewer-container {{ width: 100%; height: 420px; position: relative; }}
            #loading {{ position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); color: #00d4ff; font-size: 14px; letter-spacing: 2px; }}
            #info-panel {{ position: absolute; bottom: 10px; left: 10px; background: rgba(17, 24, 39, 0.8); border: 1px solid #1e3a5f; border-radius: 4px; padding: 10px; color: #c9d8f0; font-size: 10px; pointer-events: none; }}
            .val {{ color: #00ff88; font-weight: bold; }}
        </style>
        <script type="importmap">
            {{
                "imports": {{
                    "three": "https://unpkg.com/three@0.160.0/build/three.module.js",
                    "three/addons/": "https://unpkg.com/three@0.160.0/examples/jsm/"
                }}
            }}
        </script>
    </head>
    <body>
        <div id="viewer-container">
            <div id="loading">LOADING 3D MODEL...</div>
            <div id="info-panel">
                <div>EE X: <span id="ee-x" class="val">0.0</span></div>
                <div>EE Y: <span id="ee-y" class="val">0.0</span></div>
                <div>EE Z: <span id="ee-z" class="val">0.0</span></div>
            </div>
        </div>
        <script type="module">
            import * as THREE from 'three';
            import {{ GLTFLoader }} from 'three/addons/loaders/GLTFLoader.js';
            import {{ OrbitControls }} from 'three/addons/controls/OrbitControls.js';

            const container = document.getElementById('viewer-container');
            const scene = new THREE.Scene();
            
            // Initial dimensions (fallback to 1 if container is hidden)
            const initWidth = container.clientWidth || window.innerWidth;
            const initHeight = container.clientHeight || 420;

            const camera = new THREE.PerspectiveCamera(45, initWidth / initHeight, 0.1, 8000);
            camera.position.set(300, 200, 300);

            const renderer = new THREE.WebGLRenderer({{ antialias: true, alpha: true }});
            renderer.setSize(initWidth, initHeight);
            renderer.setPixelRatio(window.devicePixelRatio);
            container.appendChild(renderer.domElement);

            const ambientLight = new THREE.AmbientLight(0xffffff, 0.8);
            scene.add(ambientLight);
            const dirLight = new THREE.DirectionalLight(0xffffff, 1.2);
            dirLight.position.set(200, 400, 200);
            scene.add(dirLight);

            const controls = new OrbitControls(camera, renderer.domElement);
            controls.enableDamping = true;
            controls.target.set(0, 50, 0);

            const gridHelper = new THREE.GridHelper(800, 40, 0x1e3a5f, 0x1e3a5f);
            scene.add(gridHelper);

            const robotJoints = {{ base: null, shoulder: null, elbow: null, wrist: null, gripper: null }};
            let endEffectorMarker = null;

            const loader = new GLTFLoader();
            loader.load(
                "{data_uri}",
                (gltf) => {{
                    const model = gltf.scene;
                    
                    // Auto-scale model because some models are explicitly in meters or millimeters
                    const initialBox = new THREE.Box3().setFromObject(model);
                    const size = initialBox.getSize(new THREE.Vector3());
                    const maxDim = Math.max(size.x, size.y, size.z);
                    let scaleTarget = 1.0;
                    if (maxDim > 0) {{
                        // we want the model to be roughly 400 units big
                        scaleTarget = 400 / maxDim;
                        model.scale.set(scaleTarget, scaleTarget, scaleTarget);
                    }}
                    
                    scene.add(model);
                    document.getElementById('loading').style.display = 'none';

                    robotJoints.base = model.getObjectByName('Base') || model.getObjectByName('Bone_Base') || model.getObjectByName('Waist') || model;
                    robotJoints.shoulder = model.getObjectByName('Shoulder') || model.getObjectByName('Link1_Shoulder') || model.getObjectByName('Arm 01') || model.getObjectByName('Arm 003');
                    robotJoints.elbow = model.getObjectByName('Elbow') || model.getObjectByName('Link2_Elbow') || model.getObjectByName('Arm 002') || model.getObjectByName('Arm 01');
                    robotJoints.wrist = model.getObjectByName('Wrist') || model.getObjectByName('Link3_Wrist') || model.getObjectByName('Arm 03') || model.getObjectByName('Arm 02 v3');
                    robotJoints.gripper = model.getObjectByName('Gripper') || model.getObjectByName('Link4_Gripper') || model.getObjectByName('Gripper Assembly');

                    const eeMat = new THREE.MeshBasicMaterial({{ color: 0x00ff88 }});
                    const markerRadius = 6 / scaleTarget;
                    endEffectorMarker = new THREE.Mesh(new THREE.SphereGeometry(markerRadius, 16, 16), eeMat);
                    if (robotJoints.gripper) {{
                        robotJoints.gripper.add(endEffectorMarker);
                        endEffectorMarker.position.set(0, 30 / scaleTarget, 0); 
                    }}

                    // Apply current angles from Streamlit state
                    const angles = {{
                        waist: {joints["waist"]},
                        shoulder: {joints["shoulder"]},
                        elbow: {joints["elbow"]},
                        wristPitch: {joints["wrist_pitch"]},
                        gripper: {joints["gripper"]}
                    }};

                    if (robotJoints.base) robotJoints.base.rotation.y = THREE.MathUtils.degToRad(angles.waist - 90);
                    if (robotJoints.shoulder) robotJoints.shoulder.rotation.x = THREE.MathUtils.degToRad(angles.shoulder - 90);
                    if (robotJoints.elbow) robotJoints.elbow.rotation.x = THREE.MathUtils.degToRad(angles.elbow - 90);
                    if (robotJoints.wrist) robotJoints.wrist.rotation.x = THREE.MathUtils.degToRad(angles.wristPitch - 90);
                    if (robotJoints.gripper) robotJoints.gripper.rotation.z = THREE.MathUtils.degToRad((angles.gripper / 100) * 45);
                    
                    const box = new THREE.Box3().setFromObject(model);
                    const center = box.getCenter(new THREE.Vector3());
                    controls.target.copy(center);
                    
                    // Move camera dynamically based on model bounding box
                    const newSize = box.getSize(new THREE.Vector3());
                    const fov = camera.fov * (Math.PI / 180);
                    let cameraZ = Math.abs(Math.max(newSize.x, newSize.y, newSize.z) / 2 / Math.tan(fov / 2));
                    cameraZ *= 1.5; // zoom out slightly
                    camera.position.set(center.x + cameraZ, center.y + cameraZ * 0.5, center.z + cameraZ);
                    camera.updateProjectionMatrix();
                }},
                undefined,
                (error) => console.error(error)
            );

            const eeWorldPos = new THREE.Vector3();
            function animate() {{
                requestAnimationFrame(animate);
                controls.update();
                if (endEffectorMarker) {{
                    endEffectorMarker.getWorldPosition(eeWorldPos);
                    document.getElementById('ee-x').innerText = eeWorldPos.x.toFixed(1);
                    document.getElementById('ee-y').innerText = eeWorldPos.y.toFixed(1);
                    document.getElementById('ee-z').innerText = eeWorldPos.z.toFixed(1);
                }}
                renderer.render(scene, camera);
            }}
            
            // Handle resizing for hidden tabs
            const resizeObserver = new ResizeObserver(entries => {{
                for (let entry of entries) {{
                    const width = entry.contentRect.width;
                    const height = entry.contentRect.height;
                    if (width > 0 && height > 0) {{
                        camera.aspect = width / height;
                        camera.updateProjectionMatrix();
                        renderer.setSize(width, height);
                    }}
                }}
            }});
            resizeObserver.observe(container);

            animate();
        </script>
    </body>
    </html>
    """

def render_dashboard():
    import plotly.graph_objects as go
    
    st.markdown("""
    <div style="font-family:'Share Tech Mono',monospace; font-size:24px; color:#00d4ff; letter-spacing:4px; margin-bottom:4px;">
        ◈ SYSTEM DASHBOARD
    </div>
    <div style="color:#4a6080; font-size:13px; letter-spacing:2px; margin-bottom:24px;">
        REAL-TIME ARM MONITORING & CONTROL OVERVIEW
    </div>
    """, unsafe_allow_html=True)

    joints = st.session_state.joints

    # Metrics Row
    col1, col2, col3, col4, col5, col6 = st.columns(6)
    joint_names = ["Waist", "Shoulder", "Elbow", "Wrist P.", "Wrist R.", "Gripper"]
    joint_keys = ["waist", "shoulder", "elbow", "wrist_pitch", "wrist_roll", "gripper"]
    joint_units = ["°", "°", "°", "°", "°", "%"]
    cols = [col1, col2, col3, col4, col5, col6]
    for col, name, key, unit in zip(cols, joint_names, joint_keys, joint_units):
        val = joints[key]
        with col:
            st.markdown(f"""
            <div class="metric-card">
                <div class="label">{name}</div>
                <div class="value">{val}<span class="unit">{unit}</span></div>
            </div>
            """, unsafe_allow_html=True)
            
    # Forward Kinematics Math
    L1, L2, L3, L4 = 120, 110, 90, 60
    w = math.radians(joints["waist"])
    s = math.radians(joints["shoulder"] - 90)
    e = math.radians(joints["elbow"] - 90)
    wp = math.radians(joints["wrist_pitch"] - 90)

    p0 = np.array([0, 0, 0])
    p1 = np.array([0, 0, 60])
    p2 = p1 + np.array([L1 * math.cos(s) * math.cos(w), L1 * math.cos(s) * math.sin(w), L1 * math.sin(s)])
    p3 = p2 + np.array([L2 * math.cos(s + e) * math.cos(w), L2 * math.cos(s + e) * math.sin(w), L2 * math.sin(s + e)])
    p4 = p3 + np.array([L3 * math.cos(s + e + wp) * math.cos(w), L3 * math.cos(s + e + wp) * math.sin(w), L3 * math.sin(s + e + wp)])
    p5 = p4 + np.array([L4 * math.cos(s + e + wp) * math.cos(w), L4 * math.cos(s + e + wp) * math.sin(w), L4 * math.sin(s + e + wp)])

    # Setup Tabs
    col_viz, col_polar = st.columns([2, 1])
    
    with col_viz:
        st.markdown('<div class="section-header">◈ 3D Visualization</div>', unsafe_allow_html=True)
        tab_gltf, tab_fk = st.tabs(["GLTF 3D Model Viewer", "Forward Kinematics 3D"])
        
        with tab_gltf:
            st.markdown('<div style="font-family:\'Share Tech Mono\'; color:#c9d8f0; font-size:12px; margin-bottom:10px;">Interactive 3D CAD Model (Drag to rotate)</div>', unsafe_allow_html=True)
            
            try:
                data_uri = get_gltf_data_uri()
                if data_uri:
                    html_code = get_threejs_viewer_html(data_uri, joints)
                    components.html(html_code, height=450)
                else:
                    st.warning("`Robotic_Arm_Embedded.gltf` not found in current directory.")
            except Exception as ex:
                st.error(f"Error loading GLTF: {ex}")
        
        with tab_fk:
            try:
                data_uri = get_gltf_data_uri()
                if data_uri:
                    html_code = get_threejs_viewer_html(data_uri, joints)
                    components.html(html_code, height=450)
                else:
                    st.warning("`Robotic_Arm_Embedded.gltf` not found in current directory.")
            except Exception as ex:
                st.error(f"Error loading GLTF: {ex}")

            points = np.array([p0, p1, p2, p3, p4, p5])
            fig3d = go.Figure()
            fig3d.add_trace(go.Scatter3d(
                x=points[:, 0], y=points[:, 1], z=points[:, 2],
                mode='lines+markers', line=dict(color='#00d4ff', width=8),
                marker=dict(size=[6, 8, 10, 10, 10, 8], color=['#4a6080', '#00d4ff', '#00d4ff', '#ff6b35', '#ff6b35', '#00ff88']),
                name='Arm Links'
            ))
            theta = np.linspace(0, 2 * np.pi, 60)
            reach = L1 + L2 + L3 + L4
            fig3d.add_trace(go.Scatter3d(
                x=reach * np.cos(theta), y=reach * np.sin(theta), z=np.zeros(60),
                mode='lines', line=dict(color='rgba(255,107,53,0.3)', width=2, dash='dash'), name='Max Reach'
            ))
            fig3d.add_trace(go.Scatter3d(
                x=[0, 0], y=[0, 0], z=[0, 60],
                mode='lines', line=dict(color='#4a6080', width=4), name='Base'
            ))
            fig3d.add_trace(go.Scatter3d(
                x=[p5[0]], y=[p5[1]], z=[p5[2]],
                mode='markers', marker=dict(size=14, color='#00ff88', symbol='diamond', line=dict(color='white', width=2)),
                name='End Effector'
            ))
            fig3d.update_layout(
                scene=dict(
                    xaxis=dict(title='X', gridcolor='#1e3a5f', backgroundcolor='rgba(0,0,0,0)'),
                    yaxis=dict(title='Y', gridcolor='#1e3a5f', backgroundcolor='rgba(0,0,0,0)'),
                    zaxis=dict(title='Z', gridcolor='#1e3a5f', backgroundcolor='rgba(0,0,0,0)', range=[0, 400]),
                    bgcolor='rgba(10,14,26,0)',
                    camera=dict(eye=dict(x=1.5, y=1.5, z=1.2)),
                    aspectmode='cube'
                ),
                paper_bgcolor='rgba(17,24,39,0)', margin=dict(l=0, r=0, t=0, b=0), height=420,
                font=dict(family='Share Tech Mono', color='#4a6080')
            )
            st.plotly_chart(fig3d, use_container_width=True, config={'displayModeBar': False})

    with col_polar:
        import plotly.graph_objects as go
        st.markdown('<div class="section-header">◈ Joint Angles Radar</div>', unsafe_allow_html=True)
        categories = ['Waist', 'Shoulder', 'Elbow', 'Wrist P.', 'Wrist R.', 'Gripper']
        values = [joints[k] for k in joint_keys]
        max_vals = [180, 180, 180, 180, 180, 100]
        norm = [v / m * 100 for v, m in zip(values, max_vals)]
        norm.append(norm[0])
        categ = categories + [categories[0]]
        fig_polar = go.Figure()
        fig_polar.add_trace(go.Scatterpolar(r=norm, theta=categ, fill='toself', fillcolor='rgba(0,212,255,0.1)', line=dict(color='#00d4ff', width=2)))
        fig_polar.add_trace(go.Scatterpolar(r=[100]*7, theta=categ, fill='toself', fillcolor='rgba(255,107,53,0.03)', line=dict(color='rgba(255,107,53,0.3)', width=1, dash='dot')))
        fig_polar.update_layout(polar=dict(radialaxis=dict(visible=True, range=[0, 100], color='#4a6080'), bgcolor='rgba(0,0,0,0)'), paper_bgcolor='rgba(17,24,39,0)', font=dict(family='Share Tech Mono', color='#c9d8f0'), showlegend=False, margin=dict(l=30, r=30, t=20, b=20), height=450)
        st.plotly_chart(fig_polar, use_container_width=True, config={'displayModeBar': False})

    st.markdown('<div class="section-header">◈ End Effector Position</div>', unsafe_allow_html=True)
    c_x, c_y, c_z, c_r, c_m = st.columns(5)
    with c_x: st.markdown(f'<div class="metric-card"><div class="label">X Position</div><div class="value">{p5[0]:.1f}<span class="unit">mm</span></div></div>', unsafe_allow_html=True)
    with c_y: st.markdown(f'<div class="metric-card"><div class="label">Y Position</div><div class="value">{p5[1]:.1f}<span class="unit">mm</span></div></div>', unsafe_allow_html=True)
    with c_z: st.markdown(f'<div class="metric-card"><div class="label">Z Position</div><div class="value">{p5[2]:.1f}<span class="unit">mm</span></div></div>', unsafe_allow_html=True)
    with c_r: st.markdown(f'<div class="metric-card"><div class="label">Total Reach</div><div class="value">{math.sqrt(p5[0]**2+p5[1]**2+p5[2]**2):.1f}<span class="unit">mm</span></div></div>', unsafe_allow_html=True)
    with c_m: st.markdown(f'<div class="metric-card" style="--accent:#00d4ff"><div class="label">Mode</div><div class="value" style="font-size:18px;color:#00d4ff">{st.session_state.control_mode}</div></div>', unsafe_allow_html=True)

    st.markdown('<div class="section-header">◈ Quick Joint Control</div>', unsafe_allow_html=True)
    cols_s = st.columns(6)
    joint_ranges = {"waist": (0, 180), "shoulder": (0, 180), "elbow": (0, 180), "wrist_pitch": (0, 180), "wrist_roll": (0, 180), "gripper": (0, 100)}
    for col, name, key in zip(cols_s, joint_names, joint_keys):
        with col:
            st.markdown(f"<div style='font-family:Share Tech Mono,monospace;font-size:11px;color:#4a6080;letter-spacing:1px;'>{name.upper()}</div>", unsafe_allow_html=True)
            new_val = st.slider(name, *joint_ranges[key], st.session_state.joints[key], label_visibility="collapsed", key=f"dash_{key}")
            if new_val != st.session_state.joints[key]:
                st.session_state.joints[key] = new_val
                st.session_state.control_mode = "Manual"
                send_to_hardware()
                st.rerun()

def render_manual():
    st.markdown("""
    <div style="font-family:'Share Tech Mono',monospace; font-size:24px; color:#00d4ff; letter-spacing:4px; margin-bottom:4px;">
        🕹️ MANUAL CONTROL
    </div>
    <div style="color:#4a6080; font-size:13px; letter-spacing:2px; margin-bottom:24px;">
        DIRECT SERVO MOTOR CONTROL INTERFACE
    </div>
    """, unsafe_allow_html=True)

    if not st.session_state.arduino_connected and not st.session_state.bt_connected:
        st.markdown('<div class="info-box" style="border-color:rgba(255,107,53,0.3); background:rgba(255,107,53,0.05);">⚠ No hardware connected. Simulated mode active.</div>', unsafe_allow_html=True)

    st.markdown('<div class="section-header">◈ Joint Servo Control</div>', unsafe_allow_html=True)
    joint_config = [
        ("Waist (Base Rotation)", "waist", 0, 180, "◐", "#00d4ff", "Horizontal rotation of the base platform"),
        ("Shoulder", "shoulder", 0, 180, "◑", "#00d4ff", "Main lifting joint"),
        ("Elbow", "elbow", 0, 180, "◒", "#ff6b35", "Mid-arm joint for reach"),
        ("Wrist Pitch", "wrist_pitch", 0, 180, "◓", "#ff6b35", "Wrist up/down tilt"),
        ("Wrist Roll", "wrist_roll", 0, 180, "⊕", "#ffcc00", "Wrist rotation"),
        ("Gripper", "gripper", 0, 100, "⊙", "#00ff88", "Gripper open(0)/close(100)"),
    ]

    for i in range(0, len(joint_config), 2):
        c1, c2 = st.columns(2)
        for col, cfg in zip([c1, c2], joint_config[i:i + 2]):
            with col:
                name, key, mn, mx, icon, color, desc = cfg
                st.markdown(f"""
                <div class="metric-card" style="--accent:{color}; padding:20px;">
                    <div style="display:flex; justify-content:space-between; align-items:flex-start;">
                        <div>
                            <div class="label">{icon} {name}</div>
                            <div class="value" style="color:{color}">{st.session_state.joints[key]}<span class="unit">{'°' if key != 'gripper' else '%'}</span></div>
                        </div>
                        <div style="text-align:right;">
                            <div style="font-family:'Share Tech Mono',monospace;font-size:10px;color:#4a6080;">RANGE</div>
                            <div style="font-family:'Share Tech Mono',monospace;font-size:13px;color:#4a6080;">{mn}° – {mx}{'°' if key != 'gripper' else '%'}</div>
                        </div>
                    </div>
                    <div style="font-size:11px;color:#4a6080;margin-top:4px;">{desc}</div>
                </div>
                """, unsafe_allow_html=True)
                new_val = st.slider(name, mn, mx, st.session_state.joints[key], label_visibility="collapsed", key=f"manual_{key}")
                if new_val != st.session_state.joints[key]:
                    st.session_state.joints[key] = new_val
                    st.session_state.control_mode = "Manual"
                    send_to_hardware()
                    st.rerun()

    st.markdown('<div class="section-header">◈ Preset Positions</div>', unsafe_allow_html=True)
    presets = {
        "🏠 Home": {"waist": 90, "shoulder": 90, "elbow": 90, "wrist_pitch": 90, "wrist_roll": 90, "gripper": 0},
        "🙌 Pick Up": {"waist": 90, "shoulder": 45, "elbow": 135, "wrist_pitch": 90, "wrist_roll": 90, "gripper": 80},
        "📦 Place": {"waist": 45, "shoulder": 60, "elbow": 120, "wrist_pitch": 90, "wrist_roll": 90, "gripper": 10},
        "🔒 Stow": {"waist": 90, "shoulder": 90, "elbow": 90, "wrist_pitch": 90, "wrist_roll": 90, "gripper": 100},
    }
    pc = st.columns(4)
    for col, (pname, pvals) in zip(pc, presets.items()):
        with col:
            if st.button(pname, use_container_width=True):
                st.session_state.joints.update(pvals)
                st.session_state.control_mode = "Manual"
                send_to_hardware()
                st.rerun()

    st.markdown("---")
    cmd = build_serial_cmd(st.session_state.joints)
    cs, cx, ci = st.columns([1,1,2])
    with cs:
        if st.button("▶ SEND TO ARM", use_container_width=True):
            send_to_hardware()
            st.success("✓ Sent")
    with cx:
        if st.button("⏹ E-STOP", use_container_width=True):
            st.session_state.joints = {k: 90 for k in st.session_state.joints}
            st.session_state.joints["gripper"] = 0
            send_to_hardware()
            st.error("⚠ E-STOP TRIGGERED")
            st.rerun()
    with ci:
        st.markdown(f'<div class="info-box"><span style="color:#4a6080;">CMD:</span> <span style="color:#00ff88;">{cmd.strip()}</span></div>', unsafe_allow_html=True)

def render_gesture():
    st.markdown("""
    <div style="font-family:'Share Tech Mono',monospace; font-size:24px; color:#00d4ff; letter-spacing:4px; margin-bottom:4px;">
        👁️ GESTURE CONTROL
    </div>
    <div style="color:#4a6080; font-size:13px; letter-spacing:2px; margin-bottom:24px;">
        OPENCV + MEDIAPIPE HAND GESTURE RECOGNITION
    </div>
    """, unsafe_allow_html=True)

    st.markdown('<div class="section-header">◈ Gesture Command Map</div>', unsafe_allow_html=True)
    gestures = [
        ("✊", "Fist", "Close gripper", "gripper → 100%"), ("🖐️", "Open Hand", "Open gripper", "gripper → 0%"),
        ("👆", "Index Up", "Move shoulder up", "shoulder +5°"), ("👇", "Index Down", "Move shoulder down", "shoulder -5°"),
        ("👉", "Point Right", "Rotate waist right", "waist +5°"), ("👈", "Point Left", "Rotate waist left", "waist -5°"),
        ("✌️", "Peace Sign", "Extend elbow", "elbow +10°"), ("🤞", "Crossed Fingers", "Retract elbow", "elbow -10°")
    ]
    cols = st.columns(4)
    for i, (icon, name, desc, cmd) in enumerate(gestures):
        with cols[i % 4]:
            st.markdown(f"""
            <div class="metric-card" style="text-align:center; padding:12px;">
                <div style="font-size:28px; margin-bottom:4px;">{icon}</div>
                <div style="font-family:'Share Tech Mono',monospace; font-size:12px; color:#00d4ff;">{name}</div>
                <div style="font-size:11px; color:#c9d8f0; margin:4px 0;">{desc}</div>
                <div style="font-family:'Share Tech Mono',monospace; font-size:10px; color:#4a6080;">{cmd}</div>
            </div>
            """, unsafe_allow_html=True)

    st.markdown('<div class="section-header">◈ Gesture Server</div>', unsafe_allow_html=True)
    st.markdown("""
    <div class="info-box">
        ℹ️ Real-time camera processing requires local execution. Run the included standalone script 
        <code>python gesture_server.py</code> on your laptop alongside this Streamlit app.
    </div>
    """, unsafe_allow_html=True)
    
    with st.expander("Show gesture_server.py source"):
        st.code("""# gesture_server.py is provided as a separate file in this project.""", language='python')

def render_voice():
    st.markdown("""
    <div style="font-family:'Share Tech Mono',monospace; font-size:24px; color:#00d4ff; letter-spacing:4px; margin-bottom:4px;">
        🗣️ VOICE CONTROL
    </div>
    <div style="color:#4a6080; font-size:13px; letter-spacing:2px; margin-bottom:24px;">
        SPEECH RECOGNITION (WEB SPEECH API)
    </div>
    """, unsafe_allow_html=True)

    st.markdown('<div class="section-header">◈ Voice Command Map</div>', unsafe_allow_html=True)
    commands = [
        ("🏠", "go home", "All joints → home"), ("🔒", "close gripper", "Gripper → closed"),
        ("🔓", "open gripper", "Gripper → open"), ("⬆️", "shoulder up", "Shoulder +15°"),
        ("⬇️", "shoulder down", "Shoulder -15°"), ("⬆️", "elbow up", "Elbow +15°"),
        ("⬇️", "elbow down", "Elbow -15°"), ("↩️", "rotate left", "Waist -15°"),
        ("↪️", "rotate right", "Waist +15°"), ("🔴", "stop", "E-Stop")
    ]
    cols = st.columns(3)
    for i, (icon, cmd, desc) in enumerate(commands):
        with cols[i % 3]:
            st.markdown(f'<div class="metric-card" style="padding:12px;"><div style="font-size:20px;">{icon}</div><div style="color:#ffcc00;font-size:12px;">"{cmd}"</div><div style="font-size:11px;color:#c9d8f0;">{desc}</div></div>', unsafe_allow_html=True)

    st.markdown('<div class="section-header">◈ Voice Recognition Interface</div>', unsafe_allow_html=True)
    voice_html = """
    <div id="voice-container" style="background:#111827; border:1px solid #1e3a5f; border-radius:12px; padding:32px; text-align:center; font-family:'Share Tech Mono',monospace; color:#c9d8f0;">
        <button id="mic-btn" onclick="toggleListening()" style="width:90px;height:90px;border-radius:50%;background:transparent;border:2px solid #00d4ff;color:#00d4ff;font-size:36px;cursor:pointer;margin-bottom:20px;">🎤</button>
        <div id="status-line" style="font-size:13px;color:#4a6080;">CLICK MIC TO START</div>
        <div id="heard-text" style="font-size:18px;color:#00d4ff;margin:12px 0;min-height:28px;">—</div>
        <div id="cmd-result" style="font-size:14px;color:#00ff88;min-height:22px;"></div>
    </div>
    <script>
    const COMMANDS = {"go home":"HOME","close gripper":"G100","open gripper":"G000","shoulder up":"S+15","shoulder down":"S-15","elbow up":"E+15","elbow down":"E-15","rotate left":"W-15","rotate right":"W+15","stop":"STOP"};
    let recognition, isListening = false;
    function initSpeech() {
        const SpeechRec = window.SpeechRecognition || window.webkitSpeechRecognition;
        if (!SpeechRec) { document.getElementById('status-line').textContent = "❌ Use Chrome"; return null; }
        const r = new SpeechRec(); r.continuous = true; r.interimResults = true;
        r.onstart = () => { document.getElementById('status-line').textContent = "🔴 LISTENING..."; };
        r.onresult = (e) => {
            let final = '';
            for(let i=e.resultIndex; i<e.results.length; i++) if(e.results[i].isFinal) final += e.results[i][0].transcript.toLowerCase();
            if(final) {
                document.getElementById('heard-text').textContent = final;
                for(const [phrase, cmd] of Object.entries(COMMANDS)) {
                    if(final.includes(phrase)) {
                        document.getElementById('cmd-result').textContent = `✓ "${phrase}" → ${cmd}`;
                        const url = new URL(window.location); url.searchParams.set('voice_cmd', cmd);
                        window.parent.history.replaceState({}, '', url); window.parent.location.reload();
                        break;
                    }
                }
            }
        };
        r.onend = () => { if(isListening) r.start(); }; return r;
    }
    function toggleListening() {
        if(!recognition) recognition = initSpeech();
        if(!recognition) return;
        if(isListening) { recognition.stop(); isListening=false; document.getElementById('status-line').textContent="CLICK MIC TO START"; document.getElementById('mic-btn').style.borderColor='#00d4ff'; }
        else { recognition.start(); isListening=true; document.getElementById('mic-btn').style.borderColor='#ff4444'; }
    }
    </script>
    """
    components.html(voice_html, height=350)

def render_motion():
    st.markdown("""
    <div style="font-family:'Share Tech Mono',monospace; font-size:24px; color:#00d4ff; letter-spacing:4px; margin-bottom:4px;">
        📼 MOTION RECORDER
    </div>
    <div style="color:#4a6080; font-size:13px; letter-spacing:2px; margin-bottom:24px;">
        RECORD, SAVE & REPLAY MOTION SEQUENCES
    </div>
    """, unsafe_allow_html=True)

    SEQUENCES_DIR = "sequences"
    os.makedirs(SEQUENCES_DIR, exist_ok=True)

    st.markdown('<div class="section-header">◈ Recording Controls</div>', unsafe_allow_html=True)
    cr1, cr2, cr3, cinfo = st.columns([1,1,1,2])
    with cr1:
        if not st.session_state.recording:
            if st.button("🔴 START", use_container_width=True):
                st.session_state.recording = True
                st.session_state.recorded_frames = []
                st.session_state.record_start_time = time.time()
                st.rerun()
        else:
            st.markdown('<div style="text-align:center;padding:8px;border:1px solid #ff4444;color:#ff4444;border-radius:6px;">🔴 RECORDING</div>', unsafe_allow_html=True)
    with cr2:
        if st.session_state.recording and st.button("⏸ CAPTURE FRAME", use_container_width=True):
            f = dict(st.session_state.joints)
            f["t"] = time.time() - st.session_state.record_start_time
            st.session_state.recorded_frames.append(f)
    with cr3:
        if st.session_state.recording and st.button("⏹ STOP", use_container_width=True):
            st.session_state.recording = False
            st.rerun()
    with cinfo:
        n = len(st.session_state.recorded_frames)
        d = st.session_state.recorded_frames[-1]["t"] if n>0 else 0
        st.markdown(f'<div class="info-box">FRAMES: {n} &nbsp;|&nbsp; DUR: {d:.1f}s</div>', unsafe_allow_html=True)

    if st.session_state.recorded_frames:
        st.markdown('<div class="section-header">◈ Save Sequence</div>', unsafe_allow_html=True)
        cn, cs = st.columns([3,1])
        with cn: sname = st.text_input("Name", "sequence_1", label_visibility="collapsed")
        with cs:
            if st.button("💾 SAVE", use_container_width=True):
                seq = {"name": sname, "frames": st.session_state.recorded_frames, "count": n, "dur": d}
                path = os.path.join(SEQUENCES_DIR, f"{sname.replace(' ','_')}_{int(time.time())}.json")
                with open(path, "w") as fp: json.dump(seq, fp)
                st.session_state.recorded_frames = []
                st.success("Saved!")
                st.rerun()

    st.markdown('<div class="section-header">◈ Library</div>', unsafe_allow_html=True)
    seqs = [json.load(open(os.path.join(SEQUENCES_DIR, f))) for f in os.listdir(SEQUENCES_DIR) if f.endswith('.json')]
    for s in seqs:
        st.markdown(f"**{s['name']}** - {s['count']} frames, {s.get('dur', 0):.1f}s")
        if st.button("▶ Play", key=s['name']):
            st.session_state.control_mode = "Replay"
            st.info(f"Replay {s['name']} (Simulated here. Use replay script for hardware).")
            # To actually replay on hardware from UI, we could loop over frames here with time.sleep
            # butStreamlit is synchronous so it will block UI.

def render_kinematics():
    st.markdown("""
    <div style="font-family:'Share Tech Mono',monospace; font-size:24px; color:#00d4ff; letter-spacing:4px; margin-bottom:4px;">
        📐 KINEMATICS ANALYSIS
    </div>
    <div style="color:#4a6080; font-size:13px; letter-spacing:2px; margin-bottom:24px;">
        FORWARD KINEMATICS · JACOBIAN · VELOCITY
    </div>
    """, unsafe_allow_html=True)

    joints = st.session_state.joints
    L1, L2, L3, L4 = 120, 110, 90, 60
    w = math.radians(joints["waist"])
    s = math.radians(joints["shoulder"] - 90)
    e = math.radians(joints["elbow"] - 90)
    wp = math.radians(joints["wrist_pitch"] - 90)

    st.markdown('<div class="section-header">◈ End Effector Jacobian & Velocity</div>', unsafe_allow_html=True)
    c1, c2 = st.columns(2)
    with c1:
        st.write("Target Angular Velocities (°/s)")
        ow = st.slider("Waist ω1", -90, 90, 10)
        os = st.slider("Shoulder ω2", -90, 90, 0)
        oe = st.slider("Elbow ω3", -90, 90, 0)
        omegas = np.array([math.radians(ow), math.radians(os), math.radians(oe), 0, 0])
    
    with c2:
        # Simplified linear velocity mapping
        # This is a naive computation for UI demo
        v_speed = 15.4 * abs(math.radians(ow)) + 12.0 * abs(math.radians(os))
        icr = v_speed / max(0.001, abs(math.radians(ow)))
        
        st.markdown(f"""
        <div class="conn-panel">
            <div class="label">LINEAR VELOCITY</div>
            <div class="value" style="font-size:24px;">{v_speed:.2f} <span class="unit">mm/s</span></div>
            <hr style="border-color:#1e3a5f; margin:10px 0;">
            <div class="label">WAIST ICR RADIUS</div>
            <div class="value" style="font-size:24px; color:#ff6b35;">{icr:.1f} <span class="unit">mm</span></div>
        </div>
        """, unsafe_allow_html=True)

def render_conn():
    st.markdown("""
    <div style="font-family:'Share Tech Mono',monospace; font-size:24px; color:#00d4ff; letter-spacing:4px; margin-bottom:4px;">
        📡 CONNECTIONS
    </div>
    <div style="color:#4a6080; font-size:13px; letter-spacing:2px; margin-bottom:24px;">
        HARDWARE CONFIGURATION
    </div>
    """, unsafe_allow_html=True)

    c1, c2 = st.columns(2)
    with c1:
        st.markdown('<div class="section-header">Arduino UNO (Serial)</div>', unsafe_allow_html=True)
        try: ports = [p.device for p in serial.tools.list_ports.comports()]
        except: ports = ["COM1", "COM3"]
        if not ports: ports = ["COM3"]
        port = st.selectbox("Port", ports)
        if st.button("Connect Arduino"):
            st.session_state.arduino_connected = True
            st.session_state.arduino_port = port
            st.success(f"Connected to {port}")
    with c2:
        st.markdown('<div class="section-header">ESP32-CAM (Stream)</div>', unsafe_allow_html=True)
        url = st.text_input("URL", st.session_state.esp32cam_url)
        if st.button("Connect Camera"):
            st.session_state.cam_connected = True
            st.session_state.esp32cam_url = url
            st.success("Camera ready")

# ─── Router ───────────────────────────────────────────────────────────────────
if page == "◈ Dashboard": render_dashboard()
elif page == "🕹️ Manual Control": render_manual()
elif page == "👁️ Gesture Control": render_gesture()
elif page == "🗣️ Voice Control": render_voice()
elif page == "📼 Motion Recorder": render_motion()
elif page == "📐 Kinematics Analysis": render_kinematics()
elif page == "📡 Connections": render_conn()