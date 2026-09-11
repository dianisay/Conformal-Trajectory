# 🤖 Bioprinting Robot Control System

## What Does This Do?

This folder contains the complete software to control a **robotic bioprinting system** that can autonomously locate and position a target using a camera, XY cartesian platform, and MyCobot 280 robotic arm.

**In simple terms:**
1. A camera finds your target
2. An XY platform moves to get close
3. A robotic arm does the precise final positioning
4. The system measures how accurate it was

---

## 📋 Quick Reference: What's in This Folder

| File | Purpose |
|------|---------|
| **`junto2.m`** | Main MATLAB program ⭐ START HERE |
| **`mc_bridge.py`** | Connects MATLAB to the robot arm |
| **`cameraParams.mat`** | Camera calibration data (do not modify) |
| **`trainedNet.mat`** | AI model for coordinate transformation (do not modify) |
| **`gcode_parser.m`** | Parse a G-code file into waypoints + command metadata |
| **`gcode_executor.m`** | Execute parsed G-code on the XY stage + MyCobot Z axis |
| **`junto2_gcode.m` / `exec_gcode.m`** | G-code-first workflow entry points |

---

## 🧾 G-code Workflow

If you already have a pre-computed conformal toolpath, you can now run it directly:

```matlab
cd Robot8DoF
[traj, meta] = gcode_parser('C:/absolute/path/conformal_print.gcode');
[logData, report] = gcode_executor(traj, s, mc, struct('plotTelemetry', true));

% or use the one-shot entry point
[logData, report, traj] = exec_gcode('C:/absolute/path/conformal_print.gcode');
```

`gcode_parser.m` supports `G0/G1`, `G28`, `G90/G91`, `G92`, feedrate inheritance, inline comments, and common M-codes while preserving absolute waypoint output for execution.

## 🚀 Quick Start (5 Minutes)

### Prerequisites
- **MATLAB** (with Image Processing, Computer Vision, and Neural Network toolboxes)
- **Python 3.11** (specifically)
- **MyCobot 280** robotic arm (powered on, connected)
- **Webcam** (1280×720 resolution)
- **XY Cartesian platform** (with GRBL, Marlin, or Klipper firmware)

### Step 1: Configure Your Hardware Ports

Open `junto2.m` and find these lines (around line 29 and 71):

```matlab
% Line 29: Your XY platform serial port
port = 'COM5';  % ← Change to your actual port (COM3, COM4, etc.)

% Line 71: Your Python 3.11 path
pyExe = "C:\Users\MonDi\AppData\Local\Programs\Python\Python311\python.exe";  % ← Update this path
```

**How to find your ports:**
- **Windows:** Device Manager → Ports (COM & LPT)
- **Python path:** Open Command Prompt and type `python --version` to verify location

### Step 2: Install Python Dependencies

Open Command Prompt and run:
```bash
pip install pymycobot
```

### Step 3: Run the Program

1. Open MATLAB
2. Navigate to the `robot/` folder
3. Open `junto2.m`
4. Press **Run** (or Ctrl+Enter)
5. **Press 'Q'** on the figure window to stop

---

## 🔍 How the System Works (Detailed Flow)

### **Phase 1: Vision & Camera Detection**
```
Camera captures image (1280×720)
    ↓
Remove lens distortion
    ↓
Convert to grayscale
    ↓
Detect target (binary segmentation + morphology)
    ↓
Find center of target (centroid)
    ↓
Convert pixel coordinates → millimeters
```

**Key parameters** (in `junto2.m`):
```matlab
Zc = 650;          % Camera plane depth (mm)
tol_mm = 5.0;      % Stop when within 5mm of target
```

---

### **Phase 2: XY Platform Positioning (PD Controller)**
```
Target position (from camera) - Current position
    ↓
Use AI model to transform coordinates
    ↓
Calculate error
    ↓
PD Controller calculates motor command
    ↓
XY platform moves incrementally
    ↓
Repeat until distance < 4mm
```

**Key parameters** (in `junto2.m`):
```matlab
Kp = 0.8;          % Proportional gain
Kd = 0.1;          % Derivative gain
maxStepXY = 15.0;  % Max mm per step
```

**Supported platforms:**
- GRBL (CNC machines)
- Marlin/RepRap (3D printers)
- Klipper (3D printers)

The system auto-detects which one you have!

---

### **Phase 3: Robot Arm Positioning (MyCobot 280)**

When the XY platform gets close (< 4mm), the robotic arm takes over:

```
Take a new image (reduce accumulated error)
    ↓
Calculate final target position
    ↓
Check safety limits (260mm radius)
    ↓
Generate smooth trajectory (sigmoidal path)
    ↓
Move robot arm in real-time
    ↓
Measure actual vs. desired position (telemetry)
    ↓
Calculate accuracy metrics (RMSE)
```

**Target position:**
```matlab
[X_target, Y_target, 320mm]  % Z=320mm is fixed approach height
```

**Safety radius:** 260mm (adjustable at line 288)

---

### **Phase 4: Performance Analysis**

After completion, the system generates:
- 📊 3D trajectory plot (desired vs. actual)
- 📊 Error by axis (X, Y, Z)
- 📊 Total error magnitude over time
- 📈 RMSE metrics printed to console

---

## ⚙️ Configuration Parameters (What to Adjust)

Open `junto2.m` and look for these sections:

### Camera Setup (Line 4-6)
```matlab
res = "1280x720";      % Camera resolution
nameContains = "";     % Filter camera by name (leave empty for default)
```

### Vision Thresholds (Line 53-56)
```matlab
Kp = 0.8;              % Higher = more aggressive
Kd = 0.1;              % Higher = more damping
tol_mm = 5.0;          % Stop distance (mm)
maxStepXY = 15.0;      % Max movement per cycle (mm)
```

### Robot Workspace (Line 24-26)
```matlab
Xmin = -280;  Xmax = 280;    % X limits (mm)
Ymin = -280;  Ymax = 280;    % Y limits (mm)
Rsafe = 260;                  % Safety radius (mm)
z_min = 80;                   % Minimum Z height (mm)
```

### Trajectory Settings (Line 311-324)
```matlab
T = 7.0;       % Total trajectory time (seconds)
dt = 0.03;     % Command interval (seconds)
z_min = 80;    % Minimum Z during movement (mm)
```

---

## 🐍 Python Bridge (`mc_bridge.py`)

This file connects MATLAB to the MyCobot robot. It handles:

- ✅ Robot homing (moving to HOME position)
- ✅ Reading current pose (position + orientation)
- ✅ Cartesian movement (XYZ coordinates)
- ✅ Joint movement (individual angle control)
- ✅ Telemetry (accurate position feedback)
- ✅ Error handling & retries

**Configuration** (if needed, top of `mc_bridge.py`):
```python
PORT = "COM6"           # Your robot's USB port
BAUD = 115200           # MyCobot 280 M5 uses 115200 or 1000000
SPEED_DEFAULT = 50      # Movement speed (1-100)
MOVE_MODE = 1           # 1=Cartesian (smooth), 0=Joint (direct)
TIMEOUT = 20.0          # Max wait time (seconds)
```

---

## 📊 Understanding the Output

After running, you'll see 4 graphs:

### Graph 1: Real-time Camera Feed
- Shows the current frame
- Red line: trajectory of detected target centroid
- Blue star: current centroid position
- Yellow text: distance to target, coordinates

### Graph 2: Camera Inputs (Xc, Yc)
- What the camera sees (in mm)
- Should correlate with actual motion

### Graph 3: Predicted Outputs (X, Y)
- What the AI model predicted
- This drives the robot movement

### Graph 4: Trajectory in 3D
- Purple dots: desired path (reference)
- Blue line: actual path taken
- Shows how well the robot followed commands

---

## ❌ Troubleshooting

### "Error: Cannot find camera"
- Make sure webcam is connected and drivers are installed
- Check resolution is exactly 1280×720 (Camera Settings app)

### "Error: COM port not found"
- Verify XY platform is powered on
- Check Device Manager for correct port number
- Update line 29 in `junto2.m`

### "Error: Python module 'mc_bridge' not found"
- Make sure you're in the robot folder (where `mc_bridge.py` is)
- Verify Python 3.11 path is correct (line 71)
- Run: `pip install pymycobot` in Command Prompt

### "Error: No object found"
- Make sure target is clearly visible in camera
- Check lighting (should be well-lit)
- Verify camera is still calibrated (don't move it)

### Robot arm not moving
- Check MyCobot is powered on (LED indicator)
- Verify USB connection
- Try restarting the robot and MATLAB
- Check safety radius (maybe target is outside workspace)

### Inaccurate positioning
- Run camera calibration (requires calibration pattern)
- Verify cameraParams.mat is up to date
- Check XY platform firmware responds correctly
- Adjust Kp/Kd gains slowly (test with Kp=0.5 first)

---

## 🔧 Advanced: Modifying the AI Model

The neural network transforms camera coordinates to robot coordinates:
- **Input:** (Xc, Yc) from camera
- **Output:** (X, Y) for robot
- **Loaded from:** `trainedNet.mat`

To retrain with your own data:
1. Collect training examples (camera input → ground truth output)
2. Train a new neural network in MATLAB
3. Save as `trainedNet.mat` with variables: `net`, `psX`, `psT`
4. Replace the existing file

---

## 📝 File Manifest

```
robot/
├── README.md                 ← You are here
├── junto2.m                  ← Main MATLAB program
├── mc_bridge.py              ← Python-Robot bridge
├── cameraParams.mat          ← Camera intrinsics (calibration)
└── trainedNet.mat            ← Pre-trained AI model
```

---

## 🔗 Dependencies Summary

| Software | Version | Purpose |
|----------|---------|---------|
| MATLAB | Latest | Main control loop |
| Python | 3.11 | Robot communication |
| pymycobot | Latest | MyCobot library |
| Image Processing Toolbox | Latest | Camera processing |
| Computer Vision Toolbox | Latest | Camera calibration |
| Neural Network Toolbox | Latest | AI model execution |

---

## 📖 Key System Variables

```matlab
% Vision
Xc, Yc                % Camera-detected target (mm)
Xpred2, Ypred2        % AI-predicted robot coordinates (mm)
centroid              % Pixel position of target in image

% Control
XR, YR                % Current XY platform position (mm)
D                     % Distance to target (mm)
e                     % Error vector (mm)

% Telemetry
rmse_x, rmse_y, rmse_z    % Root mean square errors (mm)
rmse_3d                   % Combined 3D error (mm)
r_log                     % Robot position history
ref_log                   % Desired position history
```

---

## 💡 Tips for Best Results

1. **Lighting:** Use consistent, bright lighting (no shadows on target)
2. **Calibration:** Don't move camera after calibration
3. **Tuning:** Start with conservative Kp (0.3), increase gradually
4. **Workspace:** Keep target within the 260mm safety radius
5. **Speed:** Lower speeds (30-50) are more accurate than high speeds
6. **Momentum:** The XY platform has inertia—allow time for settling

---

## 📞 Support

For issues with:
- **Camera calibration:** See MATLAB Documentation on `calibrateCameraParameters`
- **MyCobot control:** Check [pymycobot documentation](https://github.com/elephantrobotics/pymycobot)
- **GRBL/Marlin commands:** Refer to their official documentation

---

**Last Updated:** 2026  
**System:** MyCobot 280 M5 + XY Cartesian Platform  
**Language:** MATLAB + Python 3.11
