# 🤖 Simulation of 3D Robotic Arm

<div align="center">

![MATLAB](https://img.shields.io/badge/MATLAB-R2020a+-orange?style=for-the-badge&logo=mathworks&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Complete-brightgreen?style=for-the-badge)
![Robotics](https://img.shields.io/badge/Domain-Robotics-blue?style=for-the-badge)

**A comprehensive MATLAB-based simulation of a 3-DOF (Degree of Freedom) Robotic Arm featuring forward & inverse kinematics, trajectory planning, Jacobian analysis, and interactive 3D visualization.**

</div>

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Features](#-features)
- [Project Structure](#-project-structure)
- [Prerequisites](#-prerequisites)
- [Getting Started](#-getting-started)
- [Modules](#-modules)
  - [Main Simulation](#1-main-simulation)
  - [Interactive GUI](#2-interactive-gui)
  - [Jacobian Analysis](#3-jacobian-analysis)
  - [Trajectory Planning](#4-trajectory-planning)
- [Theory & Background](#-theory--background)
- [Results](#-results)
- [Author](#-author)
- [License](#-license)

---

## 🔍 Overview

This project implements a complete simulation environment for a **3-DOF articulated robotic arm** in MATLAB. It uses the **Denavit-Hartenberg (DH) convention** for kinematic modeling and provides tools for:

- **Forward Kinematics** — Computing end-effector position from joint angles
- **Inverse Kinematics** — Finding joint angles for a desired end-effector position
- **Velocity Kinematics** — Jacobian-based velocity analysis
- **Trajectory Planning** — Smooth motion generation using polynomial interpolation
- **Workspace Analysis** — Computing and visualizing the reachable workspace envelope
- **Interactive Control** — Real-time GUI with sliders for joint angle manipulation

---

## ✨ Features

| Feature | Description |
|---------|-------------|
| 🎯 **Forward Kinematics** | DH parameter-based transformation matrices |
| 🔄 **Inverse Kinematics** | Geometric approach for 3-DOF arm |
| 📊 **Jacobian Analysis** | Velocity kinematics & manipulability ellipsoid |
| 📈 **Trajectory Planning** | Cubic & quintic polynomial interpolation |
| 🌐 **Workspace Visualization** | 3D scatter plot of reachable points |
| 🎮 **Interactive GUI** | Slider-based real-time joint control |
| 🎬 **Animation Engine** | Smooth trajectory animation with end-effector trace |
| 🎨 **Dark Theme Plots** | Professional dark-themed visualizations |

---

## 📁 Project Structure

```
Simulation-of-3D-Robotic-Arm/
│
├── robotic_arm_3d_simulation.m   # Main simulation script
├── robotic_arm_gui.m             # Interactive GUI controller
├── jacobian_analysis.m           # Jacobian & velocity kinematics
├── trajectory_planning.m         # Trajectory generation module
├── Matlab.pdf                    # Project presentation/documentation
├── README.md                     # This file
├── LICENSE                       # MIT License
└── .gitignore                    # MATLAB gitignore rules
```

---

## ⚙️ Prerequisites

- **MATLAB R2020a** or later
- No additional toolboxes required (pure MATLAB implementation)

---

## 🚀 Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/Sandeepgupta-24/Simulation-of-3D-Robotic-Arm.git
cd Simulation-of-3D-Robotic-Arm
```

### 2. Open in MATLAB

Open MATLAB and navigate to the project directory.

### 3. Run the Simulation

```matlab
% Run the main simulation (Forward Kinematics + Workspace + Animation)
robotic_arm_3d_simulation

% Launch the Interactive GUI
robotic_arm_gui

% Run Jacobian & Velocity Analysis
jacobian_analysis

% Run Trajectory Planning
trajectory_planning
```

---

## 📦 Modules

### 1. Main Simulation (`robotic_arm_3d_simulation.m`)

The core simulation script that demonstrates:

- **DH Parameter Setup** — Defines link lengths and joint parameters
- **Forward Kinematics** — Computes end-effector position using 4×4 homogeneous transformation matrices
- **3D Visualization** — Renders the robotic arm with color-coded joints, coordinate frames, and base platform
- **Workspace Analysis** — Generates a 3D scatter plot of all reachable end-effector positions
- **Trajectory Animation** — Animates the arm following sinusoidal joint trajectories with end-effector trace
- **Inverse Kinematics** — Solves for joint angles given a target position using geometric approach
- **Joint Angle Plots** — Time-series plots of joint trajectories

#### DH Parameters Table

| Link | θ (Joint Angle) | d (Offset) | a (Length) | α (Twist) |
|------|:----------------:|:----------:|:----------:|:---------:|
| 1    | θ₁ (variable)   | L₁ = 1.0m  | 0          | π/2       |
| 2    | θ₂ (variable)   | 0          | L₂ = 0.8m  | 0         |
| 3    | θ₃ (variable)   | 0          | L₃ = 0.6m  | 0         |

---

### 2. Interactive GUI (`robotic_arm_gui.m`)

A real-time interactive controller with:

- **Three slider controls** — One for each joint (θ₁, θ₂, θ₃)
- **Live 3D visualization** — Updates instantly as sliders move
- **End-effector position display** — Shows X, Y, Z coordinates in real-time
- **Reset to Home** — Returns all joints to zero position
- **Random Pose** — Generates a random arm configuration

---

### 3. Jacobian Analysis (`jacobian_analysis.m`)

Advanced velocity kinematics analysis:

- **Geometric Jacobian** — 3×3 matrix mapping joint velocities to end-effector velocities
- **Singularity Detection** — Checks determinant of Jacobian
- **Manipulability Index** — Yoshikawa's measure of dexterity
- **Manipulability Ellipsoid** — 3D visualization of velocity capability
- **Manipulability Map** — Heat map over joint space showing dexterous configurations
- **Condition Number** — Measure of kinematic conditioning

---

### 4. Trajectory Planning (`trajectory_planning.m`)

Smooth motion generation module:

- **Cubic Polynomial** — 3rd-order interpolation with zero velocity at endpoints
- **Quintic Polynomial** — 5th-order interpolation with zero velocity and acceleration at endpoints
- **Multi-segment Trajectories** — Via-point interpolation through multiple waypoints
- **Position, Velocity, Acceleration Profiles** — Complete motion analysis
- **Animated Execution** — Visualizes the robot following the planned trajectory

---

## 📐 Theory & Background

### Denavit-Hartenberg Convention

The DH convention provides a systematic method to attach coordinate frames to each link of a robot. Each transformation is described by four parameters:

```
T = Rot(z, θ) × Trans(z, d) × Trans(x, a) × Rot(x, α)
```

The homogeneous transformation matrix:

```
T = | cos(θ)  -sin(θ)cos(α)   sin(θ)sin(α)   a·cos(θ) |
    | sin(θ)   cos(θ)cos(α)  -cos(θ)sin(α)   a·sin(θ) |
    |   0        sin(α)          cos(α)           d      |
    |   0          0                0              1      |
```

### Forward Kinematics

The end-effector pose is obtained by chaining all individual transformations:

```
T₀₃ = T₀₁ × T₁₂ × T₂₃
```

### Inverse Kinematics (Geometric Approach)

For the 3-DOF arm, joint angles are computed using:

1. **θ₁** = atan2(y, x) — Base rotation
2. **θ₃** = acos((D² - L₂² - L₃²) / (2·L₂·L₃)) — Elbow angle (cosine rule)
3. **θ₂** = atan2(s, r) - atan2(L₃·sin(θ₃), L₂ + L₃·cos(θ₃)) — Shoulder angle

### Jacobian Matrix

The geometric Jacobian relates joint velocities to end-effector velocities:

```
v = J(q) · q̇
```

where each column is computed as:

```
Jᵢ = zᵢ₋₁ × (oₙ - oᵢ₋₁)    (for revolute joints)
```

---

## 📊 Results

The simulation produces the following outputs:

1. **3D Robot Visualization** — Color-coded arm with joint labels and coordinate frames
2. **Workspace Envelope** — 3D point cloud of all reachable positions
3. **Trajectory Animation** — Smooth animated motion with end-effector trace
4. **Joint Angle Plots** — Time-series analysis of joint trajectories
5. **Manipulability Ellipsoid** — Visual representation of velocity capability
6. **Manipulability Map** — Heat map showing dexterous configurations
7. **Trajectory Comparison** — Side-by-side cubic vs quintic polynomial profiles

---

## 👤 Author

**Sandeep Gupta**

- GitHub: [@Sandeepgupta-24](https://github.com/Sandeepgupta-24)

---

## 📄 License

This project is licensed under the **MIT License** — see the [LICENSE](LICENSE) file for details.

---

<div align="center">

**⭐ If you found this project helpful, please consider giving it a star! ⭐**

</div>
