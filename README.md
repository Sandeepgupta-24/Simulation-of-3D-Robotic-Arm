# 🤖 Simulation of 3D Robotic Arm

<div align="center">

![MATLAB](https://img.shields.io/badge/MATLAB-R2020a+-orange?style=for-the-badge&logo=mathworks&logoColor=white)
![Fusion 360](https://img.shields.io/badge/Fusion_360-CAD_Modeling-blue?style=for-the-badge&logo=autodesk&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)

**A comprehensive 3-DOF robotic arm motion simulation pipeline with forward kinematics using DH parameters, geometric workspace boundary analysis, and smooth 3D animation — built in MATLAB.**

*Association of Mechanical Engineers, IIT Kanpur — Dec 2024 – Jan 2025*

</div>

---

## 🎯 Objective

Architect and systematically simulate a comprehensive **3-DOF robotic arm** motion pipeline to exhaustively parse complex **spatial kinematics** and tightly constrained real-world operational workspaces.

---

## 🔧 Approach

- Simulated complete baseline **forward kinematics** alongside spatial bounds mapping of a **3-DOF robotic arm** utilizing established **DH parameters** within a native **MATLAB** environment
- Constructed and heavily analyzed functional structural limits deploying **Fusion 360**, meticulously modeling all **2D/3D prismatic geometric translations** and dynamic revolute joint mechanical articulations

---

## 📊 Result

- Successfully executed a fully validated, **smoothly animated** robotic arm structural motion simulation modeled natively in 3D, comprehensively backed by exhaustive **geometric workspace boundary analytics**

---

## 📁 Project Structure

```
Simulation-of-3D-Robotic-Arm/
│
├── robotic_arm_3d_simulation.m   # MATLAB simulation script
├── Final PPT.pdf                 # Project documentation
├── README.md                     
├── LICENSE                       
└── .gitignore                    
```

---

## ⚙️ Prerequisites

- **MATLAB R2020a** or later (no additional toolboxes needed)
- **Fusion 360** (for CAD modeling — separate structural analysis)

---

## 🚀 Getting Started

```bash
git clone https://github.com/Sandeepgupta-24/Simulation-of-3D-Robotic-Arm.git
cd Simulation-of-3D-Robotic-Arm
```

Open MATLAB → Navigate to project folder → Run:

```matlab
robotic_arm_3d_simulation
```

---

## 📐 DH Parameters

| Link | θ (Joint Angle) | d (Offset) | a (Length) | α (Twist) |
|------|:----------------:|:----------:|:----------:|:---------:|
| 1    | θ₁ (variable)   | 1.0 m      | 0          | 90°       |
| 2    | θ₂ (variable)   | 0          | 0.8 m      | 0°        |
| 3    | θ₃ (variable)   | 0          | 0.6 m      | 0°        |

---

## 🔬 What the Simulation Does

### 1. Forward Kinematics
Computes end-effector position from joint angles using **4×4 homogeneous transformation matrices** based on the DH convention:

```
T₀₃ = T₀₁ × T₁₂ × T₂₃
```

### 2. Workspace Boundary Analysis
Sweeps through all joint angle limits to compute every reachable end-effector position, producing a **3D geometric workspace envelope**.

### 3. Smooth 3D Animation
Generates smooth sinusoidal joint trajectories and animates the robotic arm in real-time with **end-effector path tracing**.

---

## 👤 Author

**Sandeep Gupta**  
GitHub: [@Sandeepgupta-24](https://github.com/Sandeepgupta-24)

---

## 📄 License

This project is licensed under the **MIT License** — see the [LICENSE](LICENSE) file for details.
