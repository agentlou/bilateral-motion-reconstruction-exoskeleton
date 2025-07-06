
# 🦿 Bilateral Motion Reconstruction for Lower-Limb Exoskeleton (MATLAB)

This project presents a bilateral motion reconstruction method for a lower-limb exoskeleton using MATLAB. It uses **Dynamic Time Warping (DTW)** for gait alignment, **linear and nonlinear symmetry analysis** for motion validation, and **adaptive fuzzy logic control** to generate assisted motion for an impaired limb. The robot model and simulation setup are organized using URDF and ROS-compatible configurations.

---

## 📁 Folder Structure

```
├── config/                # Configuration files for sensors and control parameters
├── exoskelton-project/    # Core MATLAB code: DTW, symmetry analysis, fuzzy logic
├── launch/                # ROS launch files for simulation or visualization
├── meshes/                # 3D mesh models for robot visualization
├── urdf/                  # URDF files for the exoskeleton structure
└── README.md              # Project documentation
```

---

## ⚙️ Features

- ✅ **Dynamic Time Warping (DTW)** for temporal gait alignment
- ⚖️ **Symmetry Analysis**
  - Linear: amplitude correlation, signal deviation
  - Nonlinear: entropy, recurrence quantification, phase-space similarity
- 🧠 **Adaptive Fuzzy Logic Control**
  - Intelligent control rules based on symmetry deviation
  - Adjustable behavior in response to gait changes
- 🤖 **Robot Integration**
  - URDF-based model
  - ROS/Simulink compatible for simulation and control testing

---

## 🚀 Getting Started

### Requirements

- MATLAB (R2021a or later)
- Fuzzy Logic Toolbox
- Signal Processing Toolbox
- ROS Toolbox (optional)
- Simscape Multibody (optional for simulation)

### Run the Project

1. Open MATLAB and navigate to `exoskelton-project/`
2. Run the main script:
```matlab
main
```
3. Modify parameters in the `config/` folder to test different gait scenarios

---

## 📊 Output

- Gait cycle alignment plots (before/after DTW)
- Reconstructed vs real limb trajectories
- Symmetry score visualizations
- Fuzzy logic surface plots and rule evaluations

---

## 🧪 Applications

- Robotic-assisted gait rehabilitation
- Adaptive exoskeleton control systems
- Post-stroke or injury motion compensation
- Bilateral motion mirroring systems

---



