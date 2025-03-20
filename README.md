# Optimization-Free Quadrupedal Locomotion

This repository contains MATLAB code and documentation for an **optimization-free quadrupedal locomotion** controller, building upon the paper “Optimization-free Ground Contact Force Constraint Satisfaction in Quadrupedal Locomotion” [1]. Our version incorporates a more realistic Reduced Order Model (ROM) of the Husky Carbon quadruped by adding additional joints (knees, ankles), demonstrating improved performance and closer sim-to-real dynamics.

## Table of Contents
- [Overview](#overview)
- [Setup and Installation](#setup-and-installation)
- [Results](#results)

---

## Overview
Traditional legged locomotion controllers often rely on either optimization-based or learning-based approaches. While effective, these methods can be resource-intensive or require large datasets. This project studies and implements an **Explicit Reference Governor (ERG)** scheme that enforces ground force constraints without optimization or deep reinforcement learning. Key highlights:
1. **Reduced Order Model (ROM) of Husky Carbon**: We have extended the simple “stick-leg” ROM to include more realistic knees, ankles, and the correct segment lengths.
2. **Ground Reaction Force (GRF) Modeling**: Uses a compliant ground model with friction constraints to prevent slipping.
3. **Explicit Reference Governor (ERG)**: Dynamically adjusts the reference commands to satisfy friction, no-slip, and ground-contact constraints.
4. **Simulation**: Employs a Runge-Kutta (RK4) integrator for robust state updates at each timestep.

---

## Dependencies
- **MATLAB** (R2021b or later recommended, though many versions should still work)
- The following MATLAB Toolboxes (as needed):
  - Symbolic Math Toolbox  
  - Control System Toolbox  
  - Curve Fitting Toolbox (if you use it for friction or trajectory fitting)  
  - (Optional) Other toolboxes for 3D visualization or advanced features

Ensure these are installed to avoid errors.

---

## Setup and Installation

1. **Clone or download** this repository:
   ```bash
   git clone https://github.com/YourGitHubUsername/quadruped-locomotion-ERG.git

2. **Open MATLAB** and set the current folder to the cloned directory:
   ```matlab
   cd('path_to_repo\quadruped-locomotion-ERG');


## Results
- Improved Stability: Including multiple joints (knee, ankle) reduces unnatural body pitching/rolling seen in simpler stick-leg models.
- Friction Constraint Satisfaction: ERG ensures no foot slippage by capping ground forces within friction limits.
- Straight-line Walking: The controller successfully propels the quadruped forward without tipping or drifting sideways.  