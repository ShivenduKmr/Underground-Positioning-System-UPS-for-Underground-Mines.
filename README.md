# Underground Positioning System (UPS) for Mine Safety

## Project Overview
This project focuses on the development of an **Autonomous Mobile Mapping and Tracking UAV (AMM-T-UAV)** designed for underground mining environments. The system provides real-time 3D localization and safety monitoring in GPS-denied areas.

**Developed by:** Shivendu Kumar (NIT Patna)  
**Program:** CHANAKYA Fellowship Program 2025 (TEXMiN)

## Tech Stack
*   **Operating System:** Ubuntu 22.04 LTS
*   **Middleware:** ROS 2 Humble
*   **Simulation:** Gazebo / AirSim
*   **Vision Hardware:** Intel RealSense D435i
*   **Compute:** NVIDIA RTX 3050 (Dev) | NVIDIA Jetson Orin Nano (Target)

## Development Phases (Current)

### Phase 1: Digital Foundation & Simulation
*   Development of 3D mine tunnel environments in Gazebo.
*   Integration of ROS 2 Humble with virtual sensor plugins (IMU, Stereo Depth).
*   Testing Visual-Inertial SLAM (V-SLAM) algorithms in feature-poor simulated corridors.

### Phase 2: Algorithmic Development & Bench Testing
*   Implementation of EKF (Extended Kalman Filter) for sensor fusion.
*   Processing 3D Point Clouds to generate high-resolution spatial maps.
*   Validating positional drift rates in low-light bench environments.

"Algorithm development based on Mohta et al. (GRASP Lab) logic, implemented in ROS 2 Humble."

## How to Run (Work in Progress)
*(Instructions on how to clone and launch the ROS 2 workspace will be added as code is pushed)*
