Underground Positioning System (UPS) for Subterranean Mine Safety
Chanakya Fellowship Program 2025 | TEXMiN Foundation

![alt text](https://img.shields.io/badge/ROS2-Humble-blue)
![alt text](https://img.shields.io/badge/Gazebo-Fortress-orange)
![alt text](https://img.shields.io/badge/Python-3.10-green)
![alt text](https://img.shields.io/badge/Affiliation-NIT%20Patna-red)

📌 Project Overview

This repository contains the development and validation of an Autonomous Mobile Mapping and Tracking UAV (AMM-T-UAV) specifically designed for deep-mine environments.

In underground mining, the absence of Global Navigation Satellite System (GNSS) signals and the prohibitive cost of fixed infrastructure create a significant "Data Gap." Our project bridges this gap by providing an infrastructure-less Underground Positioning System (UPS) capable of real-time 3D mapping and personnel tracing.

🚀 Key Achievements (Month 1 - Progress Report 1)

Sub-Centimeter Localization: Achieved a maximum transient error of 7.1mm and an 82.4% reduction in positional noise using a custom UKF engine.

3D Digital Twin Generation: Real-time construction of a persistent 3D mine map using a 16-ring high-fidelity LiDAR.

Autonomous Mission Cycle: Fully automated Takeoff -> Explore -> Map -> Return-to-Home (RTH) -> Landing sequence.

Reactive Obstacle Avoidance: Integration of Artificial Potential Fields (APF) to autonomously bypass structural pillars while maintaining high-speed flight.

🧠 Algorithmic Framework
1. Unscented Kalman Filter (UKF)

To handle the extreme non-linearities of drone flight in tight shafts, the algorithm utilizes the Unscented Transform. It propagates deterministically chosen Sigma Points through the drone's actual physics equations, allowing it to maintain a positioning "lock" even during aggressive evasive maneuvers or low-light conditions.

2. Reactive Potential Fields (APF)

The navigation logic treats the drone as a particle in a force field:

Attractive Force: Pulls the drone toward the goal (miner location/tunnel entrance).

Repulsive Force: Pushes the drone away from walls and debris detected by the LiDAR.

Rerouting: The drone automatically calculates a "Vortex Force" to curve around obstacles without stopping or requiring human intervention.

📊 Experimental Results
Mission Profile: 20m Round-Trip Gallery Scan
Metric	Achievement
Max Localization Error	0.0071 meters
Steady-State Error	0.0000 meters
Cruise Altitude	1.40 - 1.50 meters
Positional Convergence	Zero-Error at Landing
Safety Clearance	0.5m minimum buffer from walls
Real-Time 3D Mapping (RViz2)

The system generates a persistent 3D point cloud of the mine environment. By setting a Decay Time of 1000s and utilizing AxisColor transformation, we create a high-resolution "Rainbow" map depicting the structural integrity of the mine tunnel.

(Place for Gazebo Simulation Screenshot)
(Place for RViz 3D Map Screenshot)

🛠️ Tech Stack

OS: Ubuntu 22.04 LTS

Middleware: ROS 2 Humble

Simulator: Gazebo Sim (Fortress)

Compute Target: NVIDIA Jetson Orin Nano / Holybro X500 V2

Sensors: 16-Ring 3D LiDAR, RGB-D Camera, high-frequency IMU.

📂 Repository Structure
code
Text
download
content_copy
expand_less
├── ups_simulation/       # World (.sdf) and Model files
│   ├── worlds/           # Hollow Cylindrical Mine Gallery
│   └── models/           # Custom X500 UPS Drone with 3D LiDAR/Camera
├── ups_logic/            # ROS 2 Python Nodes
│   └── navigator.py      # UKF Engine, APF Logic, and Mission State Machine
├── docs/                 # LaTeX Reports and Mathematical Foundations
└── scripts/              # Python benchmarking & Result plotting
📜 References

This project is built upon a synthesis of world-class research:

Mohta et al. (2018): Visual-Inertial state estimation and Minimum Snap trajectories.

Liu et al. (2017): Safe Flight Corridors and convex decomposition of space.

Jiang et al. (2017): Real-time monitoring of underground miners using IoT.

Vijay Kumar Lab: Logic of Differential Flatness for real-time trajectory planning.

Lead Researcher: Shivendu Kumar
Mentor: Dr. Golak Bihari Mahanta
Affiliation: Mechanical Engineering, National Institute of Technology (NIT), Patna
Strategic Support: National Mission on Interdisciplinary Cyber-Physical Systems (NM-ICPS)
