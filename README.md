# horizon-mapless-nav
task
# Horizon Mapless Navigation System

[cite_start]This repository houses the complete autonomous, mapless navigation stack for the Team Horizon Rover[cite: 2, 14]. [cite_start]The system architecture is split between high-level autonomous processing nodes running on an NVIDIA Jetson Nano and low-level physical control firmware executing on an Arduino Due[cite: 17, 37, 38]. 

[cite_start]Unlike traditional map-based or GPS-reliant frameworks, this pipeline relies on real-time sensor streams to actively perceive, represent, and safely traverse unknown, dynamic, and uneven terrains[cite: 13, 15, 24].

---

## 📂 Repository Directory Layout

```text
horizon-mapless-nav/
├── LICENSE
├── README.md                      # General system overview and folder guide
├── high_level_ros2/               # ROS2 workspace running on the NVIDIA Jetson Nano CPU/GPU
│   └── src/
│       ├── perception/            # Module 1: Raw sensor ingestion & processing
│       ├── representation/        # Module 2: Goal-guided Transformer (GoT) core
│       ├── decision/              # Module 3: State machines & behavior planning
│       └── hardware_bridge/       # ROS2-to-Arduino serial messaging interface
└── low_level_firmware/            # Bare-metal C/C++ running on the Arduino Due
    └── rover_control/
        ├── rover_control.ino      # Main micro-controller entry loop & hardware timers
        ├── pid.cpp / pid.h        # Closed-loop PID velocity tracking algorithms
        └── kinematics.cpp / .h    # Locomotion geometry and wheel speed translation