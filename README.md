# horizon-mapless-nav
task
# Horizon Mapless Navigation System


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
