# 🤖 Robot Patrol - ROS2 Galactic

Security patrol robot system built with ROS2 Galactic running on Jetson AGX Xavier.

## Subsystems
- **Ultrasonic** - Obstacle detection
- **Lidar** - 360° environment scanning
- **Vision** - Object detection (YOLO V11 small)
- **Navigation** - Autonomous patrol
- **LoRa** - Long range communication
- **Failsafe** - Emergency stop system

## Requirements
- Ubuntu 20.04
- ROS2 Galactic
- Jetson AGX Xavier Developer Kit
- Python 3.8+

## Installation
```bash
# Clone repo
git clone https://github.com/securitypatrolrobot-ctrl/robot_patrol.git
cd robot_patrol

# Build
colcon build --packages-select robot_patrol
source install/setup.bash
```

## Run
```bash
ros2 launch robot_patrol patrol_launch.py
```

## Team
- Sistem Integrator: ROS2 Galactic + Jetson AGX Xavier
