# 🤖 Robot Patrol - ROS2 Galactic

Security patrol robot system built with ROS2 Galactic running on Jetson AGX Xavier.

## Subsystems
- **Ultrasonic** - Obstacle detection
- **Lidar** - 360° environment scanning
- **Vision** - Object detection (YOLO)
- **Navigation** - Autonomous patrol
- **LoRa** - Long range communication
- **Failsafe** - Emergency stop system

## Requirements
- Ubuntu 20.04
- ROS2 Galactic
- Jetson AGX Xavier

## Run
```bash
ros2 launch robot_patrol patrol_launch.py
```
