# UR5e Hand Teleoperation

Control a virtual UR5e robot arm using hand movements captured through a laptop webcam. The robot mimics the hand's movement in 3D and is rendered live in RViz2.

## Demo
<p align = "center">
  <img source = "assets/demo.gif" alt = "UR5e Hand Teleoperation">
</p>
## Implementation

The system uses MediaPipe Hands and OpenCV to track hand landmarks from the webcam. Hand position is mapped to the robot workspace, with hand size used to estimate depth. A clutch mechanism uses an open hand to freeze the robot and a closed fist to engage control. Custom forward kinematics calculates the robot pose, while Damped Least Squares inverse kinematics converts the target position into joint angles. The system runs at 20 Hz using ROS 2 Humble and RViz2.

## Tech Stack

| Component | Technology |
|---|---|
| Middleware | ROS 2 Humble |
| Visualisation | RViz2 |
| Hand Tracking | MediaPipe Hands |
| Computer Vision | OpenCV |
| Kinematics | Custom NumPy solver |
| Robot Model | Universal Robots UR5e |
| Language | Python 3.10 |

## Project Structure

```text
gesture_ws/
├── src/gesture_control/
│   ├── gesture_control/
│   │   ├── kinematics.py
│   │   └── gesture_control_node.py
│   ├── launch/
│   │   └── gesture_control.launch.py
│   ├── rviz/
│   │   └── ur5e.rviz
│   └── urdf/
│       └── ur5e.urdf
└── meshes/
```

## Installation

### Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- Webcam

### Clone

```bash
git clone https://github.com/yourusername/gesture_ws.git
cd gesture_ws
```

### Dependencies

```bash
pip install --user "mediapipe==0.10.14" "numpy<2"
```

### Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select gesture_control --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch gesture_control gesture_control.launch.py
```

## Control

- Open hand: robot is frozen.
- Closed fist: robot follows hand movement.
- Open hand again: robot freezes at its current position.


---
