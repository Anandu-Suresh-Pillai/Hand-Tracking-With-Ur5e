# 🤖 UR5e Hand Gesture Control — ROS 2 Simulation

> Control a robot arm in real-time using just your hand and a webcam. No special hardware needed.

![Demo](assets/output.gif)

---

## What is this?

This is a university project that lets you **control a virtual UR5e robot arm** by moving your hand in front of your laptop webcam. The robot mimics your hand movements in 3D, rendered live inside RViz2 (a ROS 2 visualisation tool).

The idea came from wanting to explore human-robot interaction without needing an actual physical robot — everything runs as a software simulation on a normal PC.

---

## How it works (the simple version)

1. Your webcam captures your hand using [MediaPipe](https://mediapipe.dev/) — a Google library that detects hand landmarks in real time.
2. The software figures out where your hand is in 3D space (left/right, up/down, and how far from the camera).
3. Those coordinates get converted into joint angles for the robot using a math technique called **Inverse Kinematics**.
4. The robot arm in RViz2 updates 20 times per second to match your hand position.

### The "Clutch" (the most important concept)

Think of it like lifting a mouse off a mousepad:

- ✋ **Open hand** → Robot is **frozen**. You can move your arm around freely without affecting the robot.
- ✊ **Close fist** → Robot **follows** your fist. The moment you close your fist, your current position becomes the "zero point" and any movement from there moves the robot.
- Open your fist again → Robot freezes at its new position. Repeat.

This lets you comfortably reposition your arm without accidentally moving the robot.

### 3D Tracking from a 2D Camera

Since a normal webcam has no depth sensor, depth is estimated by **how big your hand appears** on screen:
- Hand looks bigger (closer to camera) → robot reaches forward
- Hand looks smaller (further away) → robot pulls back

---

## Demo

![Robot arm following hand movements](assets/output.gif)

---

## Tech Stack

| What | Tool |
|------|------|
| Robot middleware | ROS 2 Humble |
| 3D Visualisation | RViz2 |
| Hand Tracking | MediaPipe Hands |
| Computer Vision | OpenCV |
| Kinematics math | Custom NumPy solver |
| Robot model | Universal Robots UR5e (URDF + meshes) |
| Language | Python 3.10 |

---

## Project Structure

```
gesture_ws/
├── assets/
│   └── output.gif              ← demo recording
├── src/gesture_control/
│   ├── gesture_control/
│   │   ├── kinematics.py       ← forward & inverse kinematics engine
│   │   └── gesture_control_node.py  ← main ROS 2 node (hand tracking + control)
│   ├── launch/
│   │   └── gesture_control.launch.py
│   ├── rviz/
│   │   └── ur5e.rviz           ← pre-configured RViz2 layout
│   └── urdf/
│       └── ur5e.urdf           ← robot description file
└── meshes/                     ← 3D model files for the robot
```

---

## Setup & Installation

### Prerequisites

- Ubuntu 22.04
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) installed
- Python 3.10
- A webcam

### 1. Clone the repo

```bash
git clone https://github.com/yourusername/gesture_ws.git
cd gesture_ws
```

### 2. Install Python dependencies

```bash
pip install --user "mediapipe==0.10.14" "numpy<2"
```

### 3. Build the ROS 2 package

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select gesture_control --symlink-install
source install/setup.bash
```

### 4. Launch everything

```bash
ros2 launch gesture_control gesture_control.launch.py
```

This opens:
- A **webcam window** with a live overlay showing your hand tracking and robot state
- **RViz2** with the UR5e robot rendered in 3D

---

## How to Use

1. Sit in front of your webcam.
2. Rest your **elbow on the desk** (acts as a natural pivot point — makes control easier).
3. Hold your **hand open** — you'll see `RELEASED` on screen. The robot stays still.
4. **Close your fist** — screen shows `ENGAGED` in green. Now move your fist to control the robot.
5. **Open your fist** to freeze the robot at its current position.
6. Reposition your arm and repeat from step 3.

### The webcam overlay explained

![HUD diagram — top-left shows clutch state and coordinates, bottom shows joint bars](assets/output.gif)

| Element | What it means |
|---------|--------------|
| 🟢 `ENGAGED` | Your fist is closed — robot is following |
| 🟠 `RELEASED` | Hand is open — robot is frozen |
| `X / Y / Z` values | Target position of the robot's hand in metres |
| Bottom bars (J1–J6) | How much each of the 6 joints has rotated |
| Yellow crosshair | The point where you closed your fist (anchor) |
| Cyan arrow | Direction your hand has moved since engaging |
| 🔴 `WORKSPACE LIMIT` | Robot has reached its max reach — it's clamped |

---

## The Math (brief, I promise)

### Forward Kinematics
To know *where* the robot's hand currently is, the code chains together 6 rotation matrices — one for each joint — based on the exact physical dimensions from the URDF file.

### Inverse Kinematics (the hard part)
Going the other way — "given a target position, what angles should the joints be?" — is much harder. This uses a method called **Damped Least Squares**, which:
1. Calculates how moving each joint a tiny bit would move the robot's hand (the "Jacobian").
2. Iteratively adjusts all 6 joints together to reduce the distance to the target.
3. The "damping" prevents the robot from making sudden jerky movements near tricky positions.

---

## Known Limitations

- Depth estimation (using hand size) is approximate — it works well but isn't as accurate as a real depth camera.
- Works best in good, even lighting.
- Only tracks one hand at a time.
- The robot's orientation (which way the wrist points) is not controlled — only position.

---

## Tuning

If the motion feels too slow or too fast, edit the constants at the top of [`gesture_control_node.py`](src/gesture_control/gesture_control/gesture_control_node.py):

```python
K_X = 2.50   # depth sensitivity
K_Y = 2.00   # left/right sensitivity
K_Z = 2.00   # up/down sensitivity
ALPHA = 0.45 # smoothing (lower = snappier, higher = smoother)
```

---

## Acknowledgements

- Robot model: [Universal Robots UR5e](https://www.universal-robots.com/)
- Hand tracking: [MediaPipe](https://mediapipe.dev/) by Google
- Built as part of a robotics simulation project using ROS 2 Humble

---
