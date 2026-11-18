# Manage your robot from gesture fingers in ROS2 simulation
A decoupled two-node ROS 2 system for controlling a mobile robot in Gazebo using real-time webcam hand gesture recognition via OpenCV and MediaPipe.

## 1. Abstract

This report documents the design, implementation, and verification of a vision-based teleoperation system for a mobile robot simulated in Gazebo using ROS 2 (Jazzy). To achieve modularity and separation of concerns, the control pipeline is decoupled into two independent ROS 2 nodes within the robot_vision_pkg package: a perception node (camera_gesture_node) responsible for real-time computer vision and hand landmark extraction, and an actuation control node (robot_controller_node) responsible for translating discrete gesture states into stamped velocity commands (/cmd_vel) with custom inverted directional mapping.


## System Architecture
The project is split into two independent ROS 2 nodes (`robot_vision_pkg`) communicating over a custom topic pipeline:

1. **`camera_gesture_node` (`camera_node`)**
   - Captures live video frames from the webcam using OpenCV.
   - Detects hand landmarks and overlays the hand skeleton (`HAND_CONNECTIONS`) for visual debugging.
   - Counts raised fingers (including X-axis thumb detection) and publishes the integer count (`std_msgs/Int32`) to `/finger_count`.

2. **`robot_controller_node` (`controller_node`)**
   - Subscribes to `/finger_count`.
   - Maps the detected finger count into velocity commands (`geometry_msgs/TwistStamped`) using custom inverted movement logic.
   - Publishes velocity commands to `/cmd_vel` to drive the mobile robot in Gazebo.
  ```text
[Webcam] ──> [camera_gesture_node] ──(/finger_count)──> [robot_controller_node] ──(/cmd_vel)──> [Gazebo Robot]

### Node Breakdown
1. **Perception Node (`camera_gesture_node` / executable: `camera_node`)**
   - **Responsibilities:** Captures live video frames via OpenCV (`cv2.VideoCapture(0)`), processes RGB frames through the **MediaPipe Hands** pipeline, overlays skeletal landmarks for visual debugging, executes geometric finger-counting logic, and publishes the resulting integer state at **10 Hz** (`0.1s` timer interval).
   - **Published Topic:** `/finger_count` (`std_msgs/msg/Int32`)

2. **Control Node (`robot_controller_node` / executable: `controller_node`)**
   - **Responsibilities:** Subscribes to `/finger_count`, evaluates the incoming integer command against a predefined kinematic lookup table, constructs timestamped velocity messages, and dispatches them to the Gazebo differential-drive controller.
   - **Subscribed Topic:** `/finger_count` (`std_msgs/msg/Int32`)
   - **Published Topic:** `/cmd_vel` (`geometry_msgs/msg/TwistStamped`)

---

## 3. Computer Vision & Gesture Recognition Methodology

Hand tracking and feature extraction are implemented using **MediaPipe Hands** configured for single-hand tracking (`max_num_hands=1`, `min_detection_confidence=0.6`, `min_tracking_confidence=0.6`).

### 3.1 Geometric Finger Counting Algorithm
Once 21 three-dimensional hand landmarks are extracted, the system determines the number of extended fingers (`0` to `5`) using axis-specific spatial comparisons:
- **Index, Middle, Ring, and Pinky Fingers (Landmarks `8, 12, 16, 20`):** Evaluated along the vertical image axis ($Y$-axis). A finger is classified as extended when its fingertip landmark $y$-coordinate is strictly less than that of its proximal interphalangeal (PIP) joint (`tip - 2`), accounting for OpenCV's top-left origin coordinate system:
  $$y_{\text{tip}} < y_{\text{PIP}}$$
- **Thumb (Landmark `4`):** Evaluated along the horizontal image axis ($X$-axis) to account for lateral abduction rather than vertical extension. The horizontal distance from the thumb tip (`4`) to the base of the pinky (`17`) is compared against the thumb interphalangeal joint (`3`):
  $$|x_4 - x_{17}| > |x_3 - x_{17}|$$

### 3.2 Real-Time Visual Debugging
To verify landmark tracking accuracy during runtime, `mp.solutions.drawing_utils.draw_landmarks` renders all 21 joint nodes and `HAND_CONNECTIONS` directly onto the BGR OpenCV frame buffer prior to calling `cv2.imshow()`, accompanied by an on-screen overlay displaying the active detected finger count.

---

## 4. Kinematic Control & Inverted Velocity Mapping

The `robot_controller_node` implements a custom inverted longitudinal mapping (where `1` finger commands reverse motion and `2` fingers command forward motion) alongside rotational and high-speed states. Each callback populates a `geometry_msgs/msg/TwistStamped` message stamped with the current ROS 2 node clock (`self.get_clock().now().to_msg()`).

| Gesture Input (`/finger_count`) | Control State | Linear Velocity $v_x$ ($\text{m/s}$) | Angular Velocity $\omega_z$ ($\text{rad/s}$) | Gazebo Robot Behavior |
| :---: | :--- | :---: | :---: | :--- |
| **`0`** *(Fist / No hand)* | Idle / Halt | `0.00` | `0.00` | Complete stop |
| **`1`** *(One finger)* | Reverse | `-0.22` | `0.00` | Moves backward along $-X$ |
| **`2`** *(Two fingers)* | Drive | `+0.22` | `0.00` | Moves forward along $+X$ |
| **`3`** *(Three fingers)* | Yaw Clockwise | `0.00` | `-0.50` | Rotates right in place |
| **`4`** *(Four fingers)* | Yaw Counter-Clockwise | `0.00` | `+0.50` | Rotates left in place |
| **`5`** *(Open palm)* | High-Speed Drive | `+0.65` | `0.00` | Turbo forward acceleration |

---

## 5. Package Structure & Dependencies

### Workspace Layout
```text
~/Desktop/ros_ws/
└── src/
    └── robot_vision_pkg/
        ├── package.xml
        ├── setup.py
        └── robot_vision_pkg/
            ├── __init__.py
            ├── camera_gesture_node.py   # Node 1: Vision & landmark publisher
            └── robot_controller_node.py # Node 2: Kinematic TwistStamped controller
```

### Software Environment
- **Middleware:** ROS 2 Jazzy Jalisco
- **Simulator:** Gazebo
- **Language:** Python 3
- **Libraries:** `rclpy`, `std_msgs`, `geometry_msgs`, `opencv-python` (`cv2`), `mediapipe`

---

## 6. Build, Deployment & Verification

### 6.1 Package Compilation
The package is built using `colcon` with `--symlink-install` so Python source modifications propagate immediately without full recompilation:
```bash
cd ~/Desktop/ros_ws
colcon build --packages-select robot_vision_pkg --symlink-install
source install/setup.bash
```

### 6.2 Execution Procedure
1. **Start the Simulation Environment:** Launch the Gazebo world containing the differential-drive mobile robot subscribed to `/cmd_vel`.
2. **Launch Node 1 (Perception):**
   ```bash
   source ~/Desktop/ros_ws/install/setup.bash
   ros2 run robot_vision_pkg camera_node
   ```
3. **Launch Node 2 (Controller):**
   ```bash
   source ~/Desktop/ros_ws/install/setup.bash
   ros2 run robot_vision_pkg controller_node
   ```

### 6.3 System Verification
- **Visual Feedback:** The OpenCV window (`Node 1: Camera View`) displays real-time skeletal tracking dots and updates `Fingers detected: N` accurately for counts `0` through `5`.
- **Topic & Graph Inspection:** Running `rqt_graph` or `ros2 topic echo /cmd_vel` confirms that `/camera_gesture_node` publishes to `/finger_count`, `/robot_controller_node` translates those integers into the corresponding `linear.x` and `angular.z` velocities, and the Gazebo robot responds immediately with inverted longitudinal control.
