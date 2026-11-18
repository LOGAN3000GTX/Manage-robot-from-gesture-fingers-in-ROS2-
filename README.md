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


## Code Explanation

The codebase is organized into two Python nodes inside `robot_vision_pkg`. Below is a breakdown of the core logic in each file.

---

### 1. Perception Node (`camera_gesture_node.py`)

This node captures video frames from the webcam, detects hand landmarks using MediaPipe, counts raised fingers, and publishes the count to `/finger_count`.

#### Initialization & ROS 2 Publisher
```python
self.gesture_pub = self.create_publisher(Int32, '/finger_count', 10)
self.cap = cv2.VideoCapture(0)
self.hands = self.mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.6,
    min_tracking_confidence=0.6
)
self.timer = self.create_timer(0.1, self.timer_callback)
```
* **`create_publisher`**: Sets up the `/finger_count` topic using standard integer messages (`std_msgs/Int32`) with a queue size of `10`.
* **`VideoCapture(0)`**: Opens the default system webcam.
* **`mp_hands.Hands(...)`**: Initializes MediaPipe to track a single hand with a 60% confidence threshold for stable real-time detection.
* **`create_timer(0.1, ...)`**: Executes `timer_callback` every `0.1` seconds (10 Hz).

#### Finger Counting Logic (`count_fingers`)
```python
tips = [8, 12, 16, 20]
count = 0
for tip in tips:
    if hand_landmarks.landmark[tip].y < hand_landmarks.landmark[tip - 2].y:
        count += 1

if abs(hand_landmarks.landmark[4].x - hand_landmarks.landmark[17].x) > \
   abs(hand_landmarks.landmark[3].x - hand_landmarks.landmark[17].x):
    count += 1
```
* **Four Main Fingers (`8, 12, 16, 20`)**: Compares the vertical `y`-coordinate of each fingertip against its middle joint (`tip - 2`). Because OpenCV's image origin `(0,0)` is at the top-left corner, a smaller `y` value means the finger is raised higher.
* **Thumb (`4`)**: Because the thumb moves horizontally rather than vertically, the code measures the horizontal (`x`-axis) distance between the thumb tip (`4`) and the base of the pinky (`17`), comparing it against the thumb's inner joint (`3`).

#### Frame Processing & Visual Debugging (`timer_callback`)
```python
frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
results = self.hands.process(frame_rgb)
```
* **Color Conversion**: OpenCV captures images in **BGR** format, while MediaPipe requires **RGB**, so `cvtColor` converts the frame before inference.
* **Skeleton Overlay**: When a hand is detected, `mp_draw.draw_landmarks(...)` draws the 21 joint nodes and connecting lines onto the BGR `frame`, `cv2.putText` prints the finger count in the corner, and `self.gesture_pub.publish(finger_msg)` sends the integer to Node 2.

---

### 2. Control Node (`robot_controller_node.py`)

This node listens to `/finger_count` and converts the detected finger count into velocity commands (`TwistStamped`) for the Gazebo robot.

#### Subscriber & Velocity Publisher
```python
self.subscription = self.create_subscription(
    Int32,
    '/finger_count',
    self.gesture_callback,
    10
)
self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
```
* **`create_subscription`**: Triggers `gesture_callback` every time a new integer arrives on `/finger_count`.
* **`cmd_pub`**: Publishes `geometry_msgs/TwistStamped` messages to `/cmd_vel`, which drives the differential-drive plugin in Gazebo.

#### Inverted Kinematic Mapping (`gesture_callback`)
```python
twist_msg = TwistStamped()
twist_msg.header.stamp = self.get_clock().now().to_msg()

if fingers == 1:
    twist_msg.twist.linear.x = -0.22  # Backward
elif fingers == 2:
    twist_msg.twist.linear.x = 0.22   # Forward
elif fingers == 3:
    twist_msg.twist.angular.z = -0.5  # Turn Right
elif fingers == 4:
    twist_msg.twist.angular.z = 0.5   # Turn Left
elif fingers == 5:
    twist_msg.twist.linear.x = 0.65   # Turbo Forward
else:
    twist_msg.twist.linear.x = 0.0    # Stop
    twist_msg.twist.angular.z = 0.0

self.cmd_pub.publish(twist_msg)
```
* **Timestamping**: `self.get_clock().now().to_msg()` attaches the current ROS 2 simulation/system timestamp to the message header.
* **Linear Velocity (`linear.x`)**: Controls forward (`+`) and backward (`-`) movement in meters per second ($\text{m/s}$). Here, `1` finger is inverted to `-0.22 m/s` (reverse) and `2` fingers to `+0.22 m/s` (forward).
* **Angular Velocity (`angular.z`)**: Controls rotation around the vertical Z-axis in radians per second ($\text{rad/s}$), where negative values rotate clockwise (right) and positive values rotate counter-clockwise (left).
* **Failsafe Stop (`else`)**: If `0` fingers are shown (a closed fist) or no hand is visible in the frame, both velocities are set to `0.0` to immediately halt the robot.
