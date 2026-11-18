# Manage your robot from gesture fingers in ROS2 simulation
A decoupled two-node ROS 2 system for controlling a mobile robot in Gazebo using real-time webcam hand gesture recognition via OpenCV and MediaPipe.
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

     
