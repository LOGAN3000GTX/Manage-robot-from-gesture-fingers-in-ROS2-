#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import TwistStamped
import requests

ESP32_IP = ""


class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')
        # Подписываемся на число пальцев от первого узла
        self.subscription = self.create_subscription(
            Int32,
            '/finger_count',
            self.gesture_callback,
            10
        )
        # Публикуем скорости в Gazebo
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.last_cmd = None
        self.get_logger().info("Node 2 (Robot Controller - Inverted) started.")

    def send_esp32(self, endpoint):
        if self.last_cmd == endpoint:
            return
        try:
            requests.get(f"http://{ESP32_IP}/{endpoint}", timeout=0.15)
            self.last_cmd = endpoint
        except requests.RequestException:
            pass

    def gesture_callback(self, msg_in):
        fingers = msg_in.data
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = ''

        if fingers == 1:
            twist_msg.twist.linear.x = -0.22
            twist_msg.twist.angular.z = 0.0
            esp_cmd = "backward"
            action = "Backward (1 finger)"

        elif fingers == 2:
            twist_msg.twist.linear.x = 0.22
            twist_msg.twist.angular.z = 0.0
            esp_cmd = "forward"
            action = "Forward (2 fingers)"

        elif fingers == 3:
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.angular.z = -0.5
            esp_cmd = "turn_right"
            action = "Turn Right (3 fingers)"

        elif fingers == 4:
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.angular.z = 0.5
            esp_cmd = "turn_left"
            action = "Turn Left (4 fingers)"

        elif fingers == 5:
            twist_msg.twist.linear.x = 0.65
            twist_msg.twist.angular.z = 0.0
            esp_cmd = "forward"
            action = "Super Acceleration (5 fingers)"

        else:
            # 0 пальцев = СТОП
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.angular.z = 0.0
            esp_cmd = "stop"
            action = "Stop"

        self.cmd_pub.publish(twist_msg)
        self.send_esp32(esp_cmd)
        self.get_logger().info(f"Action: {action}", throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()