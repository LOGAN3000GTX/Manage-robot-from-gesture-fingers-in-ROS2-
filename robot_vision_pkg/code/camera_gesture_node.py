#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import cv2
import mediapipe as mp


class CameraGestureNode(Node):
    def __init__(self):
        super().__init__('camera_gesture_node')
        # Публикуем только количество пальцев в свой топик finger_count
        self.gesture_pub = self.create_publisher(Int32, '/finger_count', 10)
        self.cap = cv2.VideoCapture(0)

        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.6
        )

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Node 1 (Camera Gesture) started.")

    def count_fingers(self, hand_landmarks):
        tips = [8, 12, 16, 20]
        count = 0
        for tip in tips:
            if hand_landmarks.landmark[tip].y < hand_landmarks.landmark[tip - 2].y:
                count += 1

        # Проверка большого пальца (4) относительно основания мизинца (17)
        if abs(hand_landmarks.landmark[4].x - hand_landmarks.landmark[17].x) > \
           abs(hand_landmarks.landmark[3].x - hand_landmarks.landmark[17].x):
            count += 1

        return count

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)

        finger_msg = Int32()
        finger_msg.data = 0  # 0 если рука не найдена или кулак

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Рисуем точки и линии скелета на кадре
                self.mp_draw.draw_landmarks(
                    frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS
                )
                finger_msg.data = self.count_fingers(hand_landmarks)

        # Отправляем число пальцев второму узлу
        self.gesture_pub.publish(finger_msg)

        cv2.putText(frame, f"Fingers detected: {finger_msg.data}", (30, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Node 1: Camera View", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraGestureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()