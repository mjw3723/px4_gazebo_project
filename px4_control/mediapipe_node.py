#!/usr/bin/env python3
import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node

class MediapipeNode(Node):
    def __init__(self):
        super().__init__('mediapipe_node')
        self.cap = cv2.VideoCapture(0) 
        self.mp_hands = mp.solutions.hands.Hands()
        self.timer = self.create_timer(0.03, self.timer_callback)  

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        results = self.mp_hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        if results.multi_hand_landmarks:
            self.get_logger().info("손 인식 성공")

        cv2.imshow("Camera", frame)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC 종료
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MediapipeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
