import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout , QHBoxLayout
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, QTimer
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()
        self.image_latest_frame = None
        self.depth_latest_frame = None

        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        qos_profile = QoSProfile(depth=5)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        qos_profile.durability = DurabilityPolicy.VOLATILE

        self.image_subscription = self.create_subscription(
            Image,
            '/realsense/camera/image_raw',
            self.image_listener_callback,
            qos_profile
        )
        self.depth_subscription = self.create_subscription(
            Image,
            '/realsense/camera/depth/image_raw',
            self.depth_listener_callback,
            qos_profile,
        )
        
    def image_listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.image_latest_frame = frame
    
    def depth_listener_callback(self,msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depth_latest_frame = frame

class ImageViewer(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.setWindowTitle("ROS2 Camera Viewer - PyQt5")
        self.ros_node = ros_node

        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.depth_label = QLabel()
        self.depth_label.setAlignment(Qt.AlignCenter)

        left_layout = QVBoxLayout()
        left_layout.addWidget(self.image_label)

        right_layout = QVBoxLayout()
        right_layout.addWidget(self.depth_label)

        main_layout = QHBoxLayout()
        main_layout.addLayout(left_layout)
        main_layout.addLayout(right_layout)

        self.setLayout(main_layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_image)
        self.timer.start(30) 

    def update_image(self):
        if self.ros_node.image_latest_frame is not None:
            rgb_image = cv2.cvtColor(self.ros_node.image_latest_frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.image_label.setPixmap(QPixmap.fromImage(qt_image))

        if self.ros_node.depth_latest_frame is not None:
            depth_frame = self.ros_node.depth_latest_frame
            if depth_frame.dtype != 'uint8':
                depth_normalized = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX)
                depth_normalized = depth_normalized.astype('uint8')
            else:
                depth_normalized = depth_frame

            depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            depth_rgb = cv2.cvtColor(depth_colored, cv2.COLOR_BGR2RGB)

            h, w, ch = depth_rgb.shape
            bytes_per_line = ch * w
            qt_image = QImage(depth_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.depth_label.setPixmap(QPixmap.fromImage(qt_image))


def main(args=None):
    rclpy.init(args=args)
    ros_node = ImageSubscriber()

    app = QApplication(sys.argv)
    viewer = ImageViewer(ros_node)
    viewer.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.01))
    timer.start(10)

    sys.exit(app.exec_())

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
