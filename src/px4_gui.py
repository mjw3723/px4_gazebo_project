#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QThread, QObject, pyqtSignal , QTimer
from px4_msgs.msg import *
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2
from pypages.path3dpage import Path3DPage
from pypages.infopage import InfoPage
from pypages.controlpage import ControlPage
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import Image
import numpy as np

class SignalBus(QObject):
    info_text = pyqtSignal(str)
    battery_remaining = pyqtSignal(float)
    odom_xyz = pyqtSignal(float,float,float)
    odom_info = pyqtSignal(object)
    cloud_ready = pyqtSignal(object)

class RosSpinThread(QThread):
    def __init__(self, node: Node, parent=None):
        super().__init__(parent)
        self._node = node
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self._node)
        self._running = True

    def run(self):
        while self._running and rclpy.ok():
            try:
                self._exec.spin_once(timeout_sec=0.05)
            except Exception:
                pass

    def stop(self):
        self._running = False
        try:
            self._exec.remove_node(self._node)
        except Exception:
            pass

class Px4Node(Node):
    def __init__(self,bus: SignalBus):
        super().__init__('pyqt_px4_localpos_viewer')
        self.bus = bus
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        self.battery_sub = self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status_v1',
            self.battery_callback,
            qos
        )

        self.odom_xyz_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos
        )

        self.point_sub = self.create_subscription(
            PointCloud2,
            "/realsense/camera/points",
            self.point_callback,
            10
        )
        self.subscription = self.create_subscription(
            Image,
            '/realsense/camera/image_raw', 
            self.camera_callback,
            10
        )

        self.set_point_pub = self.create_publisher(TrajectorySetpoint, "/control/trajectory_setpoint", 10)
        self.point_count = 0

    def battery_callback(self,msg:BatteryStatus):
        battery_remaining = msg.remaining
        self.bus.battery_remaining.emit(battery_remaining)

    def odom_callback(self,msg:VehicleOdometry):
        odom_xyz = msg.position
        self.bus.odom_xyz.emit(odom_xyz[0],odom_xyz[1],odom_xyz[2])
        self.bus.odom_info.emit(msg)

    def publish_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        self.set_point_pub.publish(msg)
    
    def point_callback(self,msg:PointCloud2):
        if self.point_count < 20:
            self.point_count+=1
            return
        self.point_count = 0
        cloud_points = np.array([
            [p[0], p[1], p[2]]
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ], dtype=np.float32)
        self.bus.cloud_ready.emit(cloud_points)

    def camera_callback(self,msg:Image):
        self.bus.camera_ready.emit(msg)
        
    
class Px4Viewer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PX4 Viewer")
        self.resize(1280, 800)
        root = QHBoxLayout(self)

        left = QVBoxLayout()
        left.setSpacing(8)

        side = QFrame()
        side.setFrameShape(QFrame.StyledPanel)
        side.setLayout(left)

        btn_info  = QPushButton("px4 정보")
        btn_control = QPushButton("제어")
        btn_path = QPushButton("이동 경로") 
        for b in (btn_info,btn_control,btn_path):
            b.setCursor(Qt.PointingHandCursor)
            b.setMinimumHeight(40)
            left.addWidget(b)

        left.addStretch(1)  

        stack = QStackedWidget()

        self.info_page = InfoPage()
        self.page_path3d = Path3DPage()
        self.control_page = ControlPage()
        self.control_page.positionSet.connect(self.publish_position)
        stack.addWidget(self.info_page)
        stack.addWidget(self.control_page)
        stack.addWidget(self.page_path3d)

        btn_info.clicked.connect(lambda: stack.setCurrentIndex(0))
        btn_control.clicked.connect(lambda: stack.setCurrentIndex(1))
        btn_path.clicked.connect(lambda: stack.setCurrentIndex(2))

        root.addWidget(side, 1)  
        root.addWidget(stack, 4) 

        self.bus = SignalBus()
        self.bus.info_text.connect(self._on_info_text)
        self.bus.battery_remaining.connect(self._on_battery_remaining)
        self.bus.odom_xyz.connect(self._on_odom_xyz)
        self.bus.odom_info.connect(self._on_odom_info)
        self.bus.cloud_ready.connect(self.control_page.set_cloud_point)
        rclpy.init(args=None)

        self.node = Px4Node(self.bus)
        self.ros_thr = RosSpinThread(self.node)
        self.ros_thr.start() 

    def _on_info_text(self, text: str):
        self.txt_info.setPlainText(text)
    
    def _on_battery_remaining(self, remaining:float):
        battery_per = int(remaining * 100)
        self.info_page.set_battery(battery_per)

    def _on_odom_xyz(self,x,y,z):
        self.page_path3d.update_path_enu(x,y,abs(z))

    def _on_odom_info(self,msg: VehicleOdometry):
        x, y, z = msg.position
        vx, vy, vz = msg.velocity
        wx, wy, wz = msg.angular_velocity
        self.control_page.set_odom((x, y, z), (vx, vy, vz), (wx, wy, wz))
    

    def publish_position(self, x, y, z):
        self.node.publish_setpoint(x, y, z)

def main():
    app = QApplication(sys.argv)
    w = Px4Viewer()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
