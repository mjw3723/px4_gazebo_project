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
from OpenGL import GL
from PyQt5.QtGui import QVector3D
from pyqtgraph.opengl import GLViewWidget, GLGridItem, GLLinePlotItem, GLScatterPlotItem
import math
import numpy as np
class Path3DPage(QWidget):
    """
    - self.update_path_enu(x, y, z): 로컬 ENU 좌표(m)로 경로 갱신
    - self.clear(): 경로 초기화
    """
    def __init__(self, parent=None):
        super().__init__(parent)

        self._pts = []      
        self._line = None
        self._dot = None
        self._origin_llh = None 

        # UI
        v = QVBoxLayout(self)
        top = QHBoxLayout()
        btn_clear = QPushButton("경로 지우기")
        btn_clear.clicked.connect(self.clear)
        top.addStretch(1)
        top.addWidget(btn_clear)
        v.addLayout(top)

        self.view = GLViewWidget()
        self.view.opts['distance'] = 50.0  
        self.view.opts['elevation'] = 20.0  
        self.view.opts['azimuth'] = 45.0
        v.addWidget(self.view, 1)

        grid = GLGridItem()
        grid.scale(5, 5, 1)  
        self.view.addItem(grid)

        self._line = GLLinePlotItem(pos=np.empty((0, 3), dtype=float),width=2.0, antialias=True, color=(0.2, 0.8, 1.0, 1.0))
        self.view.addItem(self._line)

        self._dot = GLScatterPlotItem(pos=np.empty((0, 3), dtype=float),size=8, pxMode=True)
        self.view.addItem(self._dot)

    def clear(self):
        self._pts.clear()
        self._origin_llh = None
        self._line.setData(pos=np.empty((0, 3)))
        self._dot.setData(pos=np.empty((0, 3)))

    def update_path_enu(self, x, y, z):
        self._pts.append((float(x), float(y), float(z)))
        arr = np.asarray(self._pts, dtype=float)
        self._line.setData(pos=arr)

        self._dot.setData(pos=arr[-1][None, :], color=(1.0, 0.0, 0.0, 1.0))

        if len(arr) == 1:
            p = arr[-1]
            self.view.setCameraPosition(pos=QVector3D(float(p[0]), float(p[1]), float(p[2])),
                                        distance=30.0)

class SignalBus(QObject):
    info_text = pyqtSignal(str)
    battery_remaining = pyqtSignal(float)
    odom_xyz = pyqtSignal(float,float,float)

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
            self.odom_xyz_callback,
            qos
        )

    def battery_callback(self,msg:BatteryStatus):
        battery_remaining = msg.remaining
        self.bus.battery_remaining.emit(battery_remaining)

    def odom_xyz_callback(self,msg:VehicleOdometry):
        odom_xyz = msg.position
        self.bus.odom_xyz.emit(odom_xyz[0],odom_xyz[1],odom_xyz[2])
        
    
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
        btn_path = QPushButton("이동 경로") 
        for b in (btn_info,btn_path):
            b.setCursor(Qt.PointingHandCursor)
            b.setMinimumHeight(40)
            left.addWidget(b)

        left.addStretch(1)  

        stack = QStackedWidget()

        info_page = self._build_info_page()
        path_page = self._build_path_page()
        self.page_path3d = Path3DPage()
        stack.addWidget(info_page)
        stack.addWidget(self.page_path3d)

        btn_info.clicked.connect(lambda: stack.setCurrentIndex(0))
        btn_path.clicked.connect(lambda: stack.setCurrentIndex(1))

        root.addWidget(side, 1)  
        root.addWidget(stack, 4) 

        self.bus = SignalBus()
        self.bus.info_text.connect(self._on_info_text)
        self.bus.battery_remaining.connect(self._on_battery_remaining)
        self.bus.odom_xyz.connect(self._on_odom_xyz)

        rclpy.init(args=None)

        self.node = Px4Node(self.bus)
        self.ros_thr = RosSpinThread(self.node)
        self.ros_thr.start() 

    def _on_info_text(self, text: str):
        self.txt_info.setPlainText(text)
    
    def _on_battery_remaining(self, remaining:float):
        battery_per = int(remaining * 100)
        self.battery_bar.setValue(battery_per)
    
    def _on_odom_xyz(self,x,y,z):
        self.page_path3d.update_path_enu(x,y,abs(z))

        

    def _build_info_page(self) -> QWidget:
        w = QWidget()
        v = QVBoxLayout(w)
        title = QLabel("PX4 정보")

        battery_box = QHBoxLayout()
        battery_label = QLabel("배터리 상태:")
        self.battery_bar = QProgressBar()
        self.battery_bar.setRange(0, 100)  
        self.battery_bar.setValue(0)         
        self.battery_bar.setFormat("%p%")     
        self.battery_bar.setAlignment(Qt.AlignCenter)

        battery_box.addWidget(battery_label)
        battery_box.addWidget(self.battery_bar, 1)
        self.txt_info = QTextEdit()
        self.txt_info.setReadOnly(True)
        self.txt_info.setPlaceholderText("Test")

        v.addWidget(title)
        v.addLayout(battery_box)
        v.addWidget(self.txt_info, 1)
        return w
    
    def _build_path_page(self) -> QWidget:
        w = QWidget()
        v = QVBoxLayout(w)
        title = QLabel("이동경로")
        
        v.addWidget(title)
        return w

def main():
    app = QApplication(sys.argv)
    w = Px4Viewer()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
