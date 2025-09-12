from PyQt5.QtWidgets import *
from PyQt5.QtGui import QPainter

from PyQt5.QtCore import Qt, QPointF , pyqtSignal
import open3d as o3d
import math
import numpy as np
import pyqtgraph.opengl as gl
from time import time

class ControlPage(QWidget):
    positionSet = pyqtSignal(float, float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._build_control_page()

    def _build_control_page(self):
        v = QVBoxLayout(self)
        grid = QGridLayout()
        pos_label = QLabel("현재 좌표:")
        self.pos_value = QLabel("x=0.0000, y=0.0000, z=0.0000")
        grid.addWidget(pos_label, 1, 0)
        grid.addWidget(self.pos_value, 1, 1, 1, 2)

        vel_label = QLabel("속도 (m/s):")
        self.vel_value = QLabel("vx=0.0000, vy=0.0000, vz=0.0000")
        grid.addWidget(vel_label, 2, 0)
        grid.addWidget(self.vel_value, 2, 1, 1, 2)

        ang_label = QLabel("각속도 (rad/s):")
        self.ang_value = QLabel("wx=0.0000, wy=0.0000, wz=0.0000")
        grid.addWidget(ang_label, 3, 0)
        grid.addWidget(self.ang_value, 3, 1, 1, 2)
        control_box = QHBoxLayout()
        self.coordinateView = CoordinateView()
        # self.coordinateView.setMinimumHeight(100) 
        # self.coordinateView.setMinimumWidth(100)
        self.coordinateView.positionSelected.connect(self.set_position_xy)
        control_box.addWidget(self.coordinateView,2)
        
        side_box = QVBoxLayout()
        x_box = QHBoxLayout()
        x_label = QLabel("지정 X:")
        self.x_edit = QLineEdit('0.0')
        x_box.addWidget(x_label)
        x_box.addWidget(self.x_edit)

        y_box = QHBoxLayout()
        y_label = QLabel("지정 Y:")
        self.y_edit = QLineEdit('0.0')
        y_box.addWidget(y_label)
        y_box.addWidget(self.y_edit)

        z_box = QHBoxLayout()
        z_label = QLabel("지정 Z:")
        self.z_edit = QLineEdit('0.0')
        z_box.addWidget(z_label)
        z_box.addWidget(self.z_edit)
        
        btn_box = QHBoxLayout()
        set_position_btn = QPushButton('지정')
        set_position_btn.clicked.connect(self.on_set_position_clicked)
        btn_box.addWidget(set_position_btn)
        

        side_box.addLayout(x_box)
        side_box.addLayout(y_box)
        side_box.addLayout(z_box)
        side_box.addLayout(btn_box)
        side_box.addStretch(1)

        control_box.addLayout(side_box, 1)
        point_box = QHBoxLayout()
        self.pointcloudView = PointCloudView()
        self.pointcloudView.setMinimumSize(300, 300) 
        point_box.addWidget(self.pointcloudView, 1) 
        
        v.addLayout(grid)
        v.addLayout(control_box) 
        v.addLayout(point_box)


    def set_odom(self,position,velocity,ang_velocity):
        self.pos_value.setText(f"x={position[0]:.4f}, y={position[1]:.4f}, z={position[2]:.4f}")
        self.vel_value.setText(f"vx={velocity[0]:.4f}, vy={velocity[1]:.4f}, vz={velocity[2]:.4f}")
        self.ang_value.setText(f"wx={ang_velocity[0]:.4f}, wy={ang_velocity[1]:.4f}, wz={ang_velocity[2]:.4f}")

    def set_position_xy(self,x,y):
        self.x_edit.setText(x)
        self.y_edit.setText(y)
    
    def set_cloud_point(self,points):
        self.pointcloudView.update_cloud(points)

    def on_set_position_clicked(self):
        x = float(self.x_edit.text())
        y = float(self.y_edit.text())
        z = float(self.z_edit.text())
        self.positionSet.emit(x, y, z)


class CoordinateView(QGraphicsView):
    positionSelected = pyqtSignal(str, str)

    def __init__(self):
        super().__init__()
        self.setMouseTracking(True)
        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)

        self.scene.addLine(-50, 0, 50, 0) 
        self.scene.addLine(0, -50, 0, 50)
        self.setRenderHint(QPainter.Antialiasing)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.drone_item = QGraphicsEllipseItem(-5, -5, 10, 10)
        self.drone_item.setBrush(Qt.red)
        self.scene.addItem(self.drone_item)

    def update_drone_position(self, x, y):
        self.drone_item.setPos(QPointF(x, -y))  

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            pos = self.mapToScene(event.pos())
            pos_x = f"{pos.x():.2f}"
            pos_y = f"{pos.y():.2f}"
            self.positionSelected.emit(pos_x,pos_y)
            print(f"클릭한 좌표: x={pos.x():.2f}, y={-pos.y():.2f}")
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        pos = self.mapToScene(event.pos())  
        QToolTip.showText(event.globalPos(), f"x={pos.x():.2f}, y={-pos.y():.2f}")
        super().mouseMoveEvent(event)

class PointCloudView(gl.GLViewWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.last_update = 0
        self.scatter = gl.GLScatterPlotItem()
        self.addItem(self.scatter)

        self.setBackgroundColor('k')  
        self.opts['distance'] = 100   
        self.opts['fov'] = 60         
    

    def update_cloud(self, points: np.ndarray):
        now = time()
        if now - self.last_update < 0.5:  
            return
        self.last_update = now
        if points.shape[0] == 0:
            return
        
        if points.shape[0] > 1000:
            idx = np.random.choice(points.shape[0], 1000, replace=False)
            points = points[idx]

        colors = np.ones((points.shape[0], 4))  
        z_norm = (points[:, 2] - points[:, 2].min()) / (points[:, 2].ptp() + 1e-9)
        colors[:, 0] = z_norm  

        self.removeItem(self.scatter)
        self.scatter = gl.GLScatterPlotItem(pos=points, color=colors, size=2)
        self.addItem(self.scatter)
