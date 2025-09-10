from PyQt5.QtWidgets import *

class ControlPage(QWidget):
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
        
        v.addLayout(grid)
        v.addWidget(self.coordinateView) 

    def set_odom(self,position,velocity,ang_velocity):
        self.pos_value.setText(f"x={position[0]:.4f}, y={position[1]:.4f}, z={position[2]:.4f}")
        self.vel_value.setText(f"vx={velocity[0]:.4f}, vy={velocity[1]:.4f}, vz={velocity[2]:.4f}")
        self.ang_value.setText(f"wx={ang_velocity[0]:.4f}, wy={ang_velocity[1]:.4f}, wz={ang_velocity[2]:.4f}")
