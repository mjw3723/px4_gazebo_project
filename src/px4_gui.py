#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QThread, QObject, pyqtSignal
from px4_msgs.msg import *

class SignalBus(QObject):
    info_text = pyqtSignal(str) 

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

        btn_local  = QPushButton("px4 정보")

        for b in (btn_local,btn_local):
            b.setCursor(Qt.PointingHandCursor)
            b.setMinimumHeight(40)
            left.addWidget(b)

        left.addStretch(1)  

        stack = QStackedWidget()

        page_local = self._build_info_page()
        stack.addWidget(page_local)

        btn_local.clicked.connect(lambda: stack.setCurrentIndex(0))

        root.addWidget(side, 1)  
        root.addWidget(stack, 4) 

        self.bus = SignalBus()
        self.bus.info_text.connect(self._on_info_text)

        rclpy.init(args=None)

        self.node = Px4Node(self.bus)
        self.ros_thr = RosSpinThread(self.node)
        self.ros_thr.start() 

    def _on_info_text(self, text: str):
        self.txt_info.setPlainText(text)

    def _build_info_page(self) -> QWidget:
        w = QWidget()
        v = QVBoxLayout(w)

        title = QLabel("PX4 정보")
        self.txt_info = QTextEdit()
        self.txt_info.setReadOnly(True)
        self.txt_info.setPlaceholderText("Test")

        v.addWidget(title)
        v.addWidget(self.txt_info, 1)
        return w

def main():
    app = QApplication(sys.argv)
    w = Px4Viewer()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
