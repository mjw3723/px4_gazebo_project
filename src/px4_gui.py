#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QTimer, Qt
from px4_msgs.msg import *

class Px4Viewer(Node):
    def __init__(self):
        super().__init__('pyqt_px4_localpos_viewer')


def build_ui():
    win = QWidget()
    win.setWindowTitle("PX4 Viewer")
    win.resize(1280, 800)

    root = QHBoxLayout(win)

    left = QVBoxLayout()
    left.setSpacing(8)

    side = QFrame()
    side.setFrameShape(QFrame.StyledPanel)
    side.setLayout(left)

    btn_local  = QPushButton("위치")

    for b in (btn_local,btn_local):
        b.setCursor(Qt.PointingHandCursor)
        b.setMinimumHeight(40)
        left.addWidget(b)

    left.addStretch(1)  

    stack = QStackedWidget()

    page_local = QWidget()
    lay_local = QVBoxLayout(page_local)
    lay_local.addWidget(QLabel("위치 화면"))
    txt_local = QTextEdit()
    txt_local.setPlaceholderText("test")
    lay_local.addWidget(txt_local, 1)

    stack.addWidget(page_local)

    btn_local.clicked.connect(lambda: stack.setCurrentIndex(0))

    root.addWidget(side, 1)  
    root.addWidget(stack, 4)  

    return win


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    win = build_ui()
    win.show()
    node = Px4Viewer()
    pump = QTimer()
    pump.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    pump.start(10)  

    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
