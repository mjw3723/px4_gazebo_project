from PyQt5.QtWidgets import QWidget,QVBoxLayout,QHBoxLayout,QLabel,QProgressBar,QTextEdit
from PyQt5.QtCore import Qt

class InfoPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._build_info_page()

    def _build_info_page(self):
        v = QVBoxLayout(self)
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

    def set_battery(self, percent:int):
        self.battery_bar.setValue(percent)
        