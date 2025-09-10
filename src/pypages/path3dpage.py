from PyQt5.QtWidgets import QWidget,QVBoxLayout,QHBoxLayout,QPushButton
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