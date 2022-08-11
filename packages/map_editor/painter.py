from typing import Tuple
from PyQt5 import QtGui, QtCore, QtWidgets
from mapStorage import MapStorage


class Painter(QtWidgets.QGraphicsView):
    """ Render tiles, other objects using QT API"""

    def __init__(self):
        self.map = MapStorage()
        super(Painter, self).__init__()
        
    def fill_background(self, painter: QtGui.QColor, color: str, width: float,
                        height: float):
        painter.resetTransform()
        painter.fillRect(0, 0, width, height,
                         QtGui.QColor(color))

    def draw_rect(self, start_pos: Tuple[float, float],
                  scale: float,
                  painter: QtGui.QPainter,
                  grid_width: float,
                  grid_height: float,
                  color: str = "green"):
        painter.resetTransform()
        painter.setPen(QtGui.QColor(color))
        painter.drawRect(
            QtCore.QRectF(start_pos[0] - 1,
                          start_pos[1] - 1,
                          grid_width * scale + 1,
                          grid_height * scale + 1
                          ))
