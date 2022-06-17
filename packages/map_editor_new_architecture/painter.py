from PyQt5 import QtGui, QtCore, QtWidgets
from mapStorage import MapStorage


class Painter(QtWidgets.QGraphicsView):
    """ Render tiles, other objects using QT API"""

    def __init__(self):
        self.map = MapStorage()
        super(Painter, self).__init__()

    def draw_rect(self, start_pos: tuple,
                  painter: QtGui.QPainter,
                  color: str = "green"):
        painter.resetTransform()
        painter.setPen(QtGui.QColor(color))
        painter.drawRect(
            QtCore.QRectF(start_pos[0] - 1,
                          start_pos[1] - 1,
                          self.map.gridSize + 1,
                          self.map.gridSize + 1
                          ))
