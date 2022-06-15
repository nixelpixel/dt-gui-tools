from PyQt5 import QtGui, QtCore
from classes.Commands.GetLayerCommand import GetLayerCommand
from utils.window import get_degree_for_orientation

class Painter:
    """ Render tiles, other objects using QT API"""
    def draw_background(self, map_viewer, painter: QtGui.QPainter, handlers):
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        painter.resetTransform()
        painter.fillRect(0, 0, map_viewer.size().width(), map_viewer.size().height(), QtGui.QColor('darkGray'))
        global_transform = QtGui.QTransform()
        global_transform.translate(0, 0)
        painter.setTransform(global_transform, False)

        # Draw tile layer

        tiles = handlers.handle(GetLayerCommand("tiles"))
        if tiles:
            self.draw_tiles(map_viewer, painter, tiles, global_transform)
        painter.resetTransform()

    def draw_tiles(self, map_viewer, painter: QtGui.QPainter, tiles, global_transform):
        tiles = tiles.values()
        for tile in tiles:
            orientation = tile.orientation
            painter.scale(map_viewer.scale, map_viewer.scale)
            painter.translate(tile.i * map_viewer.map.gridSize,
                              (map_viewer.size_map - 1 - tile.j) * map_viewer.map.gridSize)

            my_transform = QtGui.QTransform()
            degree = get_degree_for_orientation(orientation.value)
            # check, because QTransform().rotate clockwise
            if not degree / 90 % 2:
                my_transform.rotate(degree)
            else:
                my_transform.rotate(degree + 180)

            img = map_viewer.tile_sprites[tile.type.value].transformed(my_transform)
            painter.drawImage(
                QtCore.QRectF(0, 0, map_viewer.map.gridSize, map_viewer.map.gridSize),
                img)

            painter.setPen(QtGui.QColor('white'))
            painter.drawRect(
                QtCore.QRectF(0, 0, map_viewer.map.gridSize, map_viewer.map.gridSize))

            if tile.j == 0 and tile.i == 0:
                painter.setPen(QtGui.QColor('blue'))
                painter.drawRect(QtCore.QRectF(1, 1, map_viewer.map.gridSize - 1,
                                               map_viewer.map.gridSize - 1))
            painter.setTransform(global_transform, False)
