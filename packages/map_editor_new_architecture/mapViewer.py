# -*- coding: utf-8 -*-

from PyQt5 import QtWidgets, QtGui, QtCore
from typing import Dict
from layers import TileLayer, WatchtowersLayer
from mapStorage import MapStorage
from coordinatesTransformer import CoordinatesTransformer
from painter import Painter
from utils.window import get_list_dir_with_path

TILES_DIR_PATH = './img/tiles'
OBJECT_DIR_PATHS = ['./img/signs',
                    './img/apriltags',
                    './img/objects']


class MapViewer(QtWidgets.QGraphicsView, QtWidgets.QWidget):
    map = None
    tile_sprites: Dict[str, QtGui.QImage] = {'empty': QtGui.QImage()}
    tiles = None
    watchtowers = None
    size_map = 7
    #citizens = None
    #traffic_signs = None
    #ground_tags = None
    #vehicles = None
    #decorations = None

    scale = 1
    '''
    offsetX = 0
    offsetY = 0
    
    rmbPressed = False
    lmbPressed = False
    rmbPrevPos = [0, 0]
    mouseStartX, mouseStartY = 0, 0
    mouseCurX, mouseCurY = 0, 0
    '''

    def __init__(self):
        QtWidgets.QGraphicsView.__init__(self)

        self.setScene(QtWidgets.QGraphicsScene())

        self.painter = Painter()

        self.coordinates_transformer = CoordinatesTransformer()
        self.map = MapStorage()
        self.tiles = TileLayer()
        #self.watchtowers = WatchtowersLayer()
        #self.citizens = CitizensHandler()
        #self.traffic_signs = TrafficSignsHandler()
        #self.ground_tags = GroundTagsHandler()
        #self.vehicles = VehiclesHandler()
        #self.decorations = DecorationsHandler()

        self.handlers = self.tiles
        #self.handlers_list = [self.tiles]
        #for i in range(len(self.handlers_list) - 1):
        #    self.handlers_list.set_next(self.handlers_list[i+1])

        # load tiles
        for filename, file_path in get_list_dir_with_path(TILES_DIR_PATH):
            tile_name = filename.split('.')[0]
            self.tile_sprites[tile_name] = QtGui.QImage()
            self.tile_sprites[tile_name].load(file_path)

    def drawBackground(self, painter: QtGui.QPainter, rect: QtCore.QRectF):
        self.painter.draw_background(self, painter, handlers=self.handlers)

    '''    

    def mouseMoveEvent(self, event: QtGui.QMouseEvent) -> None:
        x, y = event.x(), event.y()
        x_map = self.get_x_from_view(x)
        y_map = self.get_y_from_view(y)
        if self.drag_mode:
            self.drag_obj.pose.x = x_map
            self.drag_obj.pose.y = y_map
            self.scene().update()
        elif self.rmbPressed:
            self.offsetX += event.x() - self.rmbPrevPos[0]
            self.offsetY += event.y() - self.rmbPrevPos[1]
            self.rmbPrevPos = [event.x(), event.y()]
            self.scene().update()
        elif self.lmbPressed:
            self.mouseCurX = event.x()
            self.mouseCurY = event.y()
            self.scene().update()
    def mouseReleaseEvent(self, event: QtGui.QMouseEvent) -> None:
        self.drag_mode = False
        self.drag_obj = None
        if event.button() == QtCore.Qt.LeftButton:
            self.lmbPressed = False
            if int((self.mouseStartX - self.offsetX) / self.sc * self.map.gridSize) == int(
                    (self.mouseCurX - self.offsetX) / self.sc * self.map.gridSize) and int(
                (self.mouseStartY - self.offsetY) / self.sc * self.map.gridSize) == int(
                (self.mouseCurY - self.offsetY) / self.sc * self.map.gridSize):
                self.lmbClicked.emit(int((self.mouseStartX - self.offsetX) / self.sc * self.map.gridSize),
                                     int((self.mouseStartY - self.offsetY) / self.sc * self.map.gridSize))

            self.raw_selection = [
                ((min(self.mouseStartX, self.mouseCurX) - self.offsetX) / self.sc
                 ) / self.map.gridSize,
                self.get_y_from_view(min(self.mouseStartY, self.mouseCurY)) / self.tile_size,
                ((max(self.mouseStartX, self.mouseCurX) - self.offsetX) / self.sc) / self.map.gridSize,
                self.get_y_from_view(max(self.mouseStartY, self.mouseCurY)) / self.tile_size
            ]

            if self.map.get_tile_layer().visible:
                self.tileSelection = [
                    int(v)  # + (1 if i > 1 else 0)
                    for i, v in enumerate(self.raw_selection)
                ]
            self.selectionChanged.emit()
        else:
            self.rmbPressed = False
        self.scene().update()
        
    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        x, y = event.x(), event.y()
        x_map = self.get_x_from_view(x)
        y_map = self.get_y_from_view(y)
        if event.buttons() == QtCore.Qt.LeftButton:
            self.lmbPressed = True
            self.mouseCurX = self.mouseStartX = x
            self.mouseCurY = self.mouseStartY = y
    '''