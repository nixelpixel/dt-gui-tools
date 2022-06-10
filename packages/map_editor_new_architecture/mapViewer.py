# -*- coding: utf-8 -*-
from enum import Enum
from PyQt5 import QtWidgets, QtGui, QtCore

from classes.objects import DraggableImage, ImageObject
from typing import Dict
from layers import TileLayerHandler, WatchtowersLayerHandler, FramesLayerHandler
from mapStorage import MapStorage
from coordinatesTransformer import CoordinatesTransformer
from painter import Painter

TILES_DIR_PATH = './img/tiles'
OBJECT_DIR_PATHS = ['./img/signs',
                    './img/apriltags',
                    './img/objects']
class TileType(Enum):
    STRAIGHT = "straight"
    CURVE_LEFT = "curve_left"
    CURVE_RIGHT = "curve_right"
    ASPHALT = "asphalt"
    FLOOR = "floor"
    GRASS = "grass"
    THREE_WAY_LEFT = "3way_left"
    THREE_WAY_RIGHT = "3way_right"
    FOUR_WAY = "4way"

class MapViewer(QtWidgets.QGraphicsView, QtWidgets.QWidget):
    map = None
    tile_sprites: Dict[str, QtGui.QImage] = {'empty': QtGui.QImage()}
    tiles = None
    watchtowers = None
    size_map = 7
    tile_size = 0.585
    objects = []
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
        self.tiles = TileLayerHandler()
        self.watchtowers = WatchtowersLayerHandler()
        self.frames = FramesLayerHandler()
        #self.citizens = CitizensHandler()
        #self.traffic_signs = TrafficSignsHandler()
        #self.ground_tags = GroundTagsHandler()
        #self.vehicles = VehiclesHandler()
        #self.decorations = DecorationsHandler()

        self.handlers_list = [self.watchtowers, self.frames]
        self.handlers = self.tiles
        for i in range(len(self.handlers_list) - 1):
            self.handlers.set_next(self.handlers_list[i+1])

        self.init_objects()


    def drawBackground(self, painter: QtGui.QPainter, rect: QtCore.QRectF):
        #self.painter.draw_background(self, painter, handlers=self.handlers)
        pass

    def init_objects(self):
        # TODO move objects moving to painter class or on move command
        for layer_name in self.map.map.layers:
            layer = self.map.map.layers[layer_name]
            for object_name in layer:
                layer_object = layer[object_name]
                # TODO refactor on more layers
                # TODO rotate tiles
                # TODO use tile_size
                new_obj = None
                if layer_name == "tiles":
                    new_obj = ImageObject(
                        f"./img/tiles/{layer_object.type.value}.png", self,
                        object_name, (self.map.gridSize, self.map.gridSize))
                    new_coordinates = (CoordinatesTransformer.get_x_to_view(layer_object.i, self.scale, self.map.gridSize),
                                       CoordinatesTransformer.get_y_to_view(layer_object.j, self.scale, self.map.gridSize, self.size_map))
                    new_obj.move_object(new_coordinates)
                elif layer_name != "frames":
                    new_obj = DraggableImage(f"./img/objects/{layer_name}.png", self,
                                             object_name)
