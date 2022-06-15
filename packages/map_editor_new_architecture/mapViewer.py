# -*- coding: utf-8 -*-
from PyQt5 import QtWidgets, QtGui, QtCore

from classes.objects import DraggableImage, ImageObject
from typing import Dict
from layers import TileLayerHandler, WatchtowersLayerHandler, FramesLayerHandler
from mapStorage import MapStorage
from coordinatesTransformer import CoordinatesTransformer
from painter import Painter
from classes.Commands.MoveObjCommand import MoveCommand
from classes.Commands.RotateObjCommand import RotateCommand

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
        # TODO move objects moving on move command
        for layer_name in self.map.map.layers:
            layer = self.map.map.layers[layer_name]
            if not layer_name == "tiles" and not layer_name == "watchtowers":
                continue
            for object_name in layer:
                layer_object = layer[object_name]
                # TODO refactor on more layers
                # TODO use tile_size
                new_obj = None

                if layer_name == "tiles":
                    new_obj = ImageObject(
                        f"./img/tiles/{layer_object.type.value}.png", self,
                        object_name, (self.map.gridSize, self.map.gridSize))
                elif layer_name == "watchtowers":
                    new_obj = DraggableImage(f"./img/objects/{layer_name}.png", self,
                                             object_name)
                if layer_name != "frames":
                    self.objects.append(new_obj)

        # set frames info into objects
        for obj in self.objects:
            frame_obj = self.map.map.layers.frames[obj.name]
            new_coordinates = (
            CoordinatesTransformer.get_x_to_view(frame_obj.pose.x, self.scale,
                                                 self.map.gridSize),
            CoordinatesTransformer.get_y_to_view(frame_obj.pose.y, self.scale,
                                                 self.map.gridSize, self.size_map))
            obj.rotate_object(frame_obj.pose.yaw)
            obj.move_object(new_coordinates)

    def move_obj(self, frame_name: str, new_pos: tuple, obj_height: float, obj_width: float):
        map_x = CoordinatesTransformer.get_x_from_view(new_pos[0], self.scale, self.map.gridSize, obj_width)
        map_y = CoordinatesTransformer.get_y_from_view(new_pos[1], self.scale, self.map.gridSize, self.size_map, obj_height)
        self.handlers.handle(command=MoveCommand(frame_name, (map_x, map_y)))

    def rotate_obj(self, frame_name: str, new_angle: float):
        self.handlers.handle(command=RotateCommand(frame_name, new_angle))
