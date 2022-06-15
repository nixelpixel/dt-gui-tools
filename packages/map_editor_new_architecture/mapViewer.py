# -*- coding: utf-8 -*-
from PyQt5 import QtWidgets, QtGui, QtCore

from classes.Commands.AddObjCommand import AddObjCommand
from classes.Commands.GetLayerCommand import GetLayerCommand
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

        self.handlers_list = [self.tiles, self.watchtowers, self.frames]
        for i in range(len(self.handlers_list) - 1):
            self.handlers_list[i].set_next(self.handlers_list[i+1])
        self.handlers = self.tiles
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
                self.add_obj_image(layer_name, object_name, layer_object)

    def add_obj(self, layer_name: str, item_type: str):
        layer = self.handlers.handle(command=GetLayerCommand(layer_name))
        object_name: str = f"map_1/{item_type}{len(layer) + 1}"
        self.map.map.layers.frames[object_name] = {}
        #self.add_obj_image(layer_name, object_name)
        #self.add_obj_image("fra")
        self.add_obj_on_map(layer_name, object_name)

    def add_obj_image(self, layer_name: str, object_name: str, layer_object = None):
        new_obj = None
        if layer_name == "tiles":
            new_obj = ImageObject(
                f"./img/tiles/{layer_object.type.value}.png", self,
                object_name, (self.map.gridSize, self.map.gridSize))
        elif layer_name == "watchtowers":
            new_obj = DraggableImage(f"./img/objects/{layer_name}.png", self,
                                     object_name)
        if new_obj:
            print(object_name)
            frame_obj = self.map.map.layers.frames[object_name]
            new_coordinates = (
                CoordinatesTransformer.get_x_to_view(frame_obj.pose.x, self.scale,
                                                     self.map.gridSize),
                CoordinatesTransformer.get_y_to_view(frame_obj.pose.y, self.scale,
                                                     self.map.gridSize,
                                                     self.size_map))
            new_obj.rotate_object(frame_obj.pose.yaw)
            new_obj.move_object(new_coordinates)
            self.objects.append(new_obj)

    def add_obj_on_map(self, layer_name: str, object_name: str):

        self.handlers.handle(command=AddObjCommand("frames", object_name))
        self.handlers.handle(command=AddObjCommand(layer_name, object_name))
        print(self.handlers.handle(command=GetLayerCommand("watchtowers")))
        print(self.handlers.handle(command=GetLayerCommand("frames")))
        self.add_obj_image(layer_name, object_name)

    def move_obj_on_map(self, frame_name: str, new_pos: tuple, obj_height: float, obj_width: float):
        map_x = CoordinatesTransformer.get_x_from_view(new_pos[0], self.scale, self.map.gridSize, obj_width)
        map_y = CoordinatesTransformer.get_y_from_view(new_pos[1], self.scale, self.map.gridSize, self.size_map, obj_height)
        self.handlers.handle(command=MoveCommand(frame_name, (map_x, map_y)))

    def rotate_obj_on_map(self, frame_name: str, new_angle: float):
        self.handlers.handle(command=RotateCommand(frame_name, new_angle))
