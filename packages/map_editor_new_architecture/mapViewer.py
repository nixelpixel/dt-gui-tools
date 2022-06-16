# -*- coding: utf-8 -*-
from PyQt5 import QtWidgets, QtGui, QtCore
from dt_maps.types.tiles import Tile

from classes.Commands.AddObjCommand import AddObjCommand
from classes.Commands.GetLayerCommand import GetLayerCommand
from classes.objects import DraggableImage, ImageObject
from typing import Dict
from layers import TileLayerHandler, WatchtowersLayerHandler, FramesLayerHandler
from coordinatesTransformer import CoordinatesTransformer
from painter import Painter
from classes.Commands.MoveObjCommand import MoveCommand
from classes.Commands.RotateObjCommand import RotateCommand
from classes.Commands.ChangeTileTypeCommand import ChangeTileTypeCommand
from utils.maps import default_map_storage

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
    tileSelection = [0] * 4
    rmbPressed = False
    lmbPressed = False
    rmbPrevPos = [0, 0]
    mouseStartX, mouseStartY = 0, 0
    mouseCurX, mouseCurY = 0, 0
    offsetX = 0
    offsetY = 0
    lmbClicked = QtCore.pyqtSignal(int, int)  # click coordinates as an index of the clicked tile

    def __init__(self):
        QtWidgets.QGraphicsView.__init__(self)

        self.setScene(QtWidgets.QGraphicsScene())

        self.painter = Painter()

        self.coordinates_transformer = CoordinatesTransformer()
        self.map = default_map_storage()
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


    def init_objects(self):
        for layer_name in self.map.map.layers:
            layer = self.map.map.layers[layer_name]
            if not layer_name == "tiles" and not layer_name == "watchtowers":
                continue
            for object_name in layer:
                layer_object = layer[object_name]
                # TODO refactor on more layers
                self.add_obj_image(layer_name, object_name, layer_object)

    def add_obj(self, layer_name: str, item_type: str):
        layer = self.handlers.handle(command=GetLayerCommand(layer_name))
        # TODO map_1
        object_name: str = f"map_1/{item_type}{len(layer) + 1}"
        self.add_obj_on_map(layer_name, object_name)

    def add_obj_image(self, layer_name: str, object_name: str, layer_object= None):
        new_obj = None
        if layer_name == "tiles":
            new_obj = ImageObject(
                f"./img/tiles/{layer_object.type.value}.png", self,
                object_name, (self.map.gridSize, self.map.gridSize))
        elif layer_name == "watchtowers":
            new_obj = DraggableImage(f"./img/objects/{layer_name}.png", self,
                                     object_name)
        if new_obj:
            frame_obj = self.map.map.layers.frames[object_name]
            new_coordinates = (
                CoordinatesTransformer.get_x_to_view(frame_obj.pose.x, self.scale,
                                                     self.map.gridSize),
                CoordinatesTransformer.get_y_to_view(frame_obj.pose.y, self.scale,
                                                     self.map.gridSize,
                                                     self.size_map))
            # FIXME rotate object functions duplicate?
            new_obj.rotate_object(frame_obj.pose.yaw)
            new_obj.move_object(new_coordinates)
            self.objects.append(new_obj)

    def add_obj_on_map(self, layer_name: str, object_name: str):
        self.handlers.handle(command=AddObjCommand("frames", object_name))
        self.handlers.handle(command=AddObjCommand(layer_name, object_name))
        self.add_obj_image(layer_name, object_name)

    def move_obj_on_map(self, frame_name: str, new_pos: tuple, obj_height: float, obj_width: float):
        map_x = CoordinatesTransformer.get_x_from_view(new_pos[0], self.scale, self.map.gridSize, obj_width)
        map_y = CoordinatesTransformer.get_y_from_view(new_pos[1], self.scale, self.map.gridSize, self.size_map, obj_height)
        self.handlers.handle(command=MoveCommand(frame_name, (map_x, map_y)))

    def rotate_obj_on_map(self, frame_name: str, new_angle: float):
        self.handlers.handle(command=RotateCommand(frame_name, new_angle))

    def is_selected_tile(self, tile: Tile) -> bool:
        return self.tileSelection[0] <= tile.i <= self.tileSelection[2] and self.tileSelection[3] <= tile.j <= \
               self.tileSelection[1]

    def get_tiles(self):
        return self.handlers.handle(command=GetLayerCommand("tiles"))

    def get_object(self, obj_name: str):
        for obj in self.objects:
            if obj_name == obj.name:
                return obj

    def painting_tiles(self, default_fill: str):
        tiles = self.get_tiles()
        for tile_name in tiles:
            tile = tiles[tile_name]
            if self.is_selected_tile(tile):
                self.change_tile(tile, default_fill)

    def change_tile(self, tile: Tile, new_tile_type: str):
        print('paint')
        img_path = f"./img/tiles/{new_tile_type}.png"
        mutable_obj = self.get_object(tile.key)
        mutable_obj.change_image(img_path)
        self.handlers.handle(command=ChangeTileTypeCommand(tile.key, new_tile_type))
        mutable_obj.rotate_object(0)
        self.rotate_obj_on_map(tile.key, 0)
            
    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        x, y = event.x(), event.y()
        if event.buttons() == QtCore.Qt.LeftButton:
            self.lmbPressed = True
            self.mouseCurX = self.mouseStartX = x
            self.mouseCurY = self.mouseStartY = y

    def mouseMoveEvent(self, event: QtGui.QMouseEvent) -> None:
        if self.lmbPressed:
            self.mouseCurX = event.x()
            self.mouseCurY = event.y()
            print('coords', self.mouseCurX, self.mouseCurY)

    def mouseReleaseEvent(self, event: QtGui.QMouseEvent) -> None:
        if event.button() == QtCore.Qt.LeftButton:
            self.lmbPressed = False
            # TODO is coordinates transformer functions?

            if int((self.mouseStartX - self.offsetX) / self.scale * self.map.gridSize) == int(
                    (self.mouseCurX - self.offsetX) / self.scale * self.map.gridSize) and int(
                (self.mouseStartY - self.offsetY) / self.scale * self.map.gridSize) == int(
                (self.mouseCurY - self.offsetY) / self.scale * self.map.gridSize):
                self.lmbClicked.emit(int((self.mouseStartX - self.offsetX) / self.scale * self.map.gridSize),
                                     int((self.mouseStartY - self.offsetY) / self.scale * self.map.gridSize))

            raw_selection = [
                ((min(self.mouseStartX, self.mouseCurX) - self.offsetX) / self.scale
                 ) / self.map.gridSize,
                CoordinatesTransformer.get_y_from_view(min(self.mouseStartY, self.mouseCurY), self.scale, self.map.gridSize, self.size_map)
                 / self.tile_size,
                ((max(self.mouseStartX, self.mouseCurX) - self.offsetX) / self.scale) / self.map.gridSize,
                CoordinatesTransformer.get_y_from_view(
                    max(self.mouseStartY, self.mouseCurY), self.scale,
                    self.map.gridSize, self.size_map) / self.tile_size
            ]

            if self.get_tiles():
                self.tileSelection = [
                    int(v)  # + (1 if i > 1 else 0)
                    for i, v in enumerate(raw_selection)
                ]
            self.parentWidget().parent().selectionUpdate()
        else:
            self.rmbPressed = False
