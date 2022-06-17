# -*- coding: utf-8 -*-
from PyQt5 import QtWidgets, QtGui, QtCore
from dt_maps.types.tiles import Tile
from classes.Commands.AddObjCommand import AddObjCommand
from classes.Commands.GetLayerCommand import GetLayerCommand
from classes.objects import DraggableImage, ImageObject
from typing import Dict, Any, Optional
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
    objects = [] # TODO replace to dict
    #citizens = None
    #traffic_signs = None
    #ground_tags = None
    #vehicles = None
    #decorations = None

    scale = 1
    tile_selection = [0] * 4
    rmbPressed = False
    lmbPressed = False
    rmbPrevPos = [0, 0]
    mouse_start_x, mouse_start_y = 0, 0
    mouse_cur_x, mouse_cur_y = 0, 0
    offset_x = 0
    offset_y = 0
    lmbClicked = QtCore.pyqtSignal(int, int)  # click coordinates as an index of the clicked tile

    def __init__(self):
        QtWidgets.QGraphicsView.__init__(self)

        self.setScene(QtWidgets.QGraphicsScene())

        self.painter = Painter()
        self.map = default_map_storage()
        self.coordinates_transformer = CoordinatesTransformer(self.scale, self.size_map, self.map.gridSize)
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

    def init_objects(self) -> None:
        for layer_name in self.map.map.layers:
            layer = self.map.map.layers[layer_name]
            if not layer_name == "tiles" and not layer_name == "watchtowers":
                continue
            for object_name in layer:
                layer_object = layer[object_name]
                # TODO refactor on more layers
                self.add_obj_image(layer_name, object_name, layer_object)

    def add_obj(self, layer_name: str, item_type: str) -> None:
        layer = self.handlers.handle(command=GetLayerCommand(layer_name))
        # TODO map_1
        object_name: str = f"map_1/{item_type}{len(layer) + 1}"
        self.add_obj_on_map(layer_name, object_name)

    def add_obj_image(self, layer_name: str, object_name: str, layer_object=None) -> None:
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
                self.coordinates_transformer.get_x_to_view(frame_obj.pose.x),
                self.coordinates_transformer.get_y_to_view(frame_obj.pose.y))
            self.rotate_obj(new_obj, frame_obj.pose.yaw)
            self.move_obj(new_obj, new_coordinates)
            self.objects.append(new_obj)

    def add_obj_on_map(self, layer_name: str, object_name: str) -> None:
        self.handlers.handle(command=AddObjCommand("frames", object_name))
        self.handlers.handle(command=AddObjCommand(layer_name, object_name))
        self.add_obj_image(layer_name, object_name)

    def move_obj(self, obj: ImageObject, new_coordinates: tuple) -> None:
        obj.move_object(new_coordinates)

    def move_obj_on_map(self, frame_name: str, new_pos: tuple, obj_height: float, obj_width: float) -> None:
        map_x = self.coordinates_transformer.get_x_from_view(new_pos[0], obj_width=obj_width)
        map_y = self.coordinates_transformer.get_y_from_view(new_pos[1], obj_height=obj_height)
        self.handlers.handle(command=MoveCommand(frame_name, (map_x, map_y)))

    def rotate_obj(self, obj: ImageObject, new_angle: float) -> None:
        obj.rotate_object(new_angle)

    def rotate_obj_on_map(self, frame_name: str, new_angle: float) -> None:
        self.handlers.handle(command=RotateCommand(frame_name, new_angle))

    def scaled_obj(self, obj: ImageObject, args: Dict[str, Any]):
        scale = args["scale"]
        obj.scale_object(scale)
        new_coordinates = (obj.pos().x() * self.scale,
                           obj.pos().y() * self.scale)
        self.move_obj(obj, new_coordinates)

    def rotate_with_button(self, args: Dict[str, Any]) -> None:
        tile_name = args["tile_name"]
        obj = self.get_object(tile_name)
        self.rotate_obj(obj, obj.yaw + 90)
        self.rotate_obj_on_map(tile_name, obj.yaw)

    def is_selected_tile(self, tile: Tile) -> bool:
        return self.tile_selection[0] <= tile.i <= self.tile_selection[2] and self.tile_selection[3] <= tile.j <= \
               self.tile_selection[1]

    def get_tiles(self) -> Dict[str, Any]:
        return self.handlers.handle(command=GetLayerCommand("tiles"))

    def get_object(self, obj_name: str) -> Optional[ImageObject]:
        for obj in self.objects:
            if obj_name == obj.name:
                return obj

    def change_tiles_handler(self, handler_func, args: Dict[str, Any]) -> None:
        tiles = self.get_tiles()
        for tile_name in tiles:
            tile = tiles[tile_name]
            if self.is_selected_tile(tile):
                args["tile_name"] = tile_name
                handler_func(args)

    def change_object_handler(self, handler_func, args: Dict[str, Any]) -> None:
        for obj in self.objects:
            handler_func(obj, args)

    def painting_tiles(self, default_fill: str) -> None:
        self.change_tiles_handler(self.change_tile_type,
                                  {"default_fill": default_fill})

    def rotate_tiles(self) -> None:
        self.change_tiles_handler(self.rotate_with_button, {})

    def change_tile_type(self, args: Dict[str, Any]) -> None:
        new_tile_type = args["default_fill"]
        tile_name = args["tile_name"]
        img_path = f"./img/tiles/{new_tile_type}.png"
        mutable_obj = self.get_object(tile_name)
        mutable_obj.change_image(img_path)
        self.handlers.handle(command=ChangeTileTypeCommand(tile_name,
                                                           new_tile_type))
        self.rotate_obj_on_map(tile_name, 0)

    def wheelEvent(self, event: QtGui.QWheelEvent) -> None:
        print( 2 ** (event.angleDelta().y() / 360))
        sf = 2 ** (event.angleDelta().y() / 240)
        self.scale *= sf
        if self.map.gridSize < 10 and sf < 1:
            return
        if self.map.gridSize > 1000 and sf > 1:
            return

        self.map.gridSize *= self.scale
        self.coordinates_transformer.set_scale(self.scale)
        self.coordinates_transformer.set_grid_size(self.map.gridSize)
        self.change_object_handler(self.scaled_obj, {"scale": self.scale})

    def mousePressEvent(self, event: tuple) -> None:
        start_pos = event[1]
        event = event[0]
        x, y = event.x(), event.y()
        if event.buttons() == QtCore.Qt.LeftButton:
            self.lmbPressed = True
            self.mouse_cur_x = self.mouse_start_x = x
            self.mouse_cur_y = self.mouse_start_y = y
            self.offset_x = start_pos[0]
            self.offset_y = start_pos[1]

    def mouseMoveEvent(self, event: QtGui.QMouseEvent) -> None:
        if self.lmbPressed:
            self.mouse_cur_x = event.x()
            self.mouse_cur_y = event.y()

    def mouseReleaseEvent(self, event: QtGui.QMouseEvent) -> None:
        if event.button() == QtCore.Qt.LeftButton:
            self.lmbPressed = False
            self.select_tiles()
            self.parentWidget().parent().selectionUpdate()
        else:
            self.rmbPressed = False

    def select_tiles(self) -> None:
        raw_selection = [
            self.coordinates_transformer.get_x_from_view(
                min(self.mouse_start_x, self.mouse_cur_x), self.offset_x),
            self.coordinates_transformer.get_y_from_view(
                min(self.mouse_start_y, self.mouse_cur_y), self.offset_y),
            self.coordinates_transformer.get_x_from_view(
                max(self.mouse_start_x, self.mouse_cur_x), self.offset_x),
            self.coordinates_transformer.get_y_from_view(
                max(self.mouse_start_y, self.mouse_cur_y), self.offset_y),
        ]

        if self.get_tiles():
            self.tile_selection = [
                int(v)  # + (1 if i > 1 else 0)
                for i, v in enumerate(raw_selection)
            ]
