# -*- coding: utf-8 -*-
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtCore import QRect, QPoint
from dt_maps.types.tiles import Tile
from classes.Commands.AddObjCommand import AddObjCommand
from classes.Commands.DeleteObjCommand import DeleteObjCommand
from classes.Commands.GetLayerCommand import GetLayerCommand
from classes.objects import DraggableImage, ImageObject
from typing import Dict, Any, Optional
from layers import TileLayerHandler, WatchtowersLayerHandler, FramesLayerHandler
from coordinatesTransformer import CoordinatesTransformer
from painter import Painter
from classes.Commands.MoveObjCommand import MoveObjCommand
from classes.Commands.RotateObjCommand import RotateCommand
from classes.Commands.ChangeTileTypeCommand import ChangeTileTypeCommand
from classes.Commands.MoveTileCommand import MoveTileCommand
from utils.maps import default_map_storage, get_map_height, get_map_width
from classes.MapDescription import MapDescription
from pathlib import Path

TILES_DIR_PATH = './img/tiles'
OBJECT_DIR_PATHS = ['./img/signs',
                    './img/apriltags',
                    './img/objects']


class MapViewer(QtWidgets.QGraphicsView, QtWidgets.QWidget):
    map = None
    tile_sprites: Dict[str, QtGui.QImage] = {'empty': QtGui.QImage()}
    tiles = None
    watchtowers = None
    frames = None
    map_height = 10
    tile_size = 0.585
    objects = {}
    handlers = None
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
    is_to_png = False

    def __init__(self):
        QtWidgets.QGraphicsView.__init__(self)

        self.setScene(QtWidgets.QGraphicsScene())
        self.map = default_map_storage()
        self.coordinates_transformer = CoordinatesTransformer(self.scale,
                                                              self.map_height,
                                                              self.map.gridSize)
        self.painter = Painter()
        self.init_handlers()
        self.init_objects()
        self.set_map_size()
        self.setMouseTracking(True)

    def init_objects(self) -> None:
        for layer_name in self.map.map.layers:
            layer = self.handlers.handle(GetLayerCommand(layer_name))
            if not layer_name == "tiles" and not layer_name == "watchtowers" \
                    or not layer:
                continue
            for object_name in layer:
                layer_object = layer[object_name]
                # TODO refactor on more layers
                self.add_obj_image(layer_name, object_name, layer_object)

    def init_handlers(self):
        self.tiles = TileLayerHandler()
        self.watchtowers = WatchtowersLayerHandler()
        self.frames = FramesLayerHandler()
        # self.citizens = CitizensHandler()
        # self.traffic_signs = TrafficSignsHandler()
        # self.ground_tags = GroundTagsHandler()
        # self.vehicles = VehiclesHandler()
        # self.decorations = DecorationsHandler()

        handlers_list = [self.tiles, self.watchtowers, self.frames]
        for i in range(len(handlers_list) - 1):
            handlers_list[i].set_next(handlers_list[i + 1])
        self.handlers = self.tiles

    def add_obj(self, layer_name: str, item_type: str) -> None:
        # TODO map_1
        i = 1
        while True:
            object_name: str = f"map_1/{item_type}{i}"
            if object_name not in self.objects:
                self.add_obj_on_map(layer_name, object_name)
                self.add_obj_image(layer_name, object_name)
                self.scaled_obj(self.get_object(object_name),
                                {'scale': self.scale})
                break
            i += 1

    def add_obj_image(self, layer_name: str, object_name: str, layer_object=None) -> None:
        new_obj = None
        if layer_name == "tiles" and layer_object:
            new_obj = ImageObject(
                f"./img/tiles/{layer_object.type.value}.png", self,
                object_name, layer_name, (self.map.gridSize, self.map.gridSize))
        elif layer_name == "watchtowers":
            new_obj = DraggableImage(f"./img/objects/{layer_name}.png", self,
                                     object_name, layer_name)
        if new_obj:
            frame_obj = self.handlers.handle(
                command=GetLayerCommand("frames"))[object_name]

            new_coordinates = (
                self.coordinates_transformer.get_x_to_view(frame_obj.pose.x),
                self.coordinates_transformer.get_y_to_view(frame_obj.pose.y))
            self.set_obj_map_pos(new_obj, (frame_obj.pose.x, frame_obj.pose.y))
            self.rotate_obj(new_obj, frame_obj.pose.yaw)
            self.move_obj(new_obj, new_coordinates)
            self.objects[object_name] = new_obj

    def add_obj_on_map(self, layer_name: str, object_name: str) -> None:
        self.add_frame_on_map(object_name)
        self.handlers.handle(command=AddObjCommand(layer_name, object_name))
        
    def add_frame_on_map(self, frame_name: str):
        self.handlers.handle(command=AddObjCommand("frames", frame_name))

    def delete_obj_on_map(self, obj: ImageObject) -> None:
        self.handlers.handle(command=DeleteObjCommand("frames", obj.name))
        self.handlers.handle(command=DeleteObjCommand(obj.layer_name,
                                                      obj.name))

    def delete_objects(self):
        for obj_name in self.objects:
            obj = self.get_object(obj_name)
            obj.delete_object()
        self.objects.clear()

    def move_obj(self, obj: ImageObject, new_coordinates: tuple) -> None:
        obj.move_object(new_coordinates)

    def set_obj_map_pos(self, obj: ImageObject, new_pos: tuple) -> None:
        obj.set_obj_map_pos(new_pos)

    def move_obj_on_map(self, frame_name: str,
                        new_pos: tuple,
                        obj_height: float,
                        obj_width: float) -> None:
        map_x = self.coordinates_transformer.get_x_from_view(new_pos[0], obj_width=obj_width)
        map_y = self.coordinates_transformer.get_y_from_view(new_pos[1], obj_height=obj_height)
        obj = self.get_object(frame_name)
        self.set_obj_map_pos(obj, (map_x, map_y))
        self.handlers.handle(command=MoveObjCommand(frame_name, (map_x, map_y)))

    def rotate_obj(self, obj: ImageObject, new_angle: float) -> None:
        obj.rotate_object(new_angle)
        self.scene_update()

    def rotate_obj_on_map(self, frame_name: str, new_angle: float) -> None:
        self.handlers.handle(command=RotateCommand(frame_name, new_angle))

    def scaled_obj(self, obj: ImageObject, args: Dict[str, Any]) -> None:
        scale = args["scale"]
        obj.scale_object(scale)
        new_coordinates = (
            self.coordinates_transformer.get_x_to_view(obj.obj_map_pos[0]),
            self.coordinates_transformer.get_y_to_view(obj.obj_map_pos[1]))
        self.move_obj(obj, new_coordinates)

    def set_map_size(self, height: int = 0) -> None:
        if not height:
            self.map_height = get_map_height(self.get_tiles())
        else:
            self.map_height = height
        self.coordinates_transformer.set_size_map(self.map_height)

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
        return self.objects[obj_name]

    def delete_object(self, obj: ImageObject) -> None:
        self.delete_obj_on_map(obj)
        self.objects.__delitem__(obj.name)

    def change_tiles_handler(self, handler_func, args: Dict[str, Any]) -> None:
        tiles = self.get_tiles()
        for tile_name in tiles:
            tile = tiles[tile_name]
            if self.is_selected_tile(tile):
                args["tile_name"] = tile_name
                args["tile"] = tile
                handler_func(args)

    def change_object_handler(self, handler_func, args: Dict[str, Any]) -> None:
        for obj_name in self.objects:
            handler_func(self.get_object(obj_name), args)

    def painting_tiles(self, default_fill: str) -> None:
        self.change_tiles_handler(self.change_tile_type,
                                  {"default_fill": default_fill})

    def rotate_tiles(self) -> None:
        self.change_tiles_handler(self.rotate_with_button, {})

    def highlight_select_tile(self, args: Dict[str, Any]):
        tile = self.get_object(args["tile_name"])
        self.painter.draw_rect((tile.pos().x() - 1, tile.pos().y() - 1),
                               self.scale,
                               args["painter"])

    def change_tile_type(self, args: Dict[str, Any]) -> None:
        new_tile_type = args["default_fill"]
        tile_name = args["tile_name"]
        img_path = f"./img/tiles/{new_tile_type}.png"
        mutable_obj = self.get_object(tile_name)
        mutable_obj.change_image(img_path, new_tile_type)
        self.handlers.handle(command=ChangeTileTypeCommand(tile_name,
                                                           new_tile_type))
        self.rotate_obj_on_map(tile_name, 0)

    def wheelEvent(self, event: QtGui.QWheelEvent) -> None:
        sf = 1.5 ** (event.angleDelta().y() / 240)
        if self.scale * sf < 0.1:
            return
        elif self.scale * sf > 5:
            return
        else:
            self.scale *= sf
        self.coordinates_transformer.set_scale(self.scale)
        self.change_object_handler(self.scaled_obj, {"scale": self.scale})
        self.scene_update()

    def scene_update(self) -> None:
        self.scene().update()

    def mousePressEvent(self, event: tuple) -> None:
        if isinstance(event, tuple):
            start_pos = event[1]
            event = event[0]
            x, y = event.x(), event.y()
            if event.buttons() == QtCore.Qt.LeftButton:
                self.lmbPressed = True
                self.mouse_cur_x = self.mouse_start_x = x
                self.mouse_cur_y = self.mouse_start_y = y
                self.offset_x = start_pos[0]
                self.offset_y = start_pos[1]

    def mouseMoveEvent(self, event: tuple) -> None:
        if isinstance(event, tuple):
            start_pos = event[1]
            event = event[0]
            if self.lmbPressed:
                self.mouse_cur_x = event.x()
                self.mouse_cur_y = event.y()
            pos = (start_pos[0] + event.x(), start_pos[1] + event.y())
            self.parentWidget().parent().update_debug_info({"mode": "set_cursor_pos", "pos" : pos})

    def mouseReleaseEvent(self, event: QtGui.QMouseEvent) -> None:
        if event.button() == QtCore.Qt.LeftButton:
            self.lmbPressed = False
            self.select_tiles()
            self.scene_update()
            self.parentWidget().parent().selectionUpdate()
        else:
            self.rmbPressed = False

    def drawBackground(self, painter: QtGui.QPainter, rect: QtCore.QRectF) -> None:

        self.painter.fill_background(painter, 'lightGray', self.size().width(),
                                     self.size().height())
        if not self.is_to_png:
            self.change_tiles_handler(self.highlight_select_tile,
                                      {"painter": painter})
        self.scene_update()

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

    def save_to_png(self, file_name: str):
        self.coordinates_transformer.set_scale(1)
        self.change_object_handler(self.scaled_obj, {"scale": 1})
        self.is_to_png = True
        self.scene_update()
        pixmap = self.grab(QRect(QPoint(0, 0),
                                 QPoint((self.map.gridSize + 1) * get_map_width(self.get_tiles()),
                                        (self.map.gridSize + 1) * get_map_height(self.get_tiles()))))
        pixmap.save(f"{file_name}.png")
        self.is_to_png = False
        self.coordinates_transformer.set_scale(self.scale)
        self.change_object_handler(self.scaled_obj, {"scale": self.scale})
        self.scene_update()

    def create_new_map(self, info: Dict[str, Any], path: Path):
        width, height = int(info['x']), int(info['y'])
        tile_size = float(info['tile_size'])
        self.map.gridSize = tile_size * 100
        self.scale = 1
        self.coordinates_transformer.set_scale(self.scale)
        self.open_map(path, self.map.map.name, True, (width, height))

    def create_default_map_content(self, size: tuple):
        width, height = size
        self.add_frame_on_map("map_1")
        for i in range(width):
            for j in range(height):
                #TODO map_1
                new_tile_name = f"map_1/tile_{i}_{j}"
                self.add_obj_on_map("tiles", new_tile_name)
                self.handlers.handle(MoveObjCommand(new_tile_name, (float(i), float(j))))
                self.handlers.handle(MoveTileCommand(new_tile_name, (i, j)))
        
    def open_map(self, path: Path, map_name: str, is_new_map: bool = False,
                 size: tuple = (0, 0)):
        self.delete_objects()
        self.map.load_map(MapDescription(path, map_name))
        self.init_handlers()
        if is_new_map:
            self.create_default_map_content(size)
        self.init_objects()
        self.change_object_handler(self.scaled_obj, {"scale": self.scale})
        self.set_map_size()
        self.scene_update()
