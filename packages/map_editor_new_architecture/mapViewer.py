# -*- coding: utf-8 -*-
import logging
from typing import Tuple, Dict

from PyQt5.QtWidgets import QGraphicsView
from PyQt5 import QtCore, QtGui, QtWidgets
from duckietown_world.structure.utils import get_degree_for_orientation, get_canonical_sign_name


from map_editor_new_architecture.drawLayersHandler import WatchtowersHandler, \
    CitizensHandler, TrafficSignsHandler, GroundTagsHandler, VehiclesHandler, \
    DecorationsHandler, TileLayer
from map_editor_new_architecture.mapStorage import MapStorage
from map_editor_new_architecture.worldApi import WorldApi
from utils import get_list_dir_with_path
from threading import Lock


logger = logging.getLogger('root')

TILES_DIR_PATH = './img/tiles'
OBJECT_DIR_PATHS = ['./img/signs',
                    './img/apriltags',
                    './img/objects']

DELTA_EUCLIDEAN_DISTANCE = .15


# TILE_SIZE = 0.585

class MapViewerMeta(type):
    _instances = {}
    _lock: Lock = Lock()

    def __call__(cls, *args, **kwargs):
        with cls._lock:
            if cls not in cls._instances:
                instance = super().__call__(*args, **kwargs)
                cls._instances[cls] = instance
        return cls._instances[cls]


class MapViewer(QGraphicsView, QtWidgets.QWidget, metaclass=MapViewerMeta):
    map = None
    tileSprites: Dict[str, QtGui.QImage] = {'empty': QtGui.QImage()}
    objects = {get_canonical_sign_name('stop'): QtGui.QImage()}
    offsetX = 0
    offsetY = 0
    sc = 1
    i_tile = 0
    j_tile = 0
    rmbPressed = False
    lmbPressed = False
    drag_mode = False
    drag_obj = None
    rmbPrevPos = [0, 0]
    mouseStartX, mouseStartY = 0, 0
    mouseCurX, mouseCurY = 0, 0
    tile_size = 0.585
    #  Stores the top left and bottom right coordinates of the selected area (including zero size areas) as array indexes
    #  If the selection is outside the array to the left, contains -1
    #  If the selection is outside the array to the right - width / height
    tileSelection = [0] * 4
    selectionChanged = QtCore.pyqtSignal()
    editObjectChanged = QtCore.pyqtSignal(tuple)
    lmbClicked = QtCore.pyqtSignal(int, int)  # click coordinates as an index of the clicked tile
    watchtowers = None
    citizens = None
    traffic_signs = None
    ground_tags = None
    vehicles = None
    decorations = None

    def __init__(self):
        QGraphicsView.__init__(self)
        self.size_map = 7
        self.setScene(QtWidgets.QGraphicsScene())
        self.load_tiles_images()
        self.load_objects_images()

        self.world_api = WorldApi()
        self.map = MapStorage()
        self.tiles = TileLayer()
        self.watchtowers = WatchtowersHandler()
        self.citizens = CitizensHandler()

        self.tiles.set_next(self.watchtowers).set_next(self.citizens)

        #self.traffic_signs = TrafficSignsHandler()
        #self.ground_tags = GroundTagsHandler()
        #self.vehicles = VehiclesHandler()
        #self.decorations = DecorationsHandler()

    def load_tiles_images(self):
        for filename, file_path in get_list_dir_with_path(TILES_DIR_PATH):
            tile_name = filename.split('.')[0]
            self.tileSprites[tile_name] = QtGui.QImage()
            self.tileSprites[tile_name].load(file_path)

    def load_objects_images(self):
        for dir_path in OBJECT_DIR_PATHS:
            for filename, file_path in get_list_dir_with_path(dir_path):
                object_name = filename.split('.')[0]
                self.objects[get_canonical_sign_name(object_name)] = QtGui.QImage()
                self.objects[get_canonical_sign_name(object_name)].load(file_path)

