# -*- coding: utf-8 -*-

from PyQt5 import QtWidgets

from layers import TileLayer, WatchtowersLayer
from mapStorage import MapStorage
from coordinatesTransformer import CoordinatesTransformer


class MapViewer(QtWidgets.QGraphicsView, QtWidgets.QWidget):
    map = None
    tiles = None
    watchtowers = None
    #citizens = None
    #traffic_signs = None
    #ground_tags = None
    #vehicles = None
    #decorations = None

    def __init__(self):
        QtWidgets.QGraphicsView.__init__(self)
        self.setScene(QtWidgets.QGraphicsScene())

        self.coordinates_transformer = CoordinatesTransformer()
        self.map = MapStorage()
        #self.tiles = TileLayer()
        #self.watchtowers = WatchtowersLayer()
        #self.citizens = CitizensHandler()
        #self.traffic_signs = TrafficSignsHandler()
        #self.ground_tags = GroundTagsHandler()
        #self.vehicles = VehiclesHandler()
        #self.decorations = DecorationsHandler()

        #handlers_list = [self.tiles, self.watchtowers]
        #for i in range(len(handlers_list) - 1):
        #    handlers_list[i].set_next(handlers_list[i+1])
