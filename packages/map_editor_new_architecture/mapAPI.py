from coordinateTransformer import CoordinateTransformer
from layersAction import LayersAction
from qtWindowAPI import QtWindowAPI
from mapStorage import MapStorage
from painter import Painter


class MapAPI:
    """High level API. MapAPI ~ Backend"""
    _coordinate_transformer: CoordinateTransformer = None
    _view_qt_api: QtWindowAPI = None
    _map_storage: MapStorage = None
    _painter: Painter = None
    _layers_action: LayersAction = None

    def __init__(self,
                 coordinator_transformer: CoordinateTransformer,
                 view_qt_api: QtWindowAPI,
                 map_storage: MapStorage,
                 painter: Painter,
                 layers_action: LayersAction) -> None:
        self._coordinate_transformer = coordinator_transformer or CoordinateTransformer()
        self._map_storage = map_storage or MapStorage()
        self._view_qt_api = view_qt_api
        self._painter = painter
        self._layers_action = layers_action

    def load(self):
        """
        pseudocode
        map_path = QT.GET_PATH
        MapStorage.change map
        :return:
        """
        pass

    def save(self):
        """save map"""
        pass

    def render(self):
        """render map"""

    def move(self):
        pass
