from CoordinatorTransformer import CoordinatorTransformer
from LayersAction import LayersAction
from QtWindowAPI import QtWindowAPI
from mapStorage import MapStorage
from painter import Painter
from utils.maps import default_map_storage


class MapAPI:
    """High level API. MapAPI ~ Backend"""
    _coordinator_transformer: CoordinatorTransformer = None
    _view_api: QtWindowAPI = None
    _map_storage: MapStorage = None
    _painter: Painter = None
    _layers_action: LayersAction = None

    def __init__(self,
                 coordinator_transformer: CoordinatorTransformer,
                 view_api: QtWindowAPI,
                 map_storage: MapStorage,
                 painter: Painter,
                 layers_action: LayersAction) -> None:
        self._coordinator_transformer = coordinator_transformer or CoordinatorTransformer()
        self._map_storage = map_storage or default_map_storage()
        self._view_api = view_api
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
