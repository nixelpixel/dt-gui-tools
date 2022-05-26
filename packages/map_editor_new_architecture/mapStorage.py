from dt_maps import Map
from map_editor_new_architecture.utils.singletonMeta import SingletonMeta


class MapStorage(metaclass=SingletonMeta):
    map = None

    def __init__(self) -> None:
        self.map = Map.from_disk("test", "./maps/tm1")
