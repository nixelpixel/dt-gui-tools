from dt_maps import Map
from threading import Lock


class MapStorageMeta(type):
    _instances = {}
    _lock: Lock = Lock()

    def __call__(cls, *args, **kwargs):
        with cls._lock:
            if cls not in cls._instances:
                instance = super().__call__(*args, **kwargs)
                cls._instances[cls] = instance
        return cls._instances[cls]


class MapStorage(metaclass=MapStorageMeta):
    map = None

    def __init__(self) -> None:
        self.map = Map.from_disk("test", "./maps/tm1")
