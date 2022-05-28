from dt_maps import Map
from pathlib import Path
from typing import Optional
from utils.singletonMeta import SingletonMeta
from classes.MapDescription import MapDescription


class MapStorage(metaclass=SingletonMeta):
    """Only this class contains object Map from dt_maps """
    map: Map = None

    def __init__(self, map_desc: Optional[MapDescription] = None) -> None:
        if map_desc:
            self.change_map(map_desc)

    def change_map(self, m: MapDescription) -> None:
        self.map = Map.from_disk(m.map_name, str(m.folder))


if __name__ == '__main__':
    m = MapStorage(MapDescription(Path("./maps/tm1"), "test"))
    print(m.map.name)
    n = MapStorage()
    print(n.map.name)
