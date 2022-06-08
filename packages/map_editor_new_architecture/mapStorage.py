from dt_maps import Map
from pathlib import Path
from typing import Optional
from utils.singletonMeta import SingletonMeta
from classes.MapDescription import MapDescription


class MapStorage(metaclass=SingletonMeta):
    """Only this class contains object Map from dt_maps """
    map: Map = None
    gridSize = 58.5

    def __init__(self, map_description: Optional[MapDescription] = None) -> None:
        if map_description:
            self.change_map(map_description)

    def change_map(self, new_map_description: MapDescription) -> None:
        self.map = Map.from_disk(new_map_description.map_name, str(new_map_description.folder))


if __name__ == '__main__':
    m = MapStorage(MapDescription(Path("./maps/tm1"), "test"))
    print(m.map.name)
    n = MapStorage()
    print(n.map.name)
