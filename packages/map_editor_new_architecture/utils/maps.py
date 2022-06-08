from pathlib import Path

from classes.MapDescription import MapDescription
from mapStorage import MapStorage


def default_map_storage() -> MapStorage:
    return MapStorage(MapDescription(Path("./maps/tm1"), "test"))  # TODO: need to open empty map


if __name__ == '__main__':
    m = default_map_storage()
    print(m.map.name)
