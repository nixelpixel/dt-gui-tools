from duckietown_world.structure.duckietown_map import DuckietownMap
from duckietown_world.structure.map_factory import MapFactory
import os
DT_WORLD = None


def get_dt_world(map_name=None) -> DuckietownMap:
    global DT_WORLD
    if not DT_WORLD:
        #if not map_name:
        #    DT_WORLD = MapFactory.load_map(map_name)
        DT_WORLD = MapFactory.load_map(os.path.abspath("maps/tm1"))
    print('MAP FACTORY', DT_WORLD, map_name)
    return DT_WORLD
