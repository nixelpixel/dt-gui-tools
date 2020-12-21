import duckietown_world.structure_2 as st
from duckietown_world.world_duckietown.duckietown_map import DuckietownMap
DT_WORLD = None


def get_dt_world(map_name=None) -> DuckietownMap:
    global DT_WORLD
    if not DT_WORLD:
        if not map_name:
            DT_WORLD = st.DuckietownMap.deserialize(map_name)
        DT_WORLD = st.DuckietownMap.deserialize("maps/tm1")
    return DT_WORLD
