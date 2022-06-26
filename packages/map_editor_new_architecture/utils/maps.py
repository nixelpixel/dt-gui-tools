import os
from pathlib import Path
from classes.MapDescription import MapDescription
from mapStorage import MapStorage
from dt_maps import Map, MapLayer
from dt_maps.types.frames import Frame
from dt_maps.types.watchtowers import Watchtower


def default_map_storage() -> MapStorage:
    return MapStorage(MapDescription(Path("./maps/tm1"), "test"))  # TODO: need to open empty map


def add_new_obj(dm: Map, layer: MapLayer, layer_name: str, obj_name: str, default_conf: dict) -> None:
    layer[obj_name] = default_conf
    layer = MapLayer(dm, layer_name, layer)
    dm._layers.__dict__[layer_name] = layer
    register = lambda l, t: dm.layers.get(l).register_entity_helper(
        t) if dm.layers.has(l) else 0
    if layer_name == "frames":
        register("frames", Frame)
    elif layer_name == "watchtowers":
        register("watchtowers", Watchtower)


def delete_obj(dm: Map, layer: MapLayer, layer_name: str, obj_name: str) -> None:
    layer.__delitem__(obj_name)
    layer = MapLayer(dm, layer_name, layer)
    dm._layers.__dict__[layer_name] = layer
    register = lambda l, t: dm.layers.get(l).register_entity_helper(
        t) if dm.layers.has(l) else 0
    if layer_name == "frames":
        register("frames", Frame)
    elif layer_name == "watchtowers":
        register("watchtowers", Watchtower)


def change_map_directory(dm: Map, new_dir: str) -> None:
    dm._path = new_dir
    dm._assets_dir = os.path.join(dm._path, "assets")

if __name__ == '__main__':
    m = default_map_storage()
    print(m.map.name)
