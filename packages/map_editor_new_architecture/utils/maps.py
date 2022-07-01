import os
from pathlib import Path

from dt_maps.types.tiles import Tile

from classes.MapDescription import MapDescription
from mapStorage import MapStorage
from dt_maps import Map, MapLayer
from dt_maps.types.frames import Frame
from dt_maps.types.watchtowers import Watchtower
from typing import Dict, Any

REGISTER = {
    "frames": Frame,
    "watchtowers": Watchtower,
    "tiles": Tile,
}


def default_map_storage() -> MapStorage:
    return MapStorage(MapDescription(Path("./maps/tm1"), "test"))  # TODO: need to open empty map


def add_new_obj(dm: Map,
                layer: MapLayer,
                layer_name: str, obj_name: str, default_conf: dict) -> None:
    layer[obj_name] = default_conf
    layer = MapLayer(dm, layer_name, layer)
    dm._layers.__dict__[layer_name] = layer
    register = lambda l, t: dm.layers.get(l).register_entity_helper(
        t) if dm.layers.has(l) else 0
    register(layer_name, REGISTER[layer_name])


def delete_obj(dm: Map, layer: MapLayer,
               layer_name: str, obj_name: str) -> None:
    layer.__delitem__(obj_name)
    layer = MapLayer(dm, layer_name, layer)
    dm._layers.__dict__[layer_name] = layer
    register = lambda l, t: dm.layers.get(l).register_entity_helper(
        t) if dm.layers.has(l) else 0
    register(layer_name, REGISTER[layer_name])


def change_map_directory(dm: Map, new_dir: str) -> None:
    dm._path = new_dir
    dm._assets_dir = os.path.join(dm._path, "assets")


def get_map_height(tiles: Dict[str, Any]):
    return get_map_size(tiles, "j")
    

def get_map_width(tiles: Dict[str, Any]):
    return get_map_size(tiles, "i")


def get_map_size(tiles: Dict[str, Any], side: str) -> int:
    elems = []
    for tile_name in tiles:
        elems.append(tiles[tile_name][side])
    if len(elems) > 0:
        return max(elems) + 1
    else:
        return 0


if __name__ == '__main__':
    m = default_map_storage()
    print(m.map.name)
