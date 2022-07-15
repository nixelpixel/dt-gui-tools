from __future__ import annotations
from pathlib import Path
from typing import Any, Dict

from mapStorage import MapStorage
from classes.layers import AbstractLayer
from classes.basic.command import Command
from classes.basic.chain import AbstractHandler
from classes.MapDescription import MapDescription
from classes.Commands.GetLayerCommand import GetLayerCommand
from dt_maps.types.tiles import TileType
from dt_maps.types.watchtowers import WatchtowerType


class TileLayerHandler(AbstractHandler, AbstractLayer):
    def __init__(self, *kwargs) -> None:
        super(TileLayerHandler, self).__init__(*kwargs)

    def handle(self, command: Command) -> Any:
        response = command.execute(self.dm, self.data, self.layer_name(),
                                   self.default_conf(),
                                   check_config=self.check_config)
        if response:
            return response
        return super().handle(command)

    def layer_name(self) -> str:
        return "tiles"

    def default_conf(self) -> Dict[str, Any]:
        return {'i': 0, 'j': 0, 'type': 'floor'}

    def check_config(self, config: Dict[str, Any]) -> bool:
        return isinstance(config['i'], int) and isinstance(config['j'], int) \
            and config['type'] in [t.value for t in TileType]


class WatchtowersLayerHandler(AbstractHandler, AbstractLayer):
    def __init__(self, *kwargs) -> None:
        super(WatchtowersLayerHandler, self).__init__(*kwargs)

    def handle(self, command: Command) -> Any:
        response = command.execute(self.dm, self.data, self.layer_name(),
                                   self.default_conf(),
                                   check_config=self.check_config)
        if response:
            return response
        return super().handle(command)

    def layer_name(self) -> str:
        return "watchtowers"

    def default_conf(self) -> Dict[str, str]:
        return {'configuration': 'WT18'}

    def check_config(self, config: Dict[str, Any]) -> bool:
        return config["configuration"] in [t.value for t in WatchtowerType]


class FramesLayerHandler(AbstractHandler, AbstractLayer):
    def __init__(self, *kwargs) -> None:
        super(FramesLayerHandler, self).__init__(*kwargs)

    def handle(self, command: Command) -> Any:
        response = command.execute(self.dm, self.data, self.layer_name(),
                                   self.default_conf())
        if response:
            return response
        return super().handle(command)

    def layer_name(self) -> str:
        return "frames"

    def default_conf(self) -> Dict[str, Any]:
        return {'pose': {'x': 1, 'y': 1, 'yaw': 0}}


class TileMapsLayerHandler(AbstractHandler, AbstractLayer):
    def __init__(self, *kwargs) -> None:
        super(TileMapsLayerHandler, self).__init__(*kwargs)

    def handle(self, command: Command) -> Any:
        response = command.execute(self.dm, self.data, self.layer_name(),
                                   self.default_conf())
        if response:
            return response
        return super().handle(command)

    def layer_name(self) -> str:
        return "tile_maps"

    def default_conf(self) -> Dict[str, Any]:
        return {'tile_size': {'x': 0.585, 'y': 0.585}}


if __name__ == '__main__':
    MapStorage(MapDescription(Path("./maps/tm1"), "test"))
    tile_layer = TileLayerHandler()
    watchtower_layer = WatchtowersLayerHandler()
    tile_layer.set_next(watchtower_layer)
    layer = tile_layer

    # while layer:
    print(layer.handle(GetLayerCommand("watchtowers")))
