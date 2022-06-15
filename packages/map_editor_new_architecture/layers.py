from __future__ import annotations
from pathlib import Path
from typing import Any

from mapStorage import MapStorage
from classes.layers import AbstractLayer
from classes.basic.command import Command
from classes.basic.chain import AbstractHandler
from classes.MapDescription import MapDescription
from classes.Commands.GetLayerCommand import GetLayerCommand


class TileLayerHandler(AbstractHandler, AbstractLayer):

    def __init__(self, *kwargs) -> None:
        super(TileLayerHandler, self).__init__(*kwargs)

    def handle(self, command: Command) -> Any:
        response = command.execute(self.dm, self.data, self.layer_name(), self.default_conf())
        if response:
            return response
        return super().handle(command)

    def layer_name(self) -> str:
        return "tiles"

    def default_conf(self):
        return {'i': 0, 'j': 0}

class WatchtowersLayerHandler(AbstractHandler, AbstractLayer):
    def __init__(self, *kwargs) -> None:
        super(WatchtowersLayerHandler, self).__init__(*kwargs)

    def handle(self, command: Command) -> Any:
        response = command.execute(self.dm, self.data, self.layer_name(), self.default_conf())
        if response:
            return response
        return super().handle(command)

    def layer_name(self) -> str:
        return "watchtowers"

    def default_conf(self):
        return {'configuration': 'WT18'}


class FramesLayerHandler(AbstractHandler, AbstractLayer):
    def __init__(self, *kwargs) -> None:
        super(FramesLayerHandler, self).__init__(*kwargs)

    def handle(self, command: Command) -> Any:
        response = command.execute(self.dm, self.data, self.layer_name(), self.default_conf())
        if response:
            return response
        return super().handle(command)

    def layer_name(self) -> str:
        return "frames"

    def default_conf(self):
        return {'pose': {'x': 1, 'y': 1, 'yaw': 0}}


if __name__ == '__main__':
    MapStorage(MapDescription(Path("./maps/tm1"), "test"))
    tile_layer = TileLayerHandler()
    watchtower_layer = WatchtowersLayerHandler()
    tile_layer.set_next(watchtower_layer)
    layer = tile_layer

    # while layer:
    # layer.handle(RenderCommand())
    print(layer.handle(GetLayerCommand("watchtowers")))
