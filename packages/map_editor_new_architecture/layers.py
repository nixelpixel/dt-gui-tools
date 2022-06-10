from __future__ import annotations
from pathlib import Path
from typing import Any

from mapStorage import MapStorage
from classes.layers import AbstractLayer
from classes.basic.command import Command
from classes.basic.chain import AbstractHandler
from classes.MapDescription import MapDescription
from classes.Commands.GetLayerСommand import GetLayerCommand


class TileLayerHandler(AbstractHandler, AbstractLayer):

    def __init__(self, *kwargs) -> None:
        super(TileLayerHandler, self).__init__(*kwargs)

    def handle(self, command: Command) -> Any:
        response = command.execute(self.dm, self.data, self.layer_name())
        if response:
            return response
        return super().handle(command)

    def layer_name(self) -> str:
        return "tiles"


class WatchtowersLayerHandler(AbstractHandler, AbstractLayer):
    def __init__(self, *kwargs) -> None:
        super(WatchtowersLayerHandler, self).__init__(*kwargs)

    def handle(self, command: Command) -> Any:
        response = command.execute(self.dm, self.data, self.layer_name())
        if response:
            return response
        return super().handle(command)

    def layer_name(self) -> str:
        return "watchtowers"


class FramesLayerHandler(AbstractHandler, AbstractLayer):
    def __init__(self, *kwargs) -> None:
        super(FramesLayerHandler, self).__init__(*kwargs)

    def handle(self, command: Command) -> Any:
        response = command.execute(self.dm, self.data, self.layer_name())
        if response:
            return response
        return super().handle(command)

    def layer_name(self) -> str:
        return "frames"


if __name__ == '__main__':
    MapStorage(MapDescription(Path("./maps/tm1"), "test"))
    tile_layer = TileLayerHandler()
    watchtower_layer = WatchtowersLayerHandler()
    tile_layer.set_next(watchtower_layer)
    layer = tile_layer

    # while layer:
    # layer.handle(RenderCommand())
    print(layer.handle(GetLayerCommand("watchtowers")))
