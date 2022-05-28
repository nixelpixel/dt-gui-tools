from __future__ import annotations
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any, Optional

from classes.Commands.RenderCommand import RenderCommand
from classes.MapDescription import MapDescription
from classes.basic.chain import AbstractHandler
from classes.basic.command import Command
from classes.layers import AbstractLayer
from mapStorage import MapStorage


class TileLayer(AbstractHandler, AbstractLayer):
    def __init__(self, *kwargs) -> None:
        super(TileLayer, self).__init__(*kwargs)

    def handle(self, command: Command) -> None:
        command.execute()
        return super().handle(command)

    def render(self):
        pass


class WatchtowersLayer(AbstractHandler, AbstractLayer):
    def __init__(self, *kwargs) -> None:
        super(WatchtowersLayer, self).__init__(*kwargs)

    def handle(self, command: Command) -> None:
        command.execute()
        return super().handle(command)


if __name__ == '__main__':
    MapStorage(MapDescription(Path("./maps/tm1"), "test"))
    tile_layer = TileLayer("tiles")
    watchtower_layer = WatchtowersLayer("watchtowers")
    tile_layer.set_next(watchtower_layer)
    layer = tile_layer

    # while layer:
    layer.handle(RenderCommand())
