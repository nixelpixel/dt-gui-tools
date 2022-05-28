from typing import Any

from dt_maps import Map, MapLayer

from classes.basic.command import Command


class RenderCommand(Command):
    _render_obj = None

    def __init__(self, render_obj):
        self._render_obj = render_obj

    def execute(self, dm: Map, layer: MapLayer, layer_name: str) -> Any:
        return True
