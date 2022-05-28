from typing import Any

from dt_maps.types.frames import Frame

from classes.basic.command import Command


class MoveCommand(Command):
    _x: int = 0
    _y: int = 0
    _obj: Frame = None

    def __init__(self, x: int = 0, y: int = 0, obj: Frame = 0):  # only for examples
        self._x = x
        self._y = y
        self._obj = obj

    def execute(self, *args, **kwargs) -> Any:
        pass
