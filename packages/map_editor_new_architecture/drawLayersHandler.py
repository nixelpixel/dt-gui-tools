from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Any, Optional


class Handler(ABC):
    @abstractmethod
    def set_next(self, handler: Handler) -> Handler:
        pass

    @abstractmethod
    def handle(self, request) -> Optional[str]:
        pass


class AbstractHandler(Handler):

    _next_handler: Handler = None

    def set_next(self, handler: Handler) -> Handler:
        self._next_handler = handler
        return handler

    @abstractmethod
    def handle(self, request: Any) -> None:
        if self._next_handler:
            return self._next_handler.handle(request)

        return None


class AbstractLayer(ABC):
    __layer_name__ = ""

    def __init__(self, layer_name: str):
        self.__layer_name__ = layer_name

    def render(self):
        pass


class TileLayer(AbstractHandler, AbstractLayer):
    def __init__(self, **kwargs):
        super(TileLayer, self).__init__(**kwargs)

    def handle(self, request: Any) -> None:
        pass

    def render(self):
        pass


class WatchtowersHandler(AbstractHandler, AbstractLayer):
    def __init__(self, **kwargs):
        super(WatchtowersHandler, self).__init__(**kwargs)

    def handle(self, request: Any) -> None:
        pass


class CitizensHandler(AbstractHandler, AbstractLayer):
    def __init__(self, **kwargs):
        super(CitizensHandler, self).__init__(**kwargs)

    def handle(self, request: Any) -> None:
        pass


class TrafficSignsHandler(AbstractHandler, AbstractLayer):
    def __init__(self, **kwargs):
        super(TrafficSignsHandler, self).__init__(**kwargs)

    def handle(self, request: Any) -> None:
        pass

    def render(self):
        pass


class GroundTagsHandler(AbstractHandler, AbstractLayer):
    def __init__(self, **kwargs):
        super(GroundTagsHandler, self).__init__(**kwargs)

    def handle(self, request: Any) -> None:
        pass


class VehiclesHandler(AbstractHandler, AbstractLayer):
    def __init__(self, **kwargs):
        super(VehiclesHandler, self).__init__(**kwargs)

    def handle(self, request: Any) -> None:
        pass


class DecorationsHandler(AbstractHandler, AbstractLayer):
    def __init__(self, **kwargs):
        super(DecorationsHandler, self).__init__(**kwargs)

    def handle(self, request: Any) -> None:
        pass
