from abc import ABC

from classes.basic.command import Command
from mapStorage import MapStorage


class DtMapCommand(Command, ABC):

    def __init__(self):
        self.dm = MapStorage()  # Duckietown MapStorage object

