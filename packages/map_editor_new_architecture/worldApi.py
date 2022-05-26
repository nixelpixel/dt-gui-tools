# -*- coding: utf-8 -*-
from mapStorage import MapStorage


class WorldApi:
    map_object = None
    offsetX = 0
    offsetY = 0
    scale = 1
    size_map = 7
    tile_size = 0.585

    def __init__(self):
        self.map_object = MapStorage()
        # get map size fom self.map

    def set_offset_x(self, new_offset_x: int) -> None:
        self.offsetX = new_offset_x

    def set_offset_y(self, new_offset_y: int) -> None:
        self.offsetY = new_offset_y

    def set_sc(self, new_scale: int) -> None:
        self.scale = new_scale

    def set_tile_size(self, new_tile_size: float) -> None:
        self.tile_size = new_tile_size

'''        

    def get_x_from_view(self, x_view: float) -> float:
        logger.debug((x_view - self.offsetX) / self.sc / self.map.gridSize * self.tile_size)
        print('X F V', x_view - self.offsetX,  x_view)
        print((x_view - self.offsetX) / self.sc / self.map.gridSize * self.tile_size)
        # return self.i_tile * self.tile_size - (x_view - self.offsetX) / self.sc / self.map.gridSize * self.tile_size
        return (x_view - self.offsetX) / self.sc / self.map.gridSize * self.tile_size

    def get_y_from_view(self, y_view: float) -> float:
        logger.debug((self.size_map - (y_view - self.offsetY) / self.sc / self.map.gridSize) \
                     * self.tile_size)
        return (self.size_map - (y_view - self.offsetY) / self.sc / self.map.gridSize) \
               * self.tile_size

    def get_x_to_view(self, x_real: float) -> float:
        # print('LEN -', self.i_tile * self.tile_size)
        print(x_real)
        print((x_real + 0) * self.sc * self.map.gridSize / self.tile_size)

        # return (self.i_tile * self.tile_size - x_real + 0) * self.sc * self.map.gridSize / self.tile_size
        return (x_real + 0) * self.sc * self.map.gridSize / self.tile_size

    def get_y_to_view(self, y_real: float) -> float:
        return ((self.size_map - y_real / self.tile_size) + 0) * self.sc * self.map.gridSize

'''