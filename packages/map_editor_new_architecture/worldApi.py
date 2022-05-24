class WorldApi:
    pass
    # 'static class'
'''    
    @staticmethod
    def get_x_from_view(self, x_view: float) -> float:
        logger.debug((x_view - self.offsetX) / self.sc / self.map.gridSize * self.tile_size)
        print('X F V', x_view - self.offsetX,  x_view)
        print((x_view - self.offsetX) / self.sc / self.map.gridSize * self.tile_size)
        #return self.i_tile * self.tile_size - (x_view - self.offsetX) / self.sc / self.map.gridSize * self.tile_size
        return (x_view - self.offsetX) / self.sc / self.map.gridSize * self.tile_size

    @staticmethod
    def get_y_from_view(self, y_view: float) -> float:
        logger.debug((self.size_map - (y_view - self.offsetY) / self.sc / self.map.gridSize) \
                     * self.tile_size)
        return (self.size_map - (y_view - self.offsetY) / self.sc / self.map.gridSize) \
               * self.tile_size
               
    @staticmethod
    def get_x_to_view(self, x_real: float) -> float:
        print('LEN -', self.i_tile * self.tile_size)
        print(x_real)
        print((x_real + 0) * self.sc * self.map.gridSize / self.tile_size)

        #return (self.i_tile * self.tile_size - x_real + 0) * self.sc * self.map.gridSize / self.tile_size
        return (x_real + 0) * self.sc * self.map.gridSize / self.tile_size
        
    @staticmethod
    def get_y_to_view(self, y_real: float) -> float:
        return ((self.size_map - y_real / self.tile_size) + 0) * self.sc * self.map.gridSize
    '''