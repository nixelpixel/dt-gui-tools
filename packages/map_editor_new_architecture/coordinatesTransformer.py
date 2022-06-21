class CoordinatesTransformer:
    """coordinate of mouse, pressevent, moveevent, offset,"""
    def __init__(self, scale: float, size_map: int, grid_size: float):
        self.scale = scale
        self.size_map = size_map
        self.grid_size = grid_size

    def get_x_from_view(self, x_view: float, offset_x: float = 0.0, obj_width: float = 0.0) -> float:
        return (x_view - obj_width + offset_x) / self.scale / self.grid_size

    def get_y_from_view(self, y_view: float, offset_y: float = 0.0, obj_height: float = 0.0) -> float:
        return self.size_map - (y_view + obj_height + offset_y) / self.scale / self.grid_size

    def get_x_to_view(self, x_real: float) -> float:
        return x_real * self.scale * (self.grid_size + 1)

    def get_y_to_view(self, y_real: float) -> float:
        return (self.size_map - y_real - 1) * self.scale * (self.grid_size + 1)

    def set_scale(self, new_scale: float) -> None:
        self.scale = new_scale

    def set_size_map(self, new_size_map: int) -> None:
        self.size_map = new_size_map

    def set_grid_size(self, new_grid_size: float) -> None:
        self.grid_size = new_grid_size
