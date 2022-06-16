
class CoordinatesTransformer:
    """coordinate of mouse, pressevent, moveevent, offset,"""
    # TODO set values on init

    @staticmethod
    def get_x_from_view(x_view: float, scale: float, grid_size: float, obj_width: float) -> float:
        return (x_view - obj_width) / scale / grid_size

    @staticmethod
    def get_y_from_view(y_view: float, scale: float, grid_size: float, size_map: int, obj_height: float = 0.0) -> float:
        return size_map - (y_view + obj_height) / scale / grid_size

    @staticmethod
    def get_x_to_view(x_real: float, scale: float, grid_size: float) -> float:
        return x_real * scale * (grid_size + 1)

    @staticmethod
    def get_y_to_view(y_real: float, scale: float, grid_size: float, size_map: int) -> float:
        return (size_map - y_real - 1) * scale * (grid_size + 1)

