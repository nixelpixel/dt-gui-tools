from utils.singletonMeta import SingletonMeta

DEFAULT_TILE_SIZE = 0.585


class EditorState(metaclass=SingletonMeta):
    drawState = ''
    copyBuffer = [[]]
    debug_mode = False

    def __init__(self) -> None:
        self.distortion_view_one_string_mode = True
        self.region_create = False
        self.active_items = []
        self.active_group = None
        self.name_of_editable_obj = None
        self.tile_size = DEFAULT_TILE_SIZE
        self.is_move = False

    def is_debug(self) -> bool:
        return self.debug_mode

    def set_move(self, val: bool) -> None:
        self.is_move = val
