from threading import Lock

from utils.singletonMeta import SingletonMeta

DEFAULT_TILE_SIZE = 0.585


class EditorState(metaclass=SingletonMeta):
    drawState = ''
    copyBuffer = [[]]

    def __init__(self, args) -> None:
        self.distortion_view_one_string_mode = True
        self.region_create = False
        self.active_items = []
        self.active_group = None
        self.name_of_editable_obj = None
        self.tile_size = DEFAULT_TILE_SIZE

        # Set locale
        self.locale = args.locale

        # Set debug mode
        self.debug_mode = args.debug
