from threading import Lock
DEFAULT_TILE_SIZE = 0.585


class EditorStateMeta(type):
    _instances = {}
    _lock: Lock = Lock()

    def __call__(cls, *args, **kwargs):
        with cls._lock:
            if cls not in cls._instances:
                instance = super().__call__(*args, **kwargs)
                cls._instances[cls] = instance
        return cls._instances[cls]


class EditorState(metaclass=EditorStateMeta):
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
