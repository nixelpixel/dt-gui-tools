from dt_maps import Map

DEFAULT_TILE_SIZE = 0.585


class DataClass:
    # make it singleton
    drawState = ''
    copyBuffer = [[]]

    def __init__(self, args):
        self.distortion_view_one_string_mode = True
        self.region_create = False
        self.active_items = []
        self.active_group = None
        self.name_of_editable_obj = None
        # Main object for api to map
        self.dm = Map.from_disk("test", "./maps/tm1")
        self.tile_size = DEFAULT_TILE_SIZE

        # Set locale
        self.locale = args.locale

        # Set debug mode
        self.debug_mode = args.debug

