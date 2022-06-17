from coordinatesTransformer import CoordinatesTransformer
from editorState import EditorState
from layersAction import LayersAction
from qtWindowAPI import QtWindowAPI
from mapStorage import MapStorage
from painter import Painter
from mapViewer import MapViewer

TILE_TYPES = ('block', 'road')


class MapAPI:
    """High level API. MapAPI ~ Backend"""
    _qt_api: QtWindowAPI = None
    _map_storage: MapStorage = None
    _layers_action: LayersAction = None
    _map_viewer: MapViewer = None
    _editor_state: EditorState = None

    def __init__(self, info_json: dict, map_viewer: MapViewer) -> None:
        self._map_storage = MapStorage()
        self._qt_api = QtWindowAPI()
        self._layers_action = LayersAction()
        self.info_json = info_json
        self._map_viewer = map_viewer
        self._editor_state = EditorState()

    def open_map_triggered(self):
        print('open map')

    def import_old_format(self):
        print('import old format')

    #  Open map
    def create_map_triggered(self):
       print('create_map_triggered')

    def create_region(self):
        print('create_region')

    def change_distortion_view_triggered(self):
        pass

    #  Save map
    def save_map_triggered(self):
        self._map_storage.map.to_disk()
        print('save map')

    #  Save map as
    def save_map_as_triggered(self):
        print('save_map_as_triggered')

    #  Calculate map characteristics
    def calc_param_triggered(self):
        print('calc_param_triggered')

    #  Help: About
    def about_author_triggered(self):
        print('about_author_triggered')

    #  Exit
    def exit_triggered(self):
        print('exit_triggered')

    # Save map before exit
    def save_before_exit(self):
        pass

    #  Hide Block menu
    def change_blocks_toggled(self):
        pass

    #  Change button state
    def blocks_event(self, event):
        pass

    #  Hide information menu
    def change_info_toggled(self):
        pass

    #  Change button state
    def info_event(self, event):
        pass

    #  Hide the menu about map properties
    def change_map_toggled(self):
        pass

    #  Change button state
    def map_event(self, event):
        pass

    # Layer window

    def toggle_layer_window(self):
        pass

    def close_layer_window_event(self, event):
        pass

    def layer_tree_clicked(self):
        pass

    def layer_tree_double_clicked(self):
        pass

    def update_layer_tree(self):
        pass

    #  Program exit event
    def quit_program_event(self, event):
        pass

    #  Handle a click on an item from a list to a list
    def item_list_clicked(self):
        pass

    #  Double click initiates as single click action
    def item_list_double_clicked(self, item_name: str, item_type: str) -> None:
        print(item_name, item_type)
        if item_name == "separator":
            pass
        elif item_type in TILE_TYPES:
            pass
        else:
            print(item_name, item_type)
            type_of_element = self.info_json['info'][item_name]['type']
            self._map_viewer.add_obj(item_name, type_of_element)

    #  Reset to default values
    def set_default_fill(self):
        pass

    #  Copy
    def copy_button_clicked(self):
        pass

    #  Cut
    def cut_button_clicked(self):
        pass

    #  Paste
    def insert_button_clicked(self):
        pass

    #  Delete
    def delete_button_clicked(self):
        pass

    #  Undo
    def undo_button_clicked(self):
        pass

    #  Brush mode
    def brush_mode(self, brush_button_is_checked: bool):
        if brush_button_is_checked:
            self._editor_state.drawState = 'brush'
        else:
            self._editor_state.drawState = ''

    def trimClicked(self):
        pass

    def selection_update(self, default_fill: str) -> None:
        if self._editor_state.drawState == 'brush':
            self._map_viewer.painting_tiles(default_fill)

    def key_press_event(self, e):
        self._qt_api.key_press_event(e)

    def rotate_selected_tiles(self) -> None:
        self._map_viewer.rotate_tiles()
