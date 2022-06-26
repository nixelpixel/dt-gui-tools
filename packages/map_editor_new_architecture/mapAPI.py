from pathlib import Path

from PyQt5 import QtWidgets

from classes.MapDescription import MapDescription
from editorState import EditorState
from utils.maps import change_map_directory
from utils.qtWindowAPI import QtWindowAPI
from mapStorage import MapStorage
from mapViewer import MapViewer
from utils.debug import DebugLine
from typing import Dict, Any


TILE_TYPES = ('block', 'road')


class MapAPI:
    """High level API. MapAPI ~ Backend"""
    _qt_api: QtWindowAPI = None
    _map_storage: MapStorage = None
    _map_viewer: MapViewer = None
    _editor_state: EditorState = None
    _debug_line: DebugLine = None

    def __init__(self, info_json: dict, map_viewer: MapViewer) -> None:
        self._map_storage = MapStorage()
        self._qt_api = QtWindowAPI()
        self.info_json = info_json
        self._map_viewer = map_viewer
        self._editor_state = EditorState()

    def open_map_triggered(self, parent: QtWidgets.QWidget):
        path = self._qt_api.get_dir(parent, "open")
        if path:
            self._map_viewer.delete_objects()
            self._map_storage.load_map(MapDescription(path,
                                                      self._map_storage.map.name))
            self._map_viewer.init_handlers()
            self._map_viewer.init_objects()

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
    def save_map_as_triggered(self, parent: QtWidgets.QWidget):
        path = self._qt_api.get_dir(parent, "save")
        if path:
            change_map_directory(self._map_storage.map, path)
            self.save_map_triggered()

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
    def brush_mode(self, brush_button_is_checked: bool) -> None:
        if brush_button_is_checked:
            self._editor_state.drawState = 'brush'
        else:
            self._editor_state.drawState = ''

    def trimClicked(self):
        pass

    def selection_update(self, default_fill: str) -> None:
        if self._editor_state.drawState == 'brush':
            self._map_viewer.painting_tiles(default_fill)

    def key_press_event(self, e) -> None:
        pass

    def rotate_selected_tiles(self) -> None:
        self._map_viewer.rotate_tiles()

    def set_debug_mode(self, debug_line: DebugLine) -> None:
        self._editor_state.debug_mode = True
        self._debug_line = debug_line

    def update_debug_info(self, event: Dict[str, Any]) -> None:
        if self._editor_state.debug_mode:
            if event["mode"] == "set_cursor_pos":
                self._debug_line.set_mouse_pos(event)
