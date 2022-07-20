from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QFileDialog


class QtWindowAPI:
    def get_dir(self, parent: QtWidgets.QWidget, info: str = "") -> str:
        return QFileDialog.getExistingDirectory(parent, f"Select a folder to {info} the map", '.',
                                                          QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks)

    def create_file_name(self, parent: QtWidgets.QWidget, info: str = "filename") -> str:
        return QFileDialog.getSaveFileName(parent, f"Select {info} to save", '.')[0]
