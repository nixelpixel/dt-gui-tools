from PyQt5 import QtGui
from PyQt5.QtWidgets import QLabel
from typing import Dict, Any


class DebugLine(QLabel):

    def __init__(self, text: str = ""):
        super().__init__()
        self.text = text
        self.set_text(self.text)

    def set_text(self, text: str):
        self.text = text
        self.setText(self.text)
        self.show()

    def set_mouse_pos(self, event: Dict[str, Any]) -> None:
        self.setText(f"Cursor position: x={event['pos'][0]}, y={event['pos'][1]}")
