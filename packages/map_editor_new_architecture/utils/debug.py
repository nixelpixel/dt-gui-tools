from PyQt5.QtWidgets import QLabel


class DebugLine(QLabel):

    def __init__(self, text: str):
        super().__init__()
        self.text = ""
        self.set_text(text)

    def set_text(self, text: str):
        self.text = text
        self.setText(self.text)
        self.show()
