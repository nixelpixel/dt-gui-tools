from PyQt5 import QtCore
from typing import Dict, Any
from PyQt5.QtWidgets import QDialog, QGroupBox, QDialogButtonBox, QFormLayout, QVBoxLayout, \
    QLineEdit, QLabel


class EditObject(QDialog):
    get_info = QtCore.pyqtSignal(object)

    def __init__(self, name: str, config: Dict[str, Any]):
        super(EditObject, self).__init__()
        self.info = {}
        self.setWindowTitle("Edit object")
        self.formGroupBox = QGroupBox(f"Object: {name}")
        self.buttonBox = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self.buttonBox.accepted.connect(self.send_info)
        self.buttonBox.rejected.connect(self.reject)
        main_layout = QVBoxLayout(self)
        main_layout.addWidget(self.formGroupBox)
        main_layout.addWidget(self.buttonBox)
        self.create_form(config)
        self.setLayout(main_layout)

    def send_info(self):
        send_info = {}
        for key in self.info:
            send_info[key] = self.info[key].text()
        self.get_info.emit(send_info)
        self.close()

    def create_form(self, config: Dict[str, Any]):
        layout = QFormLayout()
        for key in config:
            edit = QLineEdit(self)
            edit.setText(str(config[key]))
            self.info[key] = edit
            layout.addRow(QLabel(key), edit)
        self.formGroupBox.setLayout(layout)
