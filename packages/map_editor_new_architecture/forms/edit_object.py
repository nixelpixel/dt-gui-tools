from PyQt5 import QtCore
from typing import Dict, Any
from PyQt5.QtWidgets import QDialog, QGroupBox, QDialogButtonBox, QFormLayout, QVBoxLayout, \
    QLineEdit, QLabel


class EditObject(QDialog):
    get_info = QtCore.pyqtSignal(object)

    def __init__(self, layer_name: str, name: str, config: Dict[str, Any],
                 map_pose: tuple, is_draggable: bool, yaw: int):
        super(EditObject, self).__init__()
        self.info = {"types": {}}
        self.info_send = {"name": name, "layer_name": layer_name,
                          "new_config": {}, "is_draggable": is_draggable,
                          "frame": {"pos_x": 0.0, "pos_y": 0.0, "yaw": 0,
                                    "remove": "no"},
                          "is_valid": True
                          }
        self.name = name
        self.is_draggable = is_draggable
        self.info_send["frame"]["pos_x"] = map_pose[0]
        self.info_send["frame"]["pos_y"] = map_pose[1]
        self.info_send["frame"]["yaw"] = yaw
        self.info_send["new_config"] = config
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

    def send_info(self) -> None:
        try:
            for key in self.info_send["frame"]:
                if self.is_draggable or key == "yaw"  or key == "remove":
                    self.info_send["frame"][key] = (self.info["types"][key])(self.info[key].text())
            for key in self.info_send["new_config"]:
                self.info_send["new_config"][key] = (self.info["types"][key])(self.info[key].text())
        except ValueError:
            self.info_send["is_valid"] = False
        self.get_info.emit(self.info_send)
        self.close()

    def create_form(self, config: Dict[str, Any]) -> None:
        layout = QFormLayout()
        for key in self.info_send["frame"]:
            self.info["types"][key] = type(self.info_send["frame"][key])
            if self.is_draggable or key == "yaw" or key == "remove":
                edit = QLineEdit(self)
                self.info[key] = edit
                edit.setText(str(self.info_send["frame"][key]))
                layout.addRow(QLabel(key), edit)
        for key in config:
            edit = QLineEdit(self)
            self.info[key] = edit
            self.info["types"][key] = type(config[key])
            edit.setText(str(config[key]))
            layout.addRow(QLabel(key), edit)
        self.formGroupBox.setLayout(layout)
