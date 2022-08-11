from PyQt5 import QtWidgets
from PyQt5.QtWidgets import *
from PyQt5 import QtCore
import sys

class Collect_Info(QDialog):
	send_info = QtCore.pyqtSignal(object)
	def __init__(self):
		super(Collect_Info, self).__init__()

		self.setWindowTitle("Init information")
		self.setGeometry(100, 100, 500, 550)
		self.formGroupBox = QGroupBox("Collect Data")
		self.size_width = QSpinBox()
		self.size_height = QSpinBox()
		self.tile_width = QLineEdit()
		self.tile_width.setText("0.585")
		self.tile_height = QLineEdit()
		self.tile_height.setText("0.585")

		self.crossroad_count_triple = QSpinBox()
		self.crossroad_count_quad = QSpinBox()

		self.traffic_signs = QSpinBox()
		self.ground_tags = QSpinBox()
		self.citizens = QSpinBox()
		self.vehicles = QSpinBox()

		# self.road_length = QSpinBox()
		self.map_name = QLineEdit()
		self.map_name.setText('map_1')
		self.save_path = QLineEdit()

		self.save_path.setText("./maps/map1/")

		self.watchtowers = QCheckBox()
		self.createForm()
		self.buttonBox = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)

		self.buttonBox.accepted.connect(self.checkCorrectnessOfData)
		self.buttonBox.rejected.connect(self.reject)

		mainLayout = QVBoxLayout()
		mainLayout.addWidget(self.formGroupBox)
		mainLayout.addWidget(self.buttonBox)
		self.setLayout(mainLayout)

	def showError(self, message):
		self.error_dialog = QtWidgets.QErrorMessage()
		self.error_dialog.showMessage(message)


	def checkCorrectnessOfData(self):
		if int(self.size_width.text()) < 3:
			self.showError('Width cannot be less than 3')
			self.reject()
		elif int(self.size_height.text()) < 3:
			self.showError('Height cannot be less than 3')
			self.reject()
		elif float(self.tile_width.text()) <= 0:
			self.showError('Tile width cannot be less or equal to 0')
			self.reject()
		elif float(self.tile_height.text()) <= 0:
			self.showError('Tile height cannot be less or equal to 0')
			self.reject()
		else:
			self.getInfo()

	def getInfo(self):

		# printing the form information
		print("Width : {0}".format(self.size_width.text()))
		print("Height : {0}".format(self.size_height.text()))
		print("Tile Width : {0}".format(self.tile_width.text()))
		print("Tile Height : {0}".format(self.tile_height.text()))
		# print("Road length : {0}".format(self.road_length.text()))
		print("Path : {0}".format(self.save_path.text()))
		print("Watchtowers: {0}".format(self.watchtowers.isChecked()))
		info = {
			'x': int(self.size_width.text()),
			'y': int(self.size_height.text()),
			'width': int(self.size_width.text()),
			'height': int(self.size_height.text()),
			'length': 10,
			'path': self.save_path.text(),
			'crossroads_data': {
				'triple': int(self.crossroad_count_triple.text()),
				'quad': int(self.crossroad_count_quad.text()),
			},
			'map_name': self.map_name,
			'tile_width': float(self.tile_width.text()),
			'tile_height': float(self.tile_height.text()),
			'dir_name': self.save_path.text(),
			'traffic_signs': int(self.traffic_signs.text()),
			'ground_tags': int(self.ground_tags.text()),
			'citizens': int(self.citizens.text()),
			'vehicles': int(self.vehicles.text()),
			'watchtowers': self.watchtowers.isChecked(),
		}
		self.send_info.emit(info)
		self.close()

	def createForm(self):

		layout = QFormLayout()

		layout.addRow(QLabel("Width"), self.size_width)
		layout.addRow(QLabel("Height"), self.size_height)
		layout.addRow(QLabel("Tile_Width"), self.tile_width)
		layout.addRow(QLabel("Tile Height"), self.tile_height)
		layout.addRow(QLabel("Triple crossroad count"), self.crossroad_count_triple)
		layout.addRow(QLabel("Quad crossroad count"), self.crossroad_count_quad)
		layout.addRow(QLabel("Traffic_signs"), self.traffic_signs)
		layout.addRow(QLabel("Ground_tags"), self.ground_tags)
		layout.addRow(QLabel("Citizens"), self.citizens)
		layout.addRow(QLabel("Vehicles"), self.vehicles)
		layout.addRow(QLabel("Map Name"), self.map_name)

		# layout.addRow(QLabel("Road length"), self.road_length)
		layout.addRow(QLabel("Path"), self.save_path)
		layout.addRow(QLabel("Generate watchtowers"), self.watchtowers)
		self.formGroupBox.setLayout(layout)


# main method
if __name__ == '__main__':

	app = QApplication(sys.argv)
	window = Collect_Info()
	window.show()
	sys.exit(app.exec())
