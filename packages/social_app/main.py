import argparse
import os
import cv2
import sys
import rospy
import numpy as np
from cv_bridge import CvBridge
from PyQt5 import QtGui
from PyQt5.QtWidgets import QWidget, QLabel, QApplication, QComboBox, QVBoxLayout, QHBoxLayout, QPushButton, QSplitter
from PyQt5.QtCore import QThread, Qt, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QPixmap
from datetime import datetime
from sensor_msgs.msg import CompressedImage


class ROSSpin(QThread):
    changeROSImage = pyqtSignal(np.ndarray)

    def __init__(self, robot_name: str):
        super().__init__()
        rospy.init_node('ImageCreator', anonymous=False)
        self.bridge = CvBridge()
        self.image_compressed = rospy.Subscriber(f"/{robot_name}/camera_node/image/compressed",
                                                 CompressedImage, self.callback, queue_size=1)

    def callback(self, picture: CompressedImage):
        image = self.bridge.compressed_imgmsg_to_cv2(picture, "bgr8")
        self.changeROSImage.emit(image)


class App(QWidget):

    def __init__(self, robot_name: str):
        super().__init__()
        self.w, self.h = 640, 480
        self.filter_image = None
        self.mask_image = None
        script_path = os.path.dirname(__file__)
        self.script_path = (script_path + "/") if script_path else ""
        self.save_image = None
        # gui components
        self.top_layout = None
        self.middle_layout = None
        self.label_filter = None
        self.label_mask = None
        self.bot_layout = None
        self.ros = None
        # ---
        self.initUI()
        self.init_ROS(robot_name)

    @pyqtSlot(np.ndarray)
    def setImage(self, image: np.ndarray):
        self.callback(image)

    def callback(self, bot_image: np.ndarray):
        bot_image = cv2.resize(bot_image, (self.w, self.h))
        if self.mask_image is not None:
            mask_image = self.convert_cv_qt(self.add_mask(bot_image))
        else:
            mask_image = self.convert_cv_qt(bot_image)
        if self.filter_image is not None:
            self.save_image = self.add_filter(bot_image)
            filter_image = self.convert_cv_qt(self.save_image)
        else:
            filter_image = self.convert_cv_qt(bot_image)
        self.label_mask.setPixmap(mask_image)
        self.label_filter.setPixmap(filter_image)

    def add_mask(self, img):
        return cv2.addWeighted(img, 1, self.mask_image, 1, 0)

    def add_filter(self, img):
        return cv2.addWeighted(img, 1, self.filter_image, 1, 0)

    def convert_cv_qt(self, cv_img):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(self.w, self.h, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)

    def initUI(self):
        self.setWindowTitle("Image Creator From Duckiebot")
        self.setGeometry(300, 300, self.w + 300, self.h + 200)
        self.resize(2 * self.w + 100, int(1.5 * self.h))
        windowLayout = QVBoxLayout()
        self.create_top_layout()
        self.create_middle_layout()
        self.create_bot_layout()
        windowLayout.addLayout(self.top_layout)
        windowLayout.addLayout(self.middle_layout)
        windowLayout.addLayout(self.bot_layout)
        self.setLayout(windowLayout)

    def createComboBox(self):
        combo = QComboBox(self)
        filters = []
        for filter in os.listdir(self.script_path + "filters"):
            if os.path.isdir(self.script_path + 'filters/' + filter):
                filters.append(filter)
        combo.addItems(filters)
        combo.activated[str].connect(self.changed_filter)
        return combo

    def changed_filter(self, text):
        path = self.script_path + 'filters/' + text + "/"
        mask, filter = path + "mask.png", path + "filter.png"
        self.mask_image = cv2.imread(mask, cv2.COLOR_BGR2RGB)
        self.mask_image = cv2.resize(self.mask_image, (self.w, self.h))
        self.filter_image = cv2.imread(filter, cv2.COLOR_BGR2RGB)
        self.filter_image = cv2.resize(self.filter_image, (self.w, self.h))

    def create_top_layout(self):
        self.top_layout = QHBoxLayout()
        self.top_layout.addWidget(self.createComboBox())

    def create_middle_layout(self):
        self.middle_layout = QHBoxLayout()
        # filter label
        self.label_filter = QLabel(self)
        self.label_filter.resize(self.w, self.h)
        # mask label
        self.label_mask = QLabel(self)
        self.label_mask.resize(self.w, self.h)
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.label_mask)
        splitter.addWidget(self.label_filter)
        self.middle_layout.addWidget(splitter)
        # self.middle_layout.addWidget(self.label_mask)

    def create_bot_layout(self):
        self.bot_layout = QHBoxLayout()
        button_save = QPushButton('Save picture', self)
        button_save.clicked.connect(self.save_picture)
        self.bot_layout.addWidget(button_save)

    def save_picture(self):
        if self.save_image is not None:
            path = f"./pictures/picture_{datetime.now().strftime('%m_%d_%Y_%H_%M_%S')}.jpg"
            cv2.imwrite(path, self.save_image)
        print('SAVE PICTURE')

    def init_ROS(self, robot_name):
        self.ros = ROSSpin(robot_name)
        self.ros.changeROSImage.connect(self.setImage)
        self.ros.start()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("robot", type=str, help="Robot's name")
    parsed = parser.parse_args()
    # ---
    app = QApplication(sys.argv)
    window = App(parsed.robot)
    window.show()
    app.exec_()
    window.ros.terminate()
    window.ros.wait()
    # rospy.spin()


if __name__ == '__main__':
    main()
