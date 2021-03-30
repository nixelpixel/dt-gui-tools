import time

import cv2
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtCore import Qt, QThread, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QIcon, QPixmap, QImage
from PyQt5.QtWidgets import QMessageBox, QDesktopWidget, QFormLayout, QVBoxLayout, QLineEdit, QCheckBox, QGroupBox, \
    QLabel, QComboBox, QAction
import rospy
from sensor_msgs.msg import CameraInfo, CompressedImage
from cv_bridge import CvBridge


class imageGenerator(QThread):
    changePixmap = pyqtSignal(QImage)

    def run(self):
        cap = cv2.VideoCapture("/code/catkin_ws/src/dt-gui-tools/packages/image_creator/video.mkv")
        print(1)
        while True:
            print(2)
            ret, frame = cap.read()
            if ret:
                print(3)
                # https://stackoverflow.com/a/55468544/6622587
                rgbImage = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgbImage.shape
                bytesPerLine = ch * w
                convertToQtFormat = QImage(rgbImage.data, w, h, bytesPerLine, QImage.Format_RGB888)
                p = convertToQtFormat.scaled(640, 480, Qt.KeepAspectRatio)
                self.changePixmap.emit(p)


class imageCreator(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.w, self.h = 640, 480
        self.initUI()
        self.image = None
        #self.without_ros_loop()
        # self.duckiebot_sub = rospy.Subscriber("SUBSCRIBER", CompressedImage,  self.update_image, queue_size=1)

    def without_ros_loop(self):
        cap = cv2.VideoCapture('/code/catkin_ws/src/dt-gui-tools/packages/image_creator/video.mkv')
        while cap.isOpened():
            ret, frame = cap.read()
            time.sleep(1)
            self.update_image(frame)
            print(1)
            # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()

    def update_image(self, image_msg):
        self.image = image_msg  # self.bridge.compressed_imgmsg_to_cv2(image_msg)
        self.image = cv2.resize(self.image, (self.w, self.h), interpolation=cv2.INTER_NEAREST)
        self.vF.setPixmap(QPixmap.fromImage(
            QImage(cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB),
                   self.w,
                   self.h,
                   13)))
        print(2)

    @pyqtSlot(QImage)
    def setImage(self, image):
        self.left_view.setPixmap(QPixmap.fromImage(image))

    def initImageGenerator(self):
        th = imageGenerator(self)
        th.changePixmap.connect(self.setImage)
        th.start()

    def initUI(self):
        self.left_view = QLabel()
        self.setCentralWidget(self.left_view)
        self.left_view.setAlignment(Qt.AlignCenter)

        self.setGeometry(300, 300, self.w + 300, self.h + 200)
        self.setWindowTitle('Simple menu1')
