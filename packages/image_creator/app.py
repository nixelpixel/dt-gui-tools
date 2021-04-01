import os

import cv2
import sys

import numpy as np
import rospy
from PyQt5 import QtGui
from PyQt5.QtWidgets import QWidget, QLabel, QApplication, QComboBox, QVBoxLayout, QHBoxLayout, QPushButton, QSplitter
from PyQt5.QtCore import QThread, Qt, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QImage, QPixmap
from PIL import Image
from PIL.ImageQt import ImageQt
from cv_bridge import CvBridge

# /<bot>/camera_node/image/compressed
from sensor_msgs.msg import CompressedImage


class ROSSpin(QThread):
    changeROSImage = pyqtSignal(CompressedImage)

    def __init__(self, parent):
        QThread.__init__(self, parent)
        self._parent = parent
        rospy.init_node('ImageCreator', anonymous=False)
        self.image_compressed = rospy.Subscriber(f"/{sys.argv[1]}/camera_node/image/compressed",
                                                 CompressedImage, self.callback, queue_size=1)

    #def run(self):
    #    pass

    # rospy.spin()

    def callback(self, picture: CompressedImage):
        rospy.loginfo("ROS CALLBACK")

        self.changeROSImage.emit(picture)


'''
    # def __init__(self):
    #    self.picture
    def __init__(self):
        super().__init__()
        self.image_compressed = rospy.Subscriber("/autobot08/camera_node/image/compressed",
                                                 CompressedImage, self.callback, queue_size=1)

    def callbcack(self, ros_data: CompressedImage):
        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        bot_image = cv2.imdecode(np_arr, cv2.COLOR_BGR2RGB)

        # cv2 -> pill
        bot_image_pil = Image.fromarray(bot_image)

        # join mask + raw pict
        bot_image_pil = self.get_modified_picture(bot_image_pil)
        # PIL -> QImage
        qt_img = ImageQt(bot_image_pil)
        p = qt_img.scaled(640, 480, Qt.KeepAspectRatio)
        self.changePixmap.emit(p)

    def run(self):
        cap = cv2.VideoCapture(0)
        while True:
            ret, frame = cap.read()
            if ret:
                # https://stackoverflow.com/a/55468544/6622587
                rgbImage = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgbImage.shape
                bytesPerLine = ch * w

                # cv2 -> pill
                im_pil = Image.fromarray(rgbImage)

                # join mask + raw pict
                im_pil = self.get_modified_picture(im_pil)
                # PIL -> QImage
                qt_img = ImageQt(im_pil)
                finish_img = qt_img  # QtGui.QPixmap.fromImage(qt_img)
                #

                convertToQtFormat = QImage(rgbImage.data, w, h, bytesPerLine, QImage.Format_RGB888)
                p = finish_img.scaled(640, 480, Qt.KeepAspectRatio)
                self.changePixmap.emit(p)

    def get_modified_picture(self, picture):
        img1 = Image.open("/code/catkin_ws/src/dt-gui-tools/packages/image_creator/" + "t.png").convert("RGBA")
        p_width, p_height = picture.size
        img1 = img1.resize((p_width, p_height), Image.LANCZOS)
        picture.paste(img1, None, img1)
        return picture
        # img1.thumbnail((640, 480), Image.ANTIALIAS)
        # picture.thumbnail((640, 480), Image.ANTIALIAS)
        # picture.paste(img1, (0,0), img1)
        # return picture

        # img2 = Image.open()

'''


class App(QWidget):
    def __init__(self):
        super().__init__()
        self.filter_image = None
        self.mask_image = None
        script_path = os.path.dirname(__file__)
        self.script_path = (script_path + "/") if script_path else ""
        self.bridge = CvBridge()
        self.initUI()

    @pyqtSlot(CompressedImage)
    def setImage(self, image: CompressedImage):
        self.callback(image)

    def callback(self, ros_data: CompressedImage):
        rospy.loginfo("APP CALLBACK FROM ROS")
        #### direct conversion to CV2 ####
        #np_arr = np.fromstring(ros_data.data, np.uint8)
        #bot_image = cv2.imdecode(np_arr, cv2.COLOR_BGR2RGB)
        rospy.loginfo("Before bridge")
        bot_image = self.bridge.compressed_imgmsg_to_cv2(ros_data, "bgr8")
        # cv2 -> pill
        rospy.loginfo("before fromarray")
        bot_image_pil = Image.fromarray(bot_image)

        # join mask + raw pict
        rospy.loginfo("Before modif mask")
        bot_image_pil_mask = self.get_modified_picture_by_mask(bot_image_pil)
        rospy.loginfo("Before modif filter")
        bot_image_pil_filter = self.get_modified_picture_by_filter(bot_image_pil)

        rospy.loginfo("Before convert RGB")
        bot_image_pil_mask.convert("RGB")
        bot_image_pil_filter.convert("RGB")
        rospy.loginfo("After convert rgb")
        # PIL -> QImage

        mask_qt_img = ImageQt(bot_image_pil_mask)
        mask = mask_qt_img.scaled(self.w, self.h, Qt.KeepAspectRatio)

        filter_qt_img = ImageQt(bot_image_pil_filter)
        filter_img = filter_qt_img.scaled(self.w, self.h, Qt.KeepAspectRatio)

        #mask.convertTo(QtGui.QColor.Rgb) #?
        self.set_picture_mask(mask)
        self.set_picture_filter(filter_img)

    def get_modified_picture_by_mask(self, picture):
        if self.mask_image:
            return self.addition_pictures(picture, self.mask_image)
        return picture

    def get_modified_picture_by_filter(self, picture):
        '''
                img = self.filter_image
                p_width, p_height = picture.size
                img = img.resize((p_width, p_height), Image.LANCZOS)
                picture.paste(img, None, img)
                return picture
        '''
        if self.filter_image:
            return self.addition_pictures(picture, self.filter_image)
        return picture

    def addition_pictures(self, source, top_source):
        p_width, p_height = source.size
        top_source = top_source.resize((p_width, p_height), Image.LANCZOS)
        source.paste(top_source, None, top_source)
        return source

    def set_picture_mask(self, image):
        rospy.loginfo("P-MASK")
        self.set_picture_raw(self.label_mask, image)

    def set_picture_filter(self, image):
        rospy.loginfo("P-FILTER")
        self.set_picture_raw(self.label_filter, image)

    def set_picture_raw(self, dst, image):
        dst.setPixmap(QPixmap.fromImage(image))

    def initUI(self):
        self.setWindowTitle("Image Creator From Duckiebot")
        self.w, self.h = 640, 480
        self.setGeometry(300, 300, self.w + 300, self.h + 200)
        self.resize(2 * self.w + 100, 1.5 * self.h)
        # create a label
        #self.label = QLabel(self)
        windowLayout = QVBoxLayout()
        self.create_top_layout()
        self.create_middle_layout()
        self.create_bot_layout()
        windowLayout.addLayout(self.top_layout)
        windowLayout.addLayout(self.middle_layout)
        windowLayout.addLayout(self.bot_layout)
        self.setLayout(windowLayout)
        self.init_ROS()
        self.show()

    def createComboBox(self):
        combo = QComboBox(self)
        filters = []
        for filter in os.listdir(self.script_path+"filters"):
            if os.path.isdir(self.script_path+'filters/'+filter):
                filters.append(filter)
        combo.addItems(filters)
        combo.activated[str].connect(self.changed_filter)
        return combo

    def changed_filter(self, text):
        path = self.script_path+'filters/' + text + "/"
        mask, filter = path + "mask.png", path + "filter.png"
        self.mask_image = Image.open(mask).convert("RGBA")
        self.filter_image = Image.open(filter).convert("RGBA")
        print(self.mask_image, self.filter_image)

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
        #self.middle_layout.addWidget(self.label_mask)

    def create_bot_layout(self):
        self.bot_layout = QHBoxLayout()
        button_save = QPushButton('Save picture', self)
        button_save.clicked.connect(self.save_picture)
        self.bot_layout.addWidget(button_save)

    def save_picture(self):
        print('SAVE PICTURE')
        # self.filter.save ....

    def init_ROS(self):
        self.ros = ROSSpin(self)
        self.ros.changeROSImage.connect(self.setImage)
        self.ros.start()


def main():
    app = QApplication(sys.argv)
    window = App()
    print(sys.argv[1])
    app.exec_()
    window.ros.terminate()
    window.ros.wait()
    # rospy.spin()


if __name__ == '__main__':
    main()
