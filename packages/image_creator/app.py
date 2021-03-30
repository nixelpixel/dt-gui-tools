import cv2
import sys

from PyQt5 import QtGui
from PyQt5.QtWidgets import  QWidget, QLabel, QApplication
from PyQt5.QtCore import QThread, Qt, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QImage, QPixmap
from PIL import Image
from PIL.ImageQt import ImageQt

class Thread(QThread):
    changePixmap = pyqtSignal(QImage)
    #def __init__(self):
    #    self.picture

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
                finish_img = qt_img#QtGui.QPixmap.fromImage(qt_img)
                #

                convertToQtFormat = QImage(rgbImage.data, w, h, bytesPerLine, QImage.Format_RGB888)
                p = finish_img.scaled(640, 480, Qt.KeepAspectRatio)
                self.changePixmap.emit(p)

    def get_modified_picture(self, picture):
        img1 = Image.open("/code/catkin_ws/src/dt-gui-tools/packages/image_creator/"+"duck_pict1.jpg").convert("RGBA")
        img1.thumbnail((640, 480), Image.ANTIALIAS)
        picture.thumbnail((640, 480), Image.ANTIALIAS)
        picture.paste(img1, (0,0), img1)
        return picture
        #img2 = Image.open()

class App(QWidget):
    def __init__(self):
        super().__init__()
        [...]
        self.initUI()

    @pyqtSlot(QImage)
    def setImage(self, image):
        self.label.setPixmap(QPixmap.fromImage(image))

    def initUI(self):
        self.setWindowTitle("aa")
        self.w, self.h = 640, 480
        self.setGeometry(300, 300, self.w + 300, self.h + 200)
        self.resize(1800, 1200)
        # create a label
        self.label = QLabel(self)
        self.label.move(280, 120)
        self.label.resize(640, 480)
        th = Thread(self)
        th.changePixmap.connect(self.setImage)
        th.start()
        self.show()

def main():
    app = QApplication(sys.argv)
    window = App()
    app.exec_()


if __name__ == '__main__':
    main()
