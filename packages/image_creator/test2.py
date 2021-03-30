import sys    
import numpy as np
import flycapture2 as fc2

from PyQt4.QtCore import (QThread, Qt, pyqtSignal)
from PyQt4.QtGui import (QPixmap, QImage, QApplication, QWidget, QLabel)

class Thread(QThread):
    changePixmap = pyqtSignal(QImage)

    def __init__(self, parent=None):
        QThread.__init__(self, parent=parent)       
        self.cameraSettings()


    def run(self):      
        while True:
            im = fc2.Image()
            self.c.retrieve_buffer(im)
            a = np.array(im)    

            rawImage = QImage(a.data, a.shape[1], a.shape[0], QImage.Format_Indexed8)

            self.changePixmap.emit(rawImage)

    def cameraSettings(self):
        print(fc2.get_library_version())
        self.c = fc2.Context()
        numberCam = self.c.get_num_of_cameras()
        print(numberCam)    
        self.c.connect(*self.c.get_camera_from_index(0))
        print(self.c.get_camera_info())
        m, f = self.c.get_video_mode_and_frame_rate()
        print(m, f)
        print(self.c.get_property_info(fc2.FRAME_RATE))
        p = self.c.get_property(fc2.FRAME_RATE)
        print(p)
        self.c.set_property(**p)
        self.c.start_capture()


class App(QWidget):
    def __init__(self):
            super(App,self).__init__()
            self.title = 'PyQt4 Video'
            self.left = 100
            self.top = 100
            self.width = 640
            self.height = 480
            self.initUI()

    def initUI(self):
            self.setWindowTitle(self.title)
            self.setGeometry(self.left, self.top, self.width, self.height)
            self.resize(800, 600)
            # create a label
            self.label = QLabel(self)
            self.label.move(0, 0)
            self.label.resize(640, 480)
            th = Thread(self)
            th.changePixmap.connect(lambda p: self.setPixMap(p))
            th.start()

    def setPixMap(self, p):     
        p = QPixmap.fromImage(p)    
        p = p.scaled(640, 480, Qt.KeepAspectRatio)
        self.label.setPixmap(p)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = App()
    ex.show()
    sys.exit(app.exec_())
