import sys
from PyQt5.QtCore import QSize, pyqtSignal, Qt
from PyQt5.QtGui import QImage, QPalette, QBrush, QIcon, QPixmap
from PyQt5.QtWidgets import QWidget, QApplication, QPushButton, QLabel, QMainWindow


class MainWindow(QWidget):

    def __init__(self):
        QMainWindow.__init__(self)
        self.initUI()
        self.setFocusPolicy(Qt.StrongFocus)

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_Right:
            self.simulate_ros_command("right")
        elif key == Qt.Key_Left:
            self.simulate_ros_command("left")
        elif key == Qt.Key_Up:
            self.simulate_ros_command("left")
        elif key == Qt.Key_Down:
            self.simulate_ros_command("down")

    def initUI(self):
        self.setGeometry(100,100,300,200)
        label=QLabel(self)  
        self.pixmap=QPixmap('images/image.png')  
        label.setPixmap(self.pixmap)  
        self.resize(self.pixmap.width(),self.pixmap.height())
        self.create_up_button()
        self.create_left_button()
        self.create_right_button()
        self.create_down_button()

    def create_up_button(self):
        button_up = QPushButton("", self)
        icon = QIcon('images/up_button.jpg')
        button_up.setIcon(icon)
        size = 200
        button_up.setIconSize(QSize(size, size))
        button_up.resize(QSize(size - 40, size -10))
        button_up.move(self.pixmap.width()/2 - 80, 20)
        button_up.clicked.connect(lambda _ : self.simulate_ros_command('up'))

    def create_left_button(self):
        button = QPushButton("", self)
        icon = QIcon('images/left_button.jpg')
        button.setIcon(icon)
        size = 200
        button.setIconSize(QSize(size, size))
        button.resize(QSize(size - 5, size - 60))
        button.move(20, self.pixmap.height()/2 - 70)
        button.clicked.connect(lambda _ : self.simulate_ros_command('left'))

    def create_right_button(self):
        button = QPushButton("", self)
        icon = QIcon('images/right_button.jpg')
        button.setIcon(icon)
        size = 200
        button.setIconSize(QSize(size, size))
        button.resize(QSize(size - 5, size - 60))
        button.move(self.pixmap.width()/2 + 85, self.pixmap.height()/2 - 70)
        button.clicked.connect(lambda _ : self.simulate_ros_command('right'))

    def create_down_button(self):
        button = QPushButton("", self)
        icon = QIcon('images/down_button.jpg')
        button.setIcon(icon)
        size = 200
        button.setIconSize(QSize(size, size))
        button.resize(QSize(size - 50, size -10))
        button.move(self.pixmap.width()/2 - 75, self.pixmap.height()/2 + 87)
        button.clicked.connect(lambda _ : self.simulate_ros_command('down'))

    def simulate_ros_command(self, command):
        print('Command: {}'.format(command))

if __name__ == "__main__":

    app = QApplication(sys.argv)
    m = MainWindow()
    m.show()
    sys.exit(app.exec_())