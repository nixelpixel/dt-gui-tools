import sys
from PyQt5.QtCore import QSize, pyqtSignal, Qt
from PyQt5.QtGui import QImage, QPalette, QBrush, QIcon, QPixmap, QKeyEvent
from PyQt5.QtWidgets import QWidget, QApplication, QPushButton, QLabel, QMainWindow, QVBoxLayout



class MainWindow(QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        widget = Joystick()
        widget.ros_fun.connect(self.simulate_ros_command)
        self.resize(widget.pixmap.width(), widget.pixmap.height())
        self.setCentralWidget(widget)
        self.ros_commands = set()
        self.was_added_new_key = False

    def eventFilter(self, source, event):
        if isinstance(event, QKeyEvent) and self.ros_commands:
            self.simulate_ros_command(" - ".join(list(self.ros_commands)))   
            return False
        return super(MainWindow, self).eventFilter(source, event)
    
    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_Right:
            self.ros_commands.add("right")
        elif key == Qt.Key_Left:
            self.ros_commands.add("left")
        elif key == Qt.Key_Up:
            self.ros_commands.add("up")
        elif key == Qt.Key_Down:
            self.ros_commands.add("down") 

    def keyReleaseEvent(self, event):
        if self.was_added_new_key:
            self.was_added_new_key = False
            return
        key = event.key()
        if key == Qt.Key_Right:
            self.ros_commands.remove("right")
        elif key == Qt.Key_Left:
            self.ros_commands.remove("left")
        elif key == Qt.Key_Up:
            self.ros_commands.remove("up")
        elif key == Qt.Key_Down:
            self.ros_commands.remove("down")


    def simulate_ros_command(self, command):
        print('Command: {}'.format(command))

class Joystick(QWidget):
    ros_fun = pyqtSignal(str)

    def __init__(self, *args, **kwargs):
        super(Joystick, self).__init__(*args, **kwargs)
        self.initUI()
        self.setFocusPolicy(Qt.StrongFocus)

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
        button_up.clicked.connect(lambda _ : self.ros_fun.emit('up'))

    def create_left_button(self):
        button = QPushButton("", self)
        icon = QIcon('images/left_button.jpg')
        button.setIcon(icon)
        size = 200
        button.setIconSize(QSize(size, size))
        button.resize(QSize(size - 5, size - 60))
        button.move(20, self.pixmap.height()/2 - 70)
        button.clicked.connect(lambda _ : self.ros_fun.emit('left'))

    def create_right_button(self):
        button = QPushButton("", self)
        icon = QIcon('images/right_button.jpg')
        button.setIcon(icon)
        size = 200
        button.setIconSize(QSize(size, size))
        button.resize(QSize(size - 5, size - 60))
        button.move(self.pixmap.width()/2 + 85, self.pixmap.height()/2 - 70)
        button.clicked.connect(lambda _ : self.ros_fun.emit('right'))

    def create_down_button(self):
        button = QPushButton("", self)
        icon = QIcon('images/down_button.jpg')
        button.setIcon(icon)
        size = 200
        button.setIconSize(QSize(size, size))
        button.resize(QSize(size - 50, size -10))
        button.move(self.pixmap.width()/2 - 75, self.pixmap.height()/2 + 87)
        button.clicked.connect(lambda _ : self.ros_fun.emit('down'))


if __name__ == "__main__":

    app = QApplication(sys.argv)
    m = MainWindow()
    m.installEventFilter(m)
    m.show()
    sys.exit(app.exec_())