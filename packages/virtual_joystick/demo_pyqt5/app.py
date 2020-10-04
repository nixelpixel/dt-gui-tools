import sys
from PyQt5.QtCore import QSize, pyqtSignal, Qt
from PyQt5.QtCore import QThread
from PyQt5.QtGui import QImage, QPalette, QBrush, QIcon, QPixmap, QKeyEvent
from PyQt5.QtWidgets import QWidget, QApplication, QPushButton, QLabel, QMainWindow, QVBoxLayout
from pynput.keyboard import Key, Listener
from time import sleep
import rospy
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import BoolStamped

HZ = 30

Keys = {
   'Key.up': 'up',
   'Key.down': 'down',
   'Key.left': 'left',
   'Key.right': 'right' 
}

commands = set()

def cbEStop(msg):
    pass

class ROSManager():

    def __init__():
        rospy.init_node('virtual_joy', anonymous=False)
        self.sub_estop = rospy.Subscriber("~emergency_stop", BoolStamped, cbEStop, queue_size=1)
        self.pub_joystick = rospy.Publisher("~joy", Joy, queue_size=1)
        self.pub_int = rospy.Publisher("~intersection_go", BoolStamped, queue_size=1)



class ROSSimulator(QThread):

    ros_print = pyqtSignal(object)
    
    def __init__(self, parent):
        QThread.__init__(self, parent)

    def add_printer(self,  listener):
        self.ros_print.connect(listener)

    def run(self):
        while True:
            for command in commands:
                self.ros_print.emit(command) 
            sleep(1/HZ)


class MyKeyBoardThread(QThread):

    key_board_event  = pyqtSignal(object)

    def __init__(self, parent):
        QThread.__init__(self, parent)

    def add_listener(self, listener):
        self.key_board_event.connect(listener)

    def on_press(self, key):
        key_val = str(key)
        if key_val in Keys.keys():
            commands.add(Keys[key_val])
        for command in commands:
            self.key_board_event.emit(command)

    def on_release(self, key):
        key_val = str(key)
        if key_val in Keys.keys():
            commands.remove(Keys[key_val])

    def run(self):
        with Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()


class MainWindow(QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        # Key board
        self.key_board_event = MyKeyBoardThread(self)
        self.key_board_event.add_listener(self.simulate_ros_command)
        self.key_board_event.start()
        # PRINT
        self.ros_printer = ROSSimulator(self)
        self.ros_printer.add_printer(self.simulate_ros_command)
        self.ros_printer.start()
        ##
        widget = Joystick()
        widget.ros_fun.connect(self.simulate_ros_command)
        self.resize(widget.pixmap.width(), widget.pixmap.height())
        self.setCentralWidget(widget)
        self.ros_commands = set()
        self.was_added_new_key = False

    def simulate_ros_command(self, command):
        if self.isActiveWindow():
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


def print_hint():
    print("\n\n\n")
    print("Virtual Joystick for your Duckiebot")
    print("-----------------------------------")
    print("\n")
    print("[ARROW_KEYS]:    Use them to steer your Duckiebot")
    print("         [q]:    Quit the program")
    print("         [a]:    Start lane-following a.k.a. autopilot")
    print("         [s]:    Stop lane-following")
    print("         [i]:    Toggle anti-instagram")
    print("         [e]:    Toggle emergency stop")
    print("\n")

if __name__ == "__main__":

    print_hint()
    
    app = QApplication(sys.argv)
    m = MainWindow()
    m.show()
    exit_code = app.exec_()
    sys.exit(exit_code)