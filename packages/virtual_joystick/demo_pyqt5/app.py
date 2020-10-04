import sys
from PyQt5.QtCore import QSize, pyqtSignal, Qt
from PyQt5.QtCore import QThread
from PyQt5.QtGui import QImage, QPalette, QBrush, QIcon, QPixmap, QKeyEvent
from PyQt5.QtWidgets import QWidget, QApplication, QPushButton, QLabel, QMainWindow, QVBoxLayout
from pynput.keyboard import Key, Listener, KeyCode
from time import sleep
import rospy
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import BoolStamped
import socket
import re
import time

HZ = 30

KEY_LEFT = 'left'
KEY_RIGHT = 'right'
KEY_UP = 'up'
KEY_DOWN = 'down'
KEY_A = 'a'
KEY_Q = 'q'
KEY_S = 's'
KEY_I = 'i'
KEY_E = 'e'
KEY_P = 'p'


Keys = {
   'Key.up': KEY_UP,
   'Key.down': KEY_DOWN,
   'Key.left': KEY_LEFT,
   'Key.right': KEY_RIGHT,
   KEY_A: KEY_A,
   KEY_Q: KEY_Q,
   KEY_S: KEY_S,
   KEY_I: KEY_I,
   KEY_E: KEY_E,
   KEY_P: KEY_P
}

speed_tang = 1.0
speed_norm = 1.0
time_to_wait = 10000
estop_deadzone_secs = 0.5

commands = set()

class ROSManager(QThread):

    def __init__(self, parent):
        QThread.__init__(self, parent)
        rospy.init_node('virtual_joy', anonymous=False)
        self.sub_estop = rospy.Subscriber("~emergency_stop", BoolStamped, self.cbEStop, queue_size=1)
        self.pub_joystick = rospy.Publisher("~joy", Joy, queue_size=1)
        self.pub_int = rospy.Publisher("~intersection_go", BoolStamped, queue_size=1)
        self.commands = set()
        self.estop_last_time = time.time()
        self.last_ms = 0
        self.emergency_stop = False

    def cbEStop(self, estop_msg):
        """
        Callback that process the received :obj:`BoolStamped` messages.

        Args:
            estop_msg (:obj:`BoolStamped`): the emergency_stop message to process.
        """
        self.emergency_stop = estop_msg.data
        #if self.emergency_stop:
        #    pass
            # some stop
    
    def run(self):
        veh_standing = True

        while True:
            #if self.commands:
            #    for command in self.commands:
            #        if command in list(Keys.keys())[:3]:
            #            print(command)
            ms_now = int(round(time.time() * 1000))
            
            try:
                if ms_now - self.last_ms > time_to_wait:
                    rospy.get_master().getSystemState()
                    self.last_ms = ms_now
            except socket.error:
                print("Error starting main loop in virtual joystick gui")

            msg = self.get_raw_message()
            force_joy_publish = False

            # Arrows events
            if KEY_LEFT in self.commands:
                msg.axes[3] += speed_norm
            
            if KEY_RIGHT in self.commands:
                msg.axes[3] -= speed_norm

            if KEY_UP in self.commands:
                msg.axes[1] += speed_tang

            if KEY_DOWN in self.commands:
                msg.axes[1] -= speed_tang

            if KEY_A in self.commands:
                msg.buttons[7] = 1

            if KEY_S in self.commands:
                msg.buttons[6] = 1

            if KEY_I in self.commands:
                msg.buttons[3] = 1

            if KEY_P in self.commands:
                msg_int = BoolStamped()
                msg_int.data = True
                self.pub_int.publish(msg_int)

            if KEY_E in self.commands and (time.time() - self.estop_last_time > estop_deadzone_secs):
                msg.buttons[3] = 1
                self.estop_last_time = time.time()
                force_joy_publish = True

            if KEY_Q in self.commands:
                sys.exit(0)

            stands = (sum(map(abs, msg.axes)) == 0 and sum(map(abs, msg.buttons)) == 0)
            if not stands:
                veh_standing = False

            if (not veh_standing) or force_joy_publish:
                self.pub_joystick.publish(msg)
            
            if stands:
                veh_standing = True
        
            sleep(1/HZ)

    def action(self, commands):
        self.commands = commands

    def get_raw_message(self):
        msg = Joy()
        msg.header.seq = 0
        msg.header.stamp.secs = 0
        msg.header.stamp.nsecs = 0
        msg.header.frame_id = ''
        msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        return msg


class MyKeyBoardThread(QThread):

    key_board_event  = pyqtSignal(object)

    def __init__(self, parent):
        QThread.__init__(self, parent)

    def add_listener(self, listener):
        self.key_board_event.connect(listener)

    def on_press(self, key):
        key_val = str(key)
        if len(key_val) == 3:
            key_val = key_val[1]
        if key_val in Keys.keys():
            commands.add(Keys[key_val])
        self.key_board_event.emit(commands)
        
    def on_release(self, key):
        key_val = str(key)
        if len(key_val) == 3:
            key_val = key_val[1]
        if key_val in Keys.keys():
            commands.remove(Keys[key_val])

    def run(self):
        with Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()


class MainWindow(QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        # ros class
        self.ros = ROSManager(self)
        self.ros.start()        
        # Key board
        self.key_board_event = MyKeyBoardThread(self)
        self.key_board_event.add_listener(self.ros_wrapper)
        self.key_board_event.start()
        ## UI JOYSTICK
        widget = Joystick()
        widget.ros_fun.connect(self.simulate_ros_command)
        self.resize(widget.pixmap.width(), widget.pixmap.height())

        ####
        self.setCentralWidget(widget)
        self.ros_commands = set()
        self.was_added_new_key = False

    def ros_wrapper(self, commands):
        if self.isActiveWindow() and commands:
            self.ros.action(commands)
    
    def simulate_ros_command(self, command):
        # deprecated
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
        button_up.move(int(self.pixmap.width()/2 - 80), 20)
        button_up.clicked.connect(lambda _ : self.ros_fun.emit('up'))

    def create_left_button(self):
        button = QPushButton("", self)
        icon = QIcon('images/left_button.jpg')
        button.setIcon(icon)
        size = 200
        button.setIconSize(QSize(size, size))
        button.resize(QSize(size - 5, size - 60))
        button.move(20, int(self.pixmap.height()/2 - 70))
        button.clicked.connect(lambda _ : self.ros_fun.emit('left'))

    def create_right_button(self):
        button = QPushButton("", self)
        icon = QIcon('images/right_button.jpg')
        button.setIcon(icon)
        size = 200
        button.setIconSize(QSize(size, size))
        button.resize(QSize(size - 5, size - 60))
        button.move(int(self.pixmap.width()/2 + 85), int(self.pixmap.height()/2 - 70))
        button.clicked.connect(lambda _ : self.ros_fun.emit('right'))

    def create_down_button(self):
        button = QPushButton("", self)
        icon = QIcon('images/down_button.jpg')
        button.setIcon(icon)
        size = 200
        button.setIconSize(QSize(size, size))
        button.resize(QSize(size - 50, size -10))
        button.move(int(self.pixmap.width()/2 - 75), int(self.pixmap.height()/2 + 87))
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
    if len(sys.argv) < 2:
        raise Exception("No hostname specified!")
    else:
        veh_name = sys.argv[1]

    veh_no = re.sub("\D", "", veh_name)
    main_letter = veh_name[0]

    print_hint()
    
    app = QApplication(sys.argv)
    m = MainWindow()
    m.show()
    exit_code = app.exec_()
    sys.exit(exit_code)