import sys
import numpy as np
import argparse
import imutils
import cv2
import rospy
import json
import roslib
import pigpio
import qdarkstyle
import math
from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtWebKit import QWebSettings
from PyQt5.QtWebKitWidgets import *
from basestation_ui import Ui_MainWindow
from imutils.object_detection import non_max_suppression
from imutils import paths
from os import path
from threading import Thread
from time import sleep
from std_msgs.msg import Int8
from sensor_msgs.msg import Image, CompressedImage, NavSatFix, Joy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, Twist

roslib.load_manifest('sigsent')

from sigsent.msg import GPSList, Battery

# IMPORTANT
# To connect Python to Google Maps Js/HTML, follow this:
# https://stackoverflow.com/questions/47252632/how-to-pass-info-from-js-to-python-using-qwebchannel
# https://github.com/eyllanesc/stackoverflow/tree/master/47252632

ROS = False

class RecordVideo(QtCore.QObject):
    image_data = QtCore.pyqtSignal(object)
    
    def __init__(self, camera_port=0, parent=None):
        super(RecordVideo, self).__init__(parent)
        
        if not ROS:
            self.camera = cv2.VideoCapture(camera_port)
        
        self.bridge = CvBridge()            
        self.timer = QtCore.QBasicTimer()
        self.qthread = QThread()
        self.people_detection_object = PedestrianDetection(self.image_data)
        self.people_detection_object.moveToThread(self.qthread)
        self.people_detection_object.finished.connect(self.qthread.quit)
        self.qthread.start()

        self.last_img = None

        if ROS:
            self.sub = rospy.Subscriber('/usb_cam/image_raw/compressed',
                                        CompressedImage,
                                        self.img_cb,
                                        queue_size=10)

    def start_recording(self):
        self.timer.start(0, self)

    def timerEvent(self, event):
        if (event.timerId() != self.timer.timerId()):
            return

        if ROS:
            if self.last_img:
                image = self.img_to_cv2(self.last_img)
                image = imutils.resize(image, width=min(400, image.shape[1]))
                self.people_detection_object.detect_people(image)
        else:
            read, image = self.camera.read()
            if not read:
                image = cv2.imread('images/person_010.bmp')
                image = imutils.resize(image, width=min(400, image.shape[1]))
                self.people_detection_object.detect_people(image)
        
    def img_to_cv2(self, image_msg):
        """
        Convert the image message into a cv2 image (numpy.ndarray)
        to be able to do OpenCV operations in it.
        :param Image or CompressedImage image_msg: the message to transform
        """
        type_as_str = str(type(image_msg))
        if type_as_str.find('sensor_msgs.msg._CompressedImage.CompressedImage') >= 0:
            # Image to numpy array
            np_arr = np.fromstring(image_msg.data, np.uint8)
            # Decode to cv2 image and store
            return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif type_as_str.find('sensor_msgs.msg._Image.Image') >= 0:
            # Use CvBridge to transform
            try:
                return self.bridge.imgmsg_to_cv2(image_msg,
                                                 image_msg.encoding)  # "bgr8"
            except CvBridgeError as e:
                rospy.logerr("Error when converting image: " + str(e))
                return None
        else:
            rospy.logerr("We don't know how to transform image of type " +
                         str(type(image_msg)) + " to cv2 format.")
            return None
    
    def img_cb(self, image):
        """
        Callback for the Image or Compressed image subscriber, storing
        this last image and setting a flag that the image is new.
        :param Image or CompressedImage image: the data from the topic
        """
        
        self.last_img = image
        self.is_new_img = True

class PedestrianDetection(QObject):
    finished = pyqtSignal()

    def __init__(self, image_data_slot):
        super(QObject, self).__init__()
        
        self.image_data_slot = image_data_slot
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def detect_people(self, image):
        orig = image.copy()

        # detect people in the image
        (rects, weights) = self.hog.detectMultiScale(image, winStride=(4, 4),
            padding=(8, 8), scale=1.05)

        # apply non-maxima suppression to the bounding boxes using a
        # fairly large overlap threshold to try to maintain overlapping
        # boxes that are still people
        rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
        pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)

        self.image_data_slot.emit((pick, image))
        

class PedestrianDetectionWidget(QtWidgets.QWidget):
    def __init__(self, cv_label, parent=None):
        super(PedestrianDetectionWidget, self).__init__(parent)
        self.cv_label = cv_label
        
        self.image = QtGui.QImage()
        self._red = (0, 0, 255)
        self._width = 2
        self._min_size = (30, 30)

    def image_data_slot(self, people_detection):
        people, image_data = people_detection
        
        for (x, y, w, h) in people:
            cv2.rectangle(image_data, (x, y), (w, h), self._red, self._width)

        self.image = self.get_qimage(image_data)
        if self.image.size() != self.size():
            self.setFixedSize(self.image.size())

        self.update()

        if len(people) > 0:
            self.cv_label.setText('Pedestrian Detected')
            self.cv_label.setStyleSheet('QLabel { color : green; }')
        else:
            self.cv_label.setText('Pedestrian NOT Detected')
            self.cv_label.setStyleSheet('QLabel { color : red; }')

    def get_qimage(self, image):
        height, width, colors = image.shape
        bytesPerLine = 3 * width
        QImage = QtGui.QImage

        image = QImage(image.data, width, height, bytesPerLine, QImage.Format_RGB888)

        image = image.rgbSwapped()
        return image

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.drawImage(0, 0, self.image)
        self.image = QtGui.QImage()

class MainWidget(QtWidgets.QWidget):
    def __init__(self, cv_widget, parent=None):
        super(MainWidget, self).__init__(parent)
        self.people_detection_widget = PedestrianDetectionWidget(cv_widget)

        # TODO: set video port
        self.record_video = RecordVideo()
        self.run_button = QtWidgets.QPushButton('Start')

        # Connect the image data signal and slot together
        image_data_slot = self.people_detection_widget.image_data_slot
        self.record_video.image_data.connect(image_data_slot)
        # connect the run button to the start recording slot
        self.run_button.clicked.connect(self.started_video)

        # Create and set the layout
        layout = QtWidgets.QVBoxLayout()
        layout.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.people_detection_widget)
        layout.addWidget(self.run_button)

        self.setLayout(layout)

    def started_video(self):
        self.record_video.start_recording()
        self.run_button.setHidden(True)

class TeleOp():
    # rosparam set joy_node/dev "/dev/input/jsX"
    # rosrun joy joy_node
    # Add these to launch files so they launch with sigsent local.launch items
    def __init__(self):
        if ROS:
            self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_cb, queue_size=10)
            self.teleop_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
        
        # Choose an upperbound for the twist value
        self.speed = 2

        self.teleop_enabled = False
        self.checkbox = QCheckBox('Enable TeleOp')
        self.checkbox.setChecked(False)
        self.checkbox.toggled.connect(self.toggled_checkbox)

    def toggled_checkbox(self):
        self.teleop_enabled ^= True

    def joy_cb(self, data):
        if not self.teleop_enabled:
            return
            
        msg = Twist()

        # Joystick state is published to /joy, need to test joystick for correct inputs
        # Using those inputs, create a Twist() message here to send to the teleop node
        #               fwd  twist
        # axes: [-0.0, -0.0, -0.0, 0.0, 0.0, 0.0]
        msg.linear.x = self.speed * data.axes[1]
        msg.angular.z = self.speed * data.axes[2]
        
        self.teleop_pub.publish(msg)

class Maps(QObject):
    def __init__(self, parent):
        super(Maps, self).__init__(parent)

        with open('map.html', 'r') as map_file:
            maphtml = map_file.read()
            
        self.parent = parent
        self.web = QWebView(parent)
        self.web.settings().setAttribute(QWebSettings.DeveloperExtrasEnabled, True)
        self.web.setHtml(maphtml)
        self.web.page().mainFrame().addToJavaScriptWindowObject('self', self)

        self.goals_table = QTableWidget(0, 2, parent)
        self.goals_table.setHorizontalHeaderLabels(['Latitude','Longitude'])
        self.table_header = self.goals_table.horizontalHeader()
        self.table_header.setSectionResizeMode(QtWidgets.QHeaderView.Stretch)        
        self.goals_table.setFixedHeight(300)
        
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.timerEvent)
        
        self.coords = (28.6024556,-81.2023988)
        self.seq = 0
        
        self.clear_button = QtWidgets.QPushButton('Clear Goals')
        self.clear_button.clicked.connect(self.clear_goals)

        self.send_button = QtWidgets.QPushButton('Send Goals')
        self.send_button.clicked.connect(self.send_goals)

        if ROS:
            self.goal_pub = rospy.Publisher('/gps_goals_list', GPSList, queue_size=1)

            self.gps_sub = rospy.Subscriber('/fix',
                                        NavSatFix,
                                        self.gps_callback,
                                        queue_size=1)
        
    @pyqtSlot(str)
    def update_goals_table(self, map_str):
        goals = json.loads(map_str)
        self.goals_table.clear()
        self.goals_table.setHorizontalHeaderLabels(['Latitude','Longitude'])
        self.goals_table.setRowCount(len(goals))

        for row,goal in enumerate(goals):
            self.goals_table.setItem(row , 0, QTableWidgetItem(str(goal['lat'])))
            self.goals_table.setItem(row , 1, QTableWidgetItem(str(goal['lng'])))

    def send_goals(self):
        frame = self.web.page().currentFrame()
        goals = frame.documentElement().evaluateJavaScript("get_goals()")
        print(goals)
        if goals and ROS:
            goals_msg = GPSList()
            for goal in goals:
                fix = NavSatFix()
                
                fix.latitude = goal['lat']
                fix.longitude = goal['lng']
                fix.altitude = 9.3
                fix.header.frame_id = '/map'
                fix.header.stamp = rospy.Time.now()
                fix.header.seq = self.seq
                fix.position_covariance_type = 1
                
                self.seq += 1

                goals_msg.goals.append(fix)
            print(goals_msg)
            self.goal_pub.publish(goals_msg)
        
    def clear_goals(self):
        frame = self.web.page().currentFrame()
        frame.documentElement().evaluateJavaScript("clearGoals()")
        self.goals_table.clear()
        self.goals_table.setHorizontalHeaderLabels(['Latitude','Longitude'])
        self.goals_table.setRowCount(0)

    def set_coords_label(self, label):
        self.coords_label = label

    def start_timer(self):
        self.timer.start(1000)

    # Update robot marker position on map
    def timerEvent(self):
        if self.coords:
            frame = self.web.page().currentFrame()
            frame.documentElement().evaluateJavaScript("setCenter({},{})".format(*self.coords))
            frame.documentElement().evaluateJavaScript("addMarker({},{})".format(*self.coords))
        
            current_marker = frame.documentElement().evaluateJavaScript("get_position()")
            
            if current_marker:
                self.coords_label.setText('Latitude: {}, Longitude: {}'.format(current_marker['lat'], current_marker['lng']))

    def gps_callback(self, data):
        self.coords = (data.latitude, data.longitude)

class Lightbar():
    def __init__(self):
        self.light_on = False
        self.checkbox = QCheckBox('Enable Light')
        self.checkbox.setChecked(False)
        self.checkbox.toggled.connect(self.toggled_checkbox)
        
        self.strobe_button = QtWidgets.QPushButton('Strobe')
        self.strobe_button.clicked.connect(self.strobe)
        
        self.light_pub = rospy.Publisher('/blind', Int8, queue_size=10)

    def toggled_checkbox(self):
        self.light_on ^= True
        msg = Int8()
        msg.data = int(self.light_on)

        self.light_pub.publish(msg)

    # Strobe light to alert pedestrians, then reset to light off
    def strobe(self):
        msg = Int8()
        msg.data = 2

        self.light_pub.publish(msg)

class Battery:
    def __init__(self, battery_bar, voltage_label, current_label, temperature_label):
        self.battery_bar = battery_bar
        self.voltage_label = voltage_label
        self.current_label = current_label
        self.temperature_label = temperature_label

        if ROS:
            self.battery_sub = rospy.Subscriber('battery', Battery, self.battery_cb, queue_size=10)

    def battery_cb(self, data):
        self.battery_bar.setValue(int(math.ceil(data.percent_full * 100)))
        self.voltage_label.setText('{}V'.format(data.voltage))
        self.current_label.setText('{}A'.format(data.current))
        self.temperature_label.setText('{}F'.format(data.temperature))
        
class Basestation(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super(Basestation, self).__init__(parent)
        self.setupUi(self)

        self.maps = Maps(self.gps_map)
        self.maps.set_coords_label(self.coords_label)
        self.run_button = QtWidgets.QPushButton('Read GPS')
        self.run_button.clicked.connect(self.maps.start_timer)
        self.maps_layout.insertWidget(3,self.maps.clear_button)
        self.maps_layout.insertWidget(3,self.maps.send_button)
        self.maps_layout.insertWidget(3,self.run_button)

        self.cv_widget = MainWidget(self.cv_label)
        self.cv_widget.setFixedHeight(360)
        self.functionality.insertWidget(1,self.cv_widget)
        
        self.teleop = TeleOp()
        self.user_tools.insertWidget(0, self.teleop.checkbox)

        self.lightbar = Lightbar()
        self.user_tools.insertWidget(1, self.lightbar.checkbox)
        self.user_tools.insertWidget(2, self.lightbar.strobe_button)

        self.maps_layout.addWidget(self.maps.goals_table)

        self.battery = Battery(self.battery_bar, self.voltage_label, self.current_label, self.temperature_label)

if __name__ == '__main__':
    if ROS:
        rospy.init_node('base_station_gui')
    
    app = QApplication(sys.argv)
    app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())

    bs = Basestation()    
    bs.show()

    #Exit the application when the Qt window closes
    sys.exit(app.exec_())
