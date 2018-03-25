import sys
"""
sys.path.insert(0, './qMap')

import qgmap
qgmap.use('PyQt5')

from qgmap.common import QGoogleMap
"""
from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtWebKit import QWebSettings
from PyQt5.QtWebKitWidgets import *
from basestation_ui import Ui_MainWindow
from imutils.object_detection import non_max_suppression
from imutils import paths
import numpy as np
import argparse
import imutils
import cv2
from os import path
from threading import Thread
from time import sleep
import rospy
from sensor_msgs.msg import Image, CompressedImage, NavSatFix
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped

# IMPORTANT
# To connect Python to Google Maps Js/HTML, follow this:
# https://stackoverflow.com/questions/47252632/how-to-pass-info-from-js-to-python-using-qwebchannel
# https://github.com/eyllanesc/stackoverflow/tree/master/47252632

maphtml = '''
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="initial-scale=1.0, user-scalable=no" />
<style type="text/css">
  html { height: 100% }
  body { height: 100%; margin: 0px; padding: 0px }
  #map_canvas { height: 100% }
</style>
<script type="text/javascript"
  src="http://maps.google.com/maps/api/js?sensor=false">
</script>
<script type="text/javascript">
var map;
var markers = [];
var goal_markers = [];
var red = "FE7569";
var blue = "4245f4";
var positionImage = new google.maps.MarkerImage("http://chart.apis.google.com/chart?chst=d_map_pin_letter&chld=%E2%80%A2|" + red,
    new google.maps.Size(21, 34),
    new google.maps.Point(0,0),
    new google.maps.Point(10, 34));
var goalImage = new google.maps.MarkerImage("http://chart.apis.google.com/chart?chst=d_map_pin_letter&chld=%E2%80%A2|" + blue,
    new google.maps.Size(21, 34),
    new google.maps.Point(0,0),
    new google.maps.Point(10, 34));

function initialize() {
    var latlng = new google.maps.LatLng(28.6024556,-81.2023988,17);
    var myOptions = {
                    zoom: 17,
                    center: latlng,
                    mapTypeId: google.maps.MapTypeId.ROADMAP
                    };
     map = new google.maps.Map(document.getElementById("map_canvas"),
                               myOptions);
    // This event listener will call addMarker() when the map is clicked.
        map.addListener('click', function(event) {
          _addMarker(event.latLng);
        });
     doNothing();
 }

    function _addMarker(location)
    {
        var marker = new google.maps.Marker({
          position: location,
          map: map,
          icon: goalImage
        });
        for (var i = 0; i < goal_markers.length; i++) {
          goal_markers[i].setMap(null);
        }
        goal_markers = [];

        goal_markers.push(marker);
    }
 function addMarker(lat, lng) {
  var myLatLng = new google.maps.LatLng(lat, lng);
      var marker = new google.maps.Marker({position: myLatLng,
                        map: map,
                        icon: positionImage
                                               });
        for (var i = 0; i < markers.length; i++) {
          markers[i].setMap(null);
        }
        markers = [];            
        markers.push(marker);
        markers.splice(0, 1);

 }

    function setCenter(lat, lng) {
        var myLatLng = new google.maps.LatLng(lat, lng);
        map.setCenter(myLatLng);    
    }
     function setMapOnAll(map) {
        for (var i = 0; i < markers.length; i++) {
          markers[i].setMap(map);
        }
      }
    function clearMap()
    {
                setMapOnAll(null);
        markers = [];


    }


    function get_goal()
    {
        if (goal_markers.length == 0)
            return null;
        
        return {'lat': goal_markers[0].getPosition().lat(), 'lng': goal_markers[0].getPosition().lng()};
    }
</script>
</head>
<body onload="initialize();">
    <div id="map_canvas" style="width:100%; height:100%"></div>
</body>
</html>
'''

ROS = False

class RecordVideo(QtCore.QObject):
    image_data = QtCore.pyqtSignal(np.ndarray)

    def __init__(self, camera_port=0, parent=None):
        super(RecordVideo, self).__init__(parent)
        self.camera = cv2.VideoCapture(camera_port)
        self.timer = QtCore.QBasicTimer()
        
        if ROS:
            self.sub = rospy.Subscriber('/RasPiCam/image/compressed',
                                        CompressedImage,
                                        self.img_cb,
                                        queue_size=1)

    def start_recording(self):
        self.timer.start(0, self)

    def timerEvent(self, event):
        if (event.timerId() != self.timer.timerId()):
            return

        if ROS and self.last_img:
            image = self.img_to_cv2(self.last_img)
            image = imutils.resize(image, width=min(400, image.shape[1]))
            self.image_data.emit(image)

        else:
            image = cv2.imread('images/person_010.bmp')
            image = imutils.resize(image, width=min(400, image.shape[1]))
            self.image_data.emit(image)

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

class PedestrianDetectionWidget(QtWidgets.QWidget):
    def __init__(self, cv_label, parent=None):
        super(PedestrianDetectionWidget, self).__init__(parent)
        self.cv_label = cv_label
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        
        self.image = QtGui.QImage()
        self._red = (0, 0, 255)
        self._width = 2
        self._min_size = (30, 30)

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

        return pick

    def image_data_slot(self, image_data):
        people = self.detect_people(image_data)
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
        self.run_button.clicked.connect(self.record_video.start_recording)

        # Create and set the layout
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.people_detection_widget)
        layout.addWidget(self.run_button)

        self.setLayout(layout)

class Basestation(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super(Basestation, self).__init__(parent)
        self.setupUi(self)
        self.web = QWebView(self.gps_map)
        self.web.settings().setAttribute(QWebSettings.DeveloperExtrasEnabled, True)
        self.web.setHtml(maphtml)
        
        self.cv_widget = MainWidget(self.cv_label)
        self.cv_widget.setFixedHeight(400)
        self.functionality.insertWidget(1,self.cv_widget)
        
        self.run_button = QtWidgets.QPushButton('Start')
        self.run_button.clicked.connect(self.start_timer)
        self.coords = None if ROS else (28.6024556,-81.2023988)
        self.seq = 0
        
        if ROS:
            self.goal_pub = rospy.Publisher('/gps_goal_fix', NavSatFix, queue_size=1)


        if ROS:
            self.gps_sub = rospy.Subscriber('/fix',
                                        NavSatFix,
                                        self.gps_callback,
                                        queue_size=1)

        self.timer = QtCore.QBasicTimer()
        self.functionality.insertWidget(4,self.run_button)
    
    def start_timer(self):
        self.timer.start(1000, self)

    def timerEvent(self, event):
        if (event.timerId() != self.timer.timerId()):
            return

        if self.coords:
            frame = self.web.page().currentFrame()
            frame.documentElement().evaluateJavaScript("setCenter({},{})".format(*self.coords))
            frame.documentElement().evaluateJavaScript("addMarker({},{})".format(*self.coords))
            goal = frame.documentElement().evaluateJavaScript("get_goal()")
            print(goal)
            if goal and ROS:
                fix = NavSatFix()
                # lat is x
                fix.latitude = goal['lat']
                fix.longitude = goal['lng']
                fix.altitude = 9.3
                fix.header.frame_id = '/map'
                fix.header.stamp = rospy.Time.now()
                fix.header.seq = self.seq
                fix.position_covariance_type = 1
                self.seq += 1
                if ROS:
                    print(fix)
                    self.goal_pub.publish(fix)
        
    def gps_callback(self, data):
        self.coords = (data.latitude, data.longitude)

if __name__ == '__main__':
    if ROS:
        rospy.init_node('base_station_gui')
    
    #Qt initialization stuff
    #Create an object of the class QApplication to host everything
    app = QApplication(sys.argv)

    #form is an object of the class defined above
    bs = Basestation()

    #Show the UI elements
    bs.show()
    
    #Exit the application when the Qt window closes
    sys.exit(app.exec_())
