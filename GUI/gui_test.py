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
from PyQt5.QtWebEngineWidgets import *
from basestation_ui import Ui_MainWindow
from imutils.object_detection import non_max_suppression
from imutils import paths
import numpy as np
import argparse
import imutils
import cv2
from os import path

# IMPORTANT
# To connect Python to Google Maps Js/HTML, follow this:
# https://stackoverflow.com/questions/47252632/how-to-pass-info-from-js-to-python-using-qwebchannel
# https://github.com/eyllanesc/stackoverflow/tree/master/47252632

maphtml = '''
<!DOCTYPE html>
<html>
  <head>
    <style>
      html, body, #map-canvas {
        height: 100%;
        margin: 0px;
        padding: 0px
      }
    </style>
    <script src="https://maps.googleapis.com/maps/api/js?v=3.exp&sensor=false&libraries=drawing"></script>
    <script>
      function initialize() {
        var mapOptions = {
          center: new google.maps.LatLng(28.6024556,-81.2023988,17),
          zoom: 16,
          mapTypeId: google.maps.MapTypeId.ROADMAP
        };

        var map = new google.maps.Map(document.getElementById('map-canvas'), mapOptions);

        var drawingManager = new google.maps.drawing.DrawingManager({
          drawingMode: google.maps.drawing.OverlayType.POLYGON,
          drawingControl: true,
          drawingControlOptions: {
            position: google.maps.ControlPosition.TOP_CENTER,
            drawingModes: [google.maps.drawing.OverlayType.POLYGON]
          },
          polygonOptions: {editable: true, draggable: true},
        });
        drawingManager.setMap(map);

        var thePolygon = null;
        var ucfPos = new google.maps.LatLng(28.6024556,-81.2023988,17);
        var robotPos = new google.maps.Marker();
        robotPos.setPosition(ucfPos);
        robotPos.setMap(map);

        google.maps.event.addListener(drawingManager, 'polygoncomplete', function (polygon) {
          if (thePolygon) 
            thePolygon.setMap(null);
          thePolygon = polygon;
          polygon.getPath().forEach(function (xy, i) {
            self.polygoncomplete(xy.lat(), xy.lng(), i);
          });
        });
      }

      google.maps.event.addDomListener(window, 'load', initialize);

    </script>
  </head>
  <body>
    <div id="map-canvas"></div>
  </body>
</html>
'''
class RecordVideo(QtCore.QObject):
    image_data = QtCore.pyqtSignal(np.ndarray)

    def __init__(self, camera_port=0, parent=None):
        super(RecordVideo, self).__init__(parent)
        self.camera = cv2.VideoCapture(camera_port)
        self.timer = QtCore.QBasicTimer()

    def start_recording(self):
        self.timer.start(0, self)

    def timerEvent(self, event):
        if (event.timerId() != self.timer.timerId()):
            return

        read, image = self.camera.read()
        if read:
            self.image_data.emit(cv2.imread('images/person_454.bmp'))

class PedestrianDetectionWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(PedestrianDetectionWidget, self).__init__(parent)
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        
        self.image = QtGui.QImage()
        self._red = (0, 0, 255)
        self._width = 2
        self._min_size = (30, 30)

    def detect_people(self, image):
        #image = imutils.resize(image, width=min(400, image.shape[1]))
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
            print('not size')
            print(self.size(), self.image.size())
            self.setFixedSize(self.image.size())

        self.update()

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
    def __init__(self, parent=None):
        super(MainWidget, self).__init__(parent)
        self.people_detection_widget = PedestrianDetectionWidget()

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
        self.web = QWebEngineView(self.gps_map)
        self.web.setHtml(maphtml)
        self.web.page().
        self.cv_widget = MainWidget()
        self.cv_widget.setFixedHeight(400)
        #self.cv_widget.setFixedWidth(400)
        self.functionality.insertWidget(1,self.cv_widget)
        
if __name__ == '__main__':
    #Qt initialization stuff
    #Create an object of the class QApplication to host everything
    app = QApplication(sys.argv)

    #form is an object of the class defined above
    bs = Basestation()

    #Show the UI elements
    bs.show()

    #Exit the application when the Qt window closes
    sys.exit(app.exec_())