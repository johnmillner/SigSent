# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'basestation.ui'
#
# Created by: PyQt5 UI code generator 5.9
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1126, 892)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(10, 10, 1101, 861))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setSpacing(6)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.functionality = QtWidgets.QVBoxLayout()
        self.functionality.setObjectName("functionality")
        self.camera_label = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.camera_label.setMaximumSize(QtCore.QSize(16777215, 25))
        self.camera_label.setAlignment(QtCore.Qt.AlignCenter)
        self.camera_label.setObjectName("camera_label")
        self.functionality.addWidget(self.camera_label)
        self.battery_label = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.battery_label.setMaximumSize(QtCore.QSize(16777215, 25))
        self.battery_label.setAlignment(QtCore.Qt.AlignCenter)
        self.battery_label.setObjectName("battery_label")
        self.functionality.addWidget(self.battery_label)
        self.progressBar = QtWidgets.QProgressBar(self.horizontalLayoutWidget)
        self.progressBar.setProperty("value", 24)
        self.progressBar.setObjectName("progressBar")
        self.functionality.addWidget(self.progressBar)
        self.neat_label = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.neat_label.setMaximumSize(QtCore.QSize(16777215, 25))
        self.neat_label.setAlignment(QtCore.Qt.AlignCenter)
        self.neat_label.setObjectName("neat_label")
        self.functionality.addWidget(self.neat_label)
        self.cv_label = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.cv_label.setMaximumSize(QtCore.QSize(16777215, 25))
        self.cv_label.setAlignment(QtCore.Qt.AlignCenter)
        self.cv_label.setObjectName("cv_label")
        self.functionality.addWidget(self.cv_label)
        self.horizontalLayout.addLayout(self.functionality)
        self.maps_layout = QtWidgets.QVBoxLayout()
        self.maps_layout.setObjectName("maps_layout")
        self.gps_label = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.gps_label.setMaximumSize(QtCore.QSize(16777215, 25))
        self.gps_label.setAlignment(QtCore.Qt.AlignCenter)
        self.gps_label.setObjectName("gps_label")
        self.maps_layout.addWidget(self.gps_label)
        self.gps_map = QtWidgets.QWidget(self.horizontalLayoutWidget)
        self.gps_map.setMaximumSize(QtCore.QSize(16777215, 400))
        self.gps_map.setObjectName("gps_map")
        self.maps_layout.addWidget(self.gps_map)
        self.coords_label = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.coords_label.setMaximumSize(QtCore.QSize(16777215, 25))
        self.coords_label.setAlignment(QtCore.Qt.AlignCenter)
        self.coords_label.setObjectName("coords_label")
        self.maps_layout.addWidget(self.coords_label)
        self.horizontalLayout.addLayout(self.maps_layout)
        self.horizontalLayout.setStretch(0, 1)
        self.horizontalLayout.setStretch(1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.camera_label.setText(_translate("MainWindow", "Camera Feed"))
        self.battery_label.setText(_translate("MainWindow", "Battery"))
        self.neat_label.setText(_translate("MainWindow", "NEAT Output: Crutches Engaged"))
        self.cv_label.setText(_translate("MainWindow", "Pedestrian Detected: YES"))
        self.gps_label.setText(_translate("MainWindow", "GPS"))
        self.coords_label.setText(_translate("MainWindow", "Longitude: 83.77 Latitude: 42.12"))

