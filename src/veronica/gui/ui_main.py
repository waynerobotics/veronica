# python script for GUI {main UI}

import sys
import time
import subprocess
from os import system
from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(797, 503)
        MainWindow.setStyleSheet("background-color : rgb(0, 89, 79);")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.frame = QtWidgets.QFrame(self.centralwidget)
        font = QtGui.QFont()
        font.setFamily("Tlwg Typo")
        font.setPointSize(12)
        self.frame.setFont(font)
        self.frame.setStyleSheet("QFrame{\n"
                                 "    background-color : rgb(0, 89, 79);\n"
                                 "    color : rgb(220, 220, 220); \n"
                                 "}")
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.dropdown = QtWidgets.QComboBox(self.frame)
        self.dropdown.setGeometry(QtCore.QRect(220, 210, 341, 61))
        font = QtGui.QFont()
        font.setFamily("Segoe UI")
        font.setPointSize(16)
        self.dropdown.setFont(font)
        self.dropdown.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.dropdown.setStyleSheet("background-color: rgb(141, 144, 147);\n"
                                    "color: rgb(238, 238, 236); ")
        self.dropdown.setObjectName("dropdown")
        self.dropdown.addItem("")
        self.dropdown.addItem("")
        self.dropdown.addItem("")
        self.dropdown.addItem("")
        self.dropdown.addItem("")
        self.dropdown.addItem("")
        self.dropdown.addItem("")
        self.dropdown.addItem("")
        self.label = QtWidgets.QLabel(self.frame)
        self.label.setGeometry(QtCore.QRect(30, 110, 731, 71))
        font = QtGui.QFont()
        font.setFamily("Noto Sans CJK SC")
        font.setPointSize(26)
        self.label.setFont(font)
        self.label.setStyleSheet("color: rgb(255, 200, 68);\n"
                                 "")
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.select = QtWidgets.QPushButton(self.frame)
        self.select.setGeometry(QtCore.QRect(350, 320, 131, 41))
        font = QtGui.QFont()
        font.setFamily("Serif")
        font.setPointSize(11)
        self.select.setFont(font)
        self.select.setStyleSheet("background-color: rgb(141, 144, 147);\n"
                                  "color: rgb(238, 238, 236); ")
        self.select.setObjectName("select")
        self.verticalLayout.addWidget(self.frame)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        # On button click execute selected dropdown item
        self.select.clicked.connect(self.change_dropdown)
        subprocess.Popen(["gnome-terminal", "--", "roscore"])

    # Logic for dropdown menu & executables(ex. launch a file or terminate GUI)
    def change_dropdown(self):
        text = str(self.dropdown.currentText())
        if text == 'Main Competition':
            # TODO: Fix Main Comp launch pipeline
            subprocess.Popen(
                ["roslaunch", "differential_drive", "veronica_drive_manual.launch"])
            time.sleep(1.0)
            print("Main Competition")
        elif text == 'Lane Mode':
            subprocess.Popen(["roslaunch", "veronica", "lane_mode.launch"])
            print("Lane Mode")
        elif text == 'Speed Mode':
            subprocess.Popen(["roslaunch", "veronica", "speed_mode.launch"])
            print("Speed Mode")
        elif text == 'Obstacle Mode':
            subprocess.Popen(["roslaunch", "veronica", "obstacle_mode.launch"])
            print("Obstacle Mode")
        elif text == 'Waypoint Mode':
            subprocess.Popen(["roslaunch", "veronica", "waypoint_mode.launch"])
            print("Waypoint Mode")
        elif text == 'Init Param Server':
            # TODO: Fix Param launch pipeline
            # subprocess.Popen(["roslaunch", "lane_follower","lane_follower_bag.launch"])
            print("Init Param Server")
        elif text == 'Status Check':
            # TODO: Fix Status launch pipeline
            # subprocess.Popen(["roslaunch", "lane_follower","lane_follower_bag.launch"])
            print("Status Check")
        elif text == 'Kill All':
            # TODO: Test master kill
            system("killall -9 rosmaster")
            time.sleep(1.0)
            print("All Nodes Down!")
            sys.exit()
        else:
            pass

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate(
            "MainWindow", "WSU Robotics Command Center"))
        self.dropdown.setCurrentText(
            _translate("MainWindow", "Main Competition"))
        self.dropdown.setItemText(0, _translate(
            "MainWindow", "Main Competition"))
        self.dropdown.setItemText(1, _translate("MainWindow", "Lane Mode"))
        self.dropdown.setItemText(2, _translate("MainWindow", "Speed Mode"))
        self.dropdown.setItemText(3, _translate("MainWindow", "Obstacle Mode"))
        self.dropdown.setItemText(4, _translate("MainWindow", "Waypoint Mode"))
        self.dropdown.setItemText(5, _translate(
            "MainWindow", "Init Param Server"))
        self.dropdown.setItemText(6, _translate("MainWindow", "Status Check"))
        self.dropdown.setItemText(7, _translate("MainWindow", "Kill All"))
        self.label.setText(_translate(
            "MainWindow", "<html><head/><body><p>SELECT &amp; LAUNCH A <span style=\" font-weight:600;\">MODE!</span></p></body></html>"))
        self.select.setText(_translate("MainWindow", "Select"))
