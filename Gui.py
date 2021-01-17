'''
Main GUI for Veronica 
Complete with all launch piplines for each competition mode
Nnamdi Monwe 
1/5/2021

'''

#!/usr/bin/env python3

import time
import subprocess
from os import system
from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(797, 503)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.dropdown = QtWidgets.QComboBox(self.centralwidget)
        self.dropdown.setGeometry(QtCore.QRect(240, 190, 321, 61))
        font = QtGui.QFont()
        font.setFamily("Segoe UI")
        font.setPointSize(16)
        self.dropdown.setFont(font)
        self.dropdown.setObjectName("dropdown")
        self.dropdown.addItem("")
        self.dropdown.addItem("")
        self.dropdown.addItem("")
        self.dropdown.addItem("")
        self.dropdown.addItem("")
        self.dropdown.addItem("")
        self.dropdown.addItem("")
        self.dropdown.addItem("")
        self.select = QtWidgets.QPushButton(self.centralwidget)
        self.select.setGeometry(QtCore.QRect(340, 310, 131, 41))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.select.setFont(font)
        self.select.setObjectName("select")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(310, 100, 191, 71))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label.setFont(font)
        self.label.setObjectName("label")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 797, 26))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        # On button click execute selected dropdown item
        self.select.clicked.connect(self.change_dropdown)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate(
            "MainWindow", "WSU Robotics Command Center"))
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
        self.select.setText(_translate("MainWindow", "Select"))
        self.label.setText(_translate("MainWindow", "Choose an option!"))

    # Logic for dropdown menu & executables(ex. launch a file or terminate GUI)
    def change_dropdown(self):
        text = str(self.dropdown.currentText())
        if text == 'Main Competition':
            # TODO: Fix Main Comp launch pipeline
            subprocess.Popen(
                ["roslaunch", "differential_drive", "veronica_drive_manual.launch"])
            #subprocess.Popen(["roslaunch", "lane_follower","lane_follower_bag.launch"])
            print("Main Competition")
        elif text == 'Lane Mode':
            # TODO: Fix lane launch pipeline
            subprocess.Popen(["roslaunch", "lane_follower",
                              "lane_follower_bag.launch"])
            print("Lane Mode")
        elif text == 'Speed Mode':
            # TODO: Fix Speed launch pipeline
            # subprocess.Popen(["roslaunch", "lane_follower","lane_follower_bag.launch"])
            print("Speed Mode")
        elif text == 'Obstacle Mode':
            # TODO: Fix Obstacle launch pipeline
            # subprocess.Popen(["roslaunch", "lane_follower","lane_follower_bag.launch"])
            print("Obstacle Mode")
        elif text == 'Waypoint Mode':
            # TODO: Fix Waypoint launch pipeline
            # subprocess.Popen(["roslaunch", "lane_follower","lane_follower_bag.launch"])
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
            time.sleep(0.5)
            print("All Nodes Down!")
            sys.exit()
        else:
            pass
 # Clean


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

# CMD pipeline (pyuic5 -x Gui.ui -o Gui.py)
