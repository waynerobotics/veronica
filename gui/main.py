#!/usr/bin/env python3
# Main GUI Application
import sys
import platform
import time
import subprocess
from os import system
from PyQt5 import QtCore, QtGui, QtWidgets

# SPLASH SCREEN
from ui_splash_screen import Ui_SplashScreen
# MAIN WINDOW
from ui_main import Ui_MainWindow
# GLOBALS
counter = 0


# MAIN APPLICATION
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)


# SPLASH SCREEN
class SplashScreen(QtWidgets.QMainWindow):
    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)
        self.ui = Ui_SplashScreen()
        self.ui.setupUi(self)
        # TIMER ==> START
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.progress)
        # TIMER IN MILLISECONDS
        self.timer.start(32)
        # CHANGE DESCRIPTION: Initial Text
        self.ui.label_description.setText("<strong>IGVC</strong> 2021")
        # Change Texts
        QtCore.QTimer.singleShot(1500, lambda: self.ui.label_description.setText("<strong>LOADING</strong> USER INTERFACE"))
        QtCore.QTimer.singleShot(2000, lambda: self.ui.label_description.setText("<strong>LOADING</strong>  PACKAGES"))
        QtCore.QTimer.singleShot(2900, lambda: self.ui.label_description.setText("<strong>LOADING</strong>  WORKSPACE"))
        QtCore.QTimer.singleShot(3200, lambda: self.ui.label_description.setText("<strong>DONE</strong> "))
        # SHOW ==> MAIN WINDOW
        self.show()
        ## ==> END ##

    # APP FUNCTIONS
    def progress(self):
        global counter
        # SET VALUE TO PROGRESS BAR
        self.ui.progressBar.setValue(counter)
        # CLOSE SPLASH SCREE AND OPEN APP
        if counter > 100:
            # STOP TIMER
            self.timer.stop()
            # SHOW MAIN WINDOW
            self.main = MainWindow()
            self.main.show()
            # CLOSE SPLASH SCREEN
            self.close()
        # INCREASE COUNTER
        counter += 1


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = SplashScreen()
    sys.exit(app.exec_())
