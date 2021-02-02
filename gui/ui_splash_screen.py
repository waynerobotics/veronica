# python script for GUI {splash screen UI}

from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_SplashScreen(object):
    def setupUi(self, SplashScreen):
        SplashScreen.setObjectName("SplashScreen")
        SplashScreen.resize(640, 400)
        SplashScreen.setStyleSheet("    background-color : rgb(0, 89, 79);")
        self.centralwidget = QtWidgets.QWidget(SplashScreen)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setContentsMargins(10, 10, 10, 10)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.dropShadowFrame = QtWidgets.QFrame(self.centralwidget)
        self.dropShadowFrame.setStyleSheet("QFrame{\n"
                                           "    background-color : rgb(0, 89, 79);\n"
                                           "    color : rgb(220, 220, 220); \n"
                                           "}")
        self.dropShadowFrame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.dropShadowFrame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.dropShadowFrame.setObjectName("dropShadowFrame")
        self.label_title = QtWidgets.QLabel(self.dropShadowFrame)
        self.label_title.setGeometry(QtCore.QRect(0, 50, 621, 81))
        font = QtGui.QFont()
        font.setFamily("OpenSymbol")
        font.setPointSize(40)
        self.label_title.setFont(font)
        self.label_title.setStyleSheet("color: rgb(255, 200, 68);")
        self.label_title.setAlignment(QtCore.Qt.AlignCenter)
        self.label_title.setObjectName("label_title")
        self.label_description = QtWidgets.QLabel(self.dropShadowFrame)
        self.label_description.setGeometry(QtCore.QRect(0, 110, 621, 41))
        font = QtGui.QFont()
        font.setFamily("OpenSymbol")
        font.setPointSize(14)
        self.label_description.setFont(font)
        self.label_description.setStyleSheet("color: rgb(255, 255, 255);")
        self.label_description.setAlignment(QtCore.Qt.AlignCenter)
        self.label_description.setObjectName("label_description")
        self.progressBar = QtWidgets.QProgressBar(self.dropShadowFrame)
        self.progressBar.setGeometry(QtCore.QRect(30, 200, 561, 23))
        self.progressBar.setStyleSheet("QProgressBar{\n"
                                       "    background-color : rgb(141, 144, 147);\n"
                                       "    color: rgb(255, 255, 255);\n"
                                       "    border-style: none;\n"
                                       "    border-radius: 10px;\n"
                                       "    text-align: center;\n"
                                       "}\n"
                                       "\n"
                                       "QProgressBar::chunk{\n"
                                       "    border-radius: 10px;\n"
                                       "    background-color: qlineargradient(spread:pad, x1:0, y1:0.511, x2:1, y2:0.54, stop:0 rgba(255, 200, 68, 255), stop:1 rgba(180, 252, 124, 255));\n"
                                       "}")
        self.progressBar.setProperty("value", 24)
        self.progressBar.setObjectName("progressBar")
        self.label_loading = QtWidgets.QLabel(self.dropShadowFrame)
        self.label_loading.setGeometry(QtCore.QRect(0, 230, 621, 41))
        font = QtGui.QFont()
        font.setFamily("OpenSymbol")
        font.setPointSize(12)
        self.label_loading.setFont(font)
        self.label_loading.setStyleSheet("color: rgb(255, 255, 255);")
        self.label_loading.setAlignment(QtCore.Qt.AlignCenter)
        self.label_loading.setObjectName("label_loading")
        self.label_credits = QtWidgets.QLabel(self.dropShadowFrame)
        self.label_credits.setGeometry(QtCore.QRect(0, 340, 591, 41))
        font = QtGui.QFont()
        font.setFamily("OpenSymbol")
        font.setPointSize(10)
        self.label_credits.setFont(font)
        self.label_credits.setStyleSheet("color: rgb(255, 200, 68);")
        self.label_credits.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignTrailing | QtCore.Qt.AlignVCenter)
        self.label_credits.setObjectName("label_credits")
        self.verticalLayout.addWidget(self.dropShadowFrame)
        SplashScreen.setCentralWidget(self.centralwidget)

        self.retranslateUi(SplashScreen)
        QtCore.QMetaObject.connectSlotsByName(SplashScreen)

    def retranslateUi(self, SplashScreen):
        _translate = QtCore.QCoreApplication.translate
        SplashScreen.setWindowTitle(_translate("SplashScreen", "MainWindow"))
        self.label_title.setText(_translate("SplashScreen", "VERONICA <strong>GUI</strong> "))
        self.label_description.setText(_translate("SplashScreen", "<strong>IGVC</strong> 2021 "))
        self.label_loading.setText(_translate("SplashScreen", "loading...."))
        self.label_credits.setText(_translate("SplashScreen", "<strong>By:</strong>Warrior Robotics"))
