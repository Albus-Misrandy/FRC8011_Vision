"""
Author: Albus.Misrandy
"""
import base64
import sys
import time
import numpy as np
import cv2
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer
from PyQt5 import uic

from networktables import NetworkTables

NetworkTables.initialize(server='10.80.11.2')
time.sleep(1)

data_table = NetworkTables.getTable("SmartDashboard")
video_table = NetworkTables.getTable("MyVideo")
ui_table = NetworkTables.getTable("My_UI")


class FRCControlWindows(QWidget):
    def __init__(self):
        super().__init__()
        self.ui = None
        self.A = None
        self.B = None
        self.X = None
        self.Y = None
        self.Camera1_btn = None
        self.Camera2_btn = None
        self.status_browser = None
        self.button_browser = None
        self.speed_browser = None
        self.Camera1_browser = None
        self.Camera2_browser = None
        self.speed = None
        self.img_str1 = None
        self.img_str2 = None
        self.camera1_view = None
        self.camera2_view = None
        self.scene1 = QGraphicsScene()
        self.scene2 = QGraphicsScene()
        self.timer = QTimer()
        self.timer2 = QTimer()
        self.timer3 = QTimer()
        self.init_ui()

    def init_ui(self):
        # 加载UI界面
        self.ui = uic.loadUi("./UI_design/frc3.ui")
        # 定义控件
        self.A = self.ui.pushButton
        self.B = self.ui.pushButton_2
        self.X = self.ui.pushButton_3
        self.Y = self.ui.pushButton_4

        self.Camera1_btn = self.ui.pushButton_5
        self.Camera1_btn.setCheckable(True)
        self.Camera1_btn.setChecked(False)
        self.Camera2_btn = self.ui.pushButton_6
        self.Camera2_btn.setCheckable(True)
        self.Camera2_btn.setChecked(False)

        self.status_browser = self.ui.textBrowser
        self.status_browser.setStyleSheet("font-size: 22px;")
        self.button_browser = self.ui.textBrowser_2
        self.button_browser.setStyleSheet("font-size: 22px;")
        self.speed_browser = self.ui.textBrowser_3
        self.speed_browser.setStyleSheet("font-size: 22px;")
        self.Camera1_browser = self.ui.textBrowser_5
        self.Camera1_browser.setStyleSheet("font-size: 22px;")
        self.Camera2_browser = self.ui.textBrowser_6
        self.Camera2_browser.setStyleSheet("font-size: 22px;")

        self.speed = self.ui.verticalSlider
        self.speed.setMinimum(0)
        self.speed.setMaximum(100)

        self.camera1_view = self.ui.graphicsView
        self.camera1_view.setScene(self.scene1)
        self.camera2_view = self.ui.graphicsView_2
        self.camera2_view.setScene(self.scene2)

        # 信号与槽的绑定
        self.A.clicked.connect(self.A_clicked)
        self.B.clicked.connect(self.B_clicked)
        self.X.clicked.connect(self.X_clicked)
        self.Y.clicked.connect(self.Y_clicked)
        self.Camera1_btn.clicked.connect(self.camera1_btn_check)
        self.Camera2_btn.clicked.connect(self.camera2_btn_check)
        self.speed.valueChanged.connect(self.Speed_silder)
        self.timer.timeout.connect(self.monitor_connection)

        self.timer.start(1000)  # 每秒检查一次连接状态

    def A_clicked(self):
        data_table.getEntry("btn_id").setDouble(1)
        self.button_browser.setPlainText("A")

    def B_clicked(self):
        data_table.getEntry("btn_id").setDouble(2)
        self.button_browser.setPlainText("B")

    def X_clicked(self):
        data_table.getEntry("btn_id").setDouble(3)
        self.button_browser.setPlainText("X")

    def Y_clicked(self):
        data_table.getEntry("btn_id").setDouble(4)
        self.button_browser.setPlainText("Y")

    def camera1_btn_check(self):
        if self.Camera1_btn.isChecked():
            self.start_camera1()
        else:
            self.close_camera1()

    def camera2_btn_check(self):
        if self.Camera2_btn.isChecked():
            self.start_camera2()
        else:
            self.close_camera2()

    def Speed_silder(self, value):
        self.speed_browser.setPlainText(f"{value}")
        data_table.getEntry("Speed_Value").setDouble(value)

    def start_camera1(self):
        # if not NetworkTables.isConnected():
        #     self.Camera1_browser.setPlainText("Failed connected!")
        if not self.timer2.isActive():
            self.timer2.timeout.connect(self.update_frame1)

        self.timer2.start(50)

    def start_camera2(self):
        if not self.timer3.isActive():
            self.timer3.timeout.connect(self.update_frame2)

        self.timer3.start(50)

    def update_frame1(self):
        self.img_str1 = video_table.getString("CameraImage1", "")
        if self.img_str1 == "":
            self.Camera1_browser.setPlainText("Failed Opened!")
            return
        else:
            self.Camera1_browser.setPlainText("On")

        img_data1 = base64.b64decode(self.img_str1)
        img_arr1 = np.frombuffer(img_data1, np.uint8)
        frame1 = cv2.imdecode(img_arr1, cv2.IMREAD_COLOR)

        frame1 = cv2.resize(frame1, (self.camera1_view.width(), self.camera1_view.height()))
        frame1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB)

        h1, w1, c1 = frame1.shape
        q_img1 = QImage(frame1.data, w1, h1, c1 * w1, QImage.Format_RGB888)

        pixmap1 = QPixmap.fromImage(q_img1)

        self.scene1.clear()
        self.scene1.addPixmap(pixmap1)
        self.scene1.update()

    def update_frame2(self):
        self.img_str2 = video_table.getString("CameraImage2", "")
        if self.img_str2 == "":
            self.Camera2_browser.setPlainText("Failed Open!")
            return
        else:
            self.Camera2_browser.setPlainText("On")
        img_data2 = base64.b64decode(self.img_str2)
        img_arr2 = np.frombuffer(img_data2, np.uint8)
        frame2 = cv2.imdecode(img_arr2, cv2.IMREAD_COLOR)

        frame2 = cv2.resize(frame2, (self.camera2_view.width(), self.camera2_view.height()))
        frame2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2RGB)

        h2, w2, c2 = frame2.shape
        q_img2 = QImage(frame2.data, w2, h2, c2 * w2, QImage.Format_RGB888)

        pixmap2 = QPixmap.fromImage(q_img2)

        self.scene2.clear()
        self.scene2.addPixmap(pixmap2)
        self.scene2.update()

    def close_camera1(self):
        self.timer2.stop()
        self.img_str1 = ""
        self.scene1.clear()
        self.Camera1_browser.setPlainText("OFF")

    def close_camera2(self):
        self.timer3.stop()
        self.img_str2 = ""
        self.scene2.clear()
        self.Camera2_browser.setPlainText("OFF")

    def monitor_connection(self):
        if NetworkTables.isConnected():
            self.status_browser.setPlainText("Connected to NetworkTables!")
        else:
            self.status_browser.setPlainText("Disconnected from NetworkTables!")


if __name__ == '__main__':
    app = QApplication(sys.argv)

    w = FRCControlWindows()
    # 展示窗口
    w.ui.show()

    app.exec()