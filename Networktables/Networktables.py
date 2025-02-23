import cv2
from networktables import NetworkTables
import numpy as np
import base64

NetworkTables.initialize(server='10.80.11.2')

class Networktables():
    def __init__(self):
        self.v_table_1 = NetworkTables.getTable("MyVideo1")
        self.v_table_2 = NetworkTables.getTable("MyVideo2")
        self.d_table = NetworkTables.getTable("SmartDashboard")
        self.ui_table = NetworkTables.getTable("My_UI")
        
    def send_double_data(self, d_name, d_value):
        # 用于监测和Networktables的连接
        if not NetworkTables.isConnected():
            print("Disconnect")
        self.d_table.getEntry(d_name).setDouble(d_value)
        # print("sended!")

    def send_double_array_data(self, darr_name, darr_value):
        self.d_table.getEntry(darr_name).setDoubleArray(darr_value)

    # def ui_double_data(self, uidata_name, uidata_value):
    #     self.ui_table = NetworkTables.getEntry(uidata_name).setDouble(uidata_value)

    def send_video_data_1(self, v_name, v_value):
        self.v_table_1.putString(v_name, v_value)

    def send_video_data_2(self, v_name, v_value):
        self.v_table_2.putString(v_name, v_value)

    def encode_video(self, img):
        JPEG_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
        _, buffer = cv2.imencode(".jpg", img, JPEG_param)
        img_str = base64.b64encode(buffer).decode('utf-8')
        return img_str
