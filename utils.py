import math
import cv2

def check_AprilTag_ID(id, id_list):
    for i in id_list:
        if id == i:
            print("Fuck nice!")

def Put_Distance_txt(frame, distance):
    # 绘制相机位置到标签的向量
    cv2.putText(frame, f"Distance: {distance:.2f} m", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

def deg_to_rad(deg):
    return math.radians(deg)