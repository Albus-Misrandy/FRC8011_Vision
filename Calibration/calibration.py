import cv2
import numpy as np

# 定义棋盘格的实际尺寸
square_size = 1  # 每个方格的实际大小，单位：厘米
objp = np.zeros((6 * 9, 3), np.float32)
objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)
objp = objp * square_size  # 根据实际尺寸调整

# 存储标定数据
obj_points = []  # 3D 点
img_points = []  # 2D 点

# 打开摄像头
cap = cv2.VideoCapture(0)

clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture image.")
        break
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    gray = clahe.apply(gray)
    
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # 检测棋盘格的角点
    ret, corners = cv2.findChessboardCorners(gray, (9, 6), None, flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK)
    
    if ret:
        # 如果检测到角点
        obj_points.append(objp)
        img_points.append(corners)
        
        # 绘制角点
        cv2.drawChessboardCorners(frame, (9, 6), corners, ret)
    
    # 每100帧进行标定计算
    if len(obj_points) > 10:
        # 相机标定
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)
        
        # 显示内参矩阵和畸变系数
        print("相机内参矩阵:")
        print(mtx)
        print("畸变系数:")
        print(dist)
        
        # 标定完成后，重新清空数据以准备下一轮标定
        obj_points.clear()
        img_points.clear()

    # 显示图像
    cv2.imshow('Chessboard Detection', frame)

    # 按下 'q' 键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头资源并关闭窗口
cap.release()
cv2.destroyAllWindows()
