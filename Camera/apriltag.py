import cv2
from Camera.PnP_Define import *
from Calculate.calculate_function import *


class AprilTag_Camera():
    def __init__(self, i):
        self.cap = cv2.VideoCapture(i)

        self.kalman = cv2.KalmanFilter(4, 2)
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0],
                                     [0, 1, 0, 0]], np.float32)
        self.kalman.transitionMatrix = np.array([[1, 0, 1, 0],
                                    [0, 1, 0, 1],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]], np.float32)
        self.kalman.processNoiseCov = np.array([[1, 0, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 0.05, 0],
                                   [0, 0, 0, 0.05]], np.float32)
        self.kalman.measurementNoiseCov = np.array([[2, 0],
                                       [0, 2]], np.float32)
        
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    def image_processing(self, frame, detector, K_M, D_C):
        # 畸变矫正
        # undistorted_img = cv2.undistort(frame, K_M, D_C)
        # cv2.imshow("Un", undistorted_img)
        # 将图像转换为灰度图（一定要加，虽然是黑白摄像头，但是他返回的图像是三通道的）
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # 自适应直方图均衡
        gray = self.clahe.apply(gray)
        # 高斯模糊，减少噪声
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        # 检测 AprilTags
        results = detector.detect(gray)

        return results
    
    def apriltag_process(self, results, frame, K_M, D_C):
        for r in results:
            # 获取编号
            ID = r.tag_id
            print(ID)
            # 获取标签的中心位置
            tag_center = (int(r.center[0]), int(r.center[1]))
            # 更新卡尔曼滤波器
            self.kalman.correct(np.array([[np.float32(tag_center[0])], [np.float32(tag_center[1])]]))
            predicted = self.kalman.predict()
            predicted_center = (int(predicted[0]), int(predicted[1]))

            # 绘制预测的中心点和标签的边框
            cv2.circle(frame, predicted_center, 5, (0, 0, 255), -1)
            for i in range(4):
                pt1 = (int(r.corners[i][0]), int(r.corners[i][1]))
                pt2 = (int(r.corners[(i + 1) % 4][0]), int(r.corners[(i + 1) % 4][1]))
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

            # 显示标签ID
            cv2.putText(frame, str(r.tag_id), predicted_center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # 获取 AprilTag 在图像中的 2D 坐标
            image_points = np.array([
                [r.corners[0][0], r.corners[0][1]],
                [r.corners[1][0], r.corners[1][1]],
                [r.corners[2][0], r.corners[2][1]],
                [r.corners[3][0], r.corners[3][1]]
            ], dtype=np.float32)

            # 使用 PnP 算法估算相机到标签的距离
            success, rvec, tvec = cv2.solvePnP(tag_3d_points, image_points, K_M, D_C)
            
            if success:
                # 计算距离（tvec 是平移向量）
                distance = np.linalg.norm(tvec)
                print(f"Distance to AprilTag: {distance:.2f} meters")

                # 获取旋转矩阵
                rotation_matrix, _ = cv2.Rodrigues(rvec)

                # 世界坐标系中的 AprilTag 中心位置
                tag_position_world = np.dot(rotation_matrix, np.array([0, 0, 0], dtype=np.float32)) + tvec.flatten()
                print(f"AprilTag position in world coordinates: {tag_position_world}")
                print("tevc:", tvec)
                deg = calculate_theta(tvec[0], tvec[2])
                print("Theta:", deg)

                # 绘制相机位置到标签的向量
                cv2.putText(frame, f"Distance: {distance:.2f} m", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                return [ID, distance, tvec, rotation_matrix, deg]

    def close(self):
        # 释放摄像头并关闭窗口
        self.cap.release()
        cv2.destroyAllWindows()
        