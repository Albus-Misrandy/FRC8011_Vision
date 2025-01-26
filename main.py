import apriltag
from Camera.PnP_Define import *
from Camera.apriltag import *
from Calculate.calculate_function import *
from Networktables.Networktables import *

# 实例化Networktables通信类
Net = Networktables()

# 创建摄像头实例
camera_1 = AprilTag_Camera(0)
camera_2 = AprilTag_Camera(2)

# 创建 AprilTag 检测器
detector = apriltag.Detector()

while True:
    ret1, frame1 = camera_1.cap.read()
    ret2, frame2 = camera_2.cap.read()
    if not ret1:
        print("Error: Failed to capture image_1.")
        break
    if not ret2:
        print("Error: Failed to capture image_2.")

    # print("Success!")

    results1 = camera_1.image_processing(frame1, detector)
    results2 = camera_2.image_processing(frame2, detector)

    info1 = camera_1.apriltag_process(results1, frame1)
    info2 = camera_2.apriltag_process(results2, frame2)

    if info1:
        print("Fuck yes")
        # print("image_Points:", info1[1])
        apriltag_world, rotate_theta, is_apriltag = check_id(info1[0], 0.165)
        calculate_distance_yaw(frame1, tag_3d_points, info1[1], K1, dist_coeffs)
        cam1_points = calculate_camera_points(apriltag_world, info1[1], K1, dist_coeffs)
        yaw1, _, _ = calculate_yaw(tag_3d_points, info1[1], K1, dist_coeffs)
        # print("Points:", cam1_points)
        # print(cam1_points.shape)
        print("yaw:", yaw1)
        Net.send_double_data("AprilTag_ID", info1[0])
        vision_list = [cam1_points[0][0], cam1_points[1][0], yaw1]
        Net.send_double_array_data("Vision", vision_list)
    elif info2 and info1 == None:
        print("Fuck")
    else:
        print("MotherFucker")

    frame_str1 = Net.encode_video(frame1)
    frame_str2 = Net.encode_video(frame2)
    Net.send_video_data("CameraImage1", frame_str1)
    Net.send_video_data("CameraImage2", frame_str2)

    # 显示检测结果
    cv2.imshow("AprilTag Detection with Preprocessing and Kalman Filter", frame1)
    cv2.imshow("Camera_2", frame2)

    # 按下 'q' 键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头并关闭窗口
camera_1.close()
