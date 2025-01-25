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

    results1 = camera_1.image_processing(frame1, detector, K1, dist_coeffs)
    results2 = camera_2.image_processing(frame2, detector, K2, dist_coeffs)

    info1 = camera_1.apriltag_process(results1, frame1, K1, dist_coeffs)
    info2 = camera_2.apriltag_process(results2, frame2, K2, dist_coeffs)

    if info1:
        print("Fuck yes")
        apriltag_world, rotate_theta, is_apriltag = check_id(info1[0])
        if is_apriltag:
            R_w2c = calculate_Rotation_Matrix(rotate_theta)
            R_c2w = np.transpose(R_w2c)
            # cam_in_tag_coordinate = inverse_camera(info1[2], info1[3])
            cam_in_tag_coordinate = corrcet_camera_coordinate(info1[2])
            cam_position = calculate_car_position(R_c2w, cam_in_tag_coordinate, apriltag_world)
            print("R:", R_w2c)
            print("Camera_coordinate:", cam_position)
            print("cam_in_tag:", cam_in_tag_coordinate)
            print("apriltag_world:", apriltag_world)
        # calculate_car_position(is_apriltag, apriltag_world, info1[2], info1[3], info1[4])
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
