import apriltag
from utils import *
from Camera.PnP_Define import *
from Camera.apriltag import *
from Calculate.calculate_function import *
from Networktables.Networktables import *

cam_to_center = 0.28872
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
        d1, t1 = calculate_distance(frame1, tag_3d_points, info1[1], K1, dist_coeffs)
        if is_apriltag == True:
            cam1_points = calculate_camera_points(apriltag_world, info1[1], K1, dist_coeffs)
        car_points = calculate_car_points(np.array([cam1_points[0][0], cam1_points[1][0]]), -20, 1)
        yaw1, _, _ = calculate_yaw(tag_3d_points, info1[1], K1, dist_coeffs)
        yaw1 = yaw1 - 20
        arpha = Solving_RTtriangle_degrees(t1[2][0], d1, yaw1)
        beta = 90 - arpha + 9.788  # 22.22为固定角度
        d = Solving_triangle_sides(d1, cam_to_center, beta)
        gama = 150.21 - Solving_Laws_of_Cosines_degrees(cam_to_center, d, d1)
        offset_y, offset_x = Solving_RTtriangle_RTsides(deg_to_rad(gama), d, yaw1)
        Put_Distance_txt(frame1, d)
        # print("distance:", d)
        # print("yaw1:", yaw1)
        Net.send_double_data("AprilTag_ID", info1[0])
        Net.send_double_data("yaw", yaw1)
        Net.send_double_array_data("OffsetXYD", [offset_x, offset_y, d])
        vision_list = [car_points[0], car_points[1], yaw1]
        print(vision_list)
        Net.send_double_array_data("Vision", vision_list)
    elif info2 and info1 == None:
        print("Fuck")
        apriltag_world, rotate_theta, is_apriltag = check_id(info2[0], 0.165)
        d2, t2 = calculate_distance(frame2, tag_3d_points, info2[1], K2, dist_coeffs)
        if is_apriltag == True:
            cam2_points = calculate_camera_points(apriltag_world, info2[1], K2, dist_coeffs)
        car_points = calculate_car_points(np.array([cam2_points[0][0], cam2_points[1][0]]), 20, 2)
        yaw2, _, _ = calculate_yaw(tag_3d_points, info2[1], K2, dist_coeffs)
        yaw2 = yaw2 + 20
        arpha = Solving_RTtriangle_degrees(t2[2][0], d2, yaw2)
        beta = 90 + arpha + 9.788
        d = Solving_triangle_sides(d2, cam_to_center, beta)
        gama = 42.78 + Solving_Laws_of_Cosines_degrees(cam_to_center, d, d2)
        offset_y, offset_x = Solving_RTtriangle_RTsides(deg_to_rad(gama), d, yaw2)
        print("yaw2:", yaw2)
        Net.send_double_data("AprilTag_ID", info2[0])
        Net.send_double_data("yaw", yaw2)
        Net.send_double_array_data("OffsetXYD", [offset_x, offset_y, d])
        vision_list = [car_points[0], car_points[1], yaw2]
        Net.send_double_array_data("Vision", vision_list)

    else:
        # print("MotherFucker")
        Net.send_double_data("AprilTag_ID", 100)

    frame_str1 = Net.encode_video(frame1)
    frame_str2 = Net.encode_video(frame2)
    Net.send_video_data_1("CameraImage1", frame_str1)
    Net.send_video_data_2("CameraImage2", frame_str2)

    # 显示检测结果
    cv2.imshow("Camera_1", frame1)
    cv2.imshow("Camera_2", frame2)

    # 按下 'q' 键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头并关闭窗口
camera_1.close()
