import math
import numpy as np
import cv2

Site_coordinates = [[4.148, 3.383], [3.750, 4.103], [4.148, 4.824], [4.980, 4.824], [5.396, 4.103], [4.980, 3.383]]

def calculate_theta(x, z):
        theta = math.atan2(x, z)
        theta_deg = math.degrees(theta)
        return theta_deg

def calculate_flat_distance(x, z):
      flat_distance = math.sqrt(x*x + z*z)
      return flat_distance

def check_id(apriltag_id, tag_size):
    if apriltag_id == 13:
        actual_x = tag_size * math.cos(np.radians(30))
        actual_y = tag_size * math.sin(np.radians(30))
        apriltag_world = np.array([
            [Site_coordinates[0][0] - 0.5 * actual_x, Site_coordinates[0][1] + actual_y * 0.5, 0.3075 + tag_size * 0.5], # 左上角
            [Site_coordinates[0][0] + 0.5 * actual_x, Site_coordinates[0][1] - actual_y * 0.5, 0.3075 + tag_size * 0.5], # 右上角
            [Site_coordinates[0][0] + 0.5 * actual_x, Site_coordinates[0][1] - actual_y * 0.5, 0.3075 - tag_size * 0.5], # 右下角
            [Site_coordinates[0][0] - 0.5 * actual_x, Site_coordinates[0][1] + actual_y * 0.5, 0.3075 - tag_size * 0.5]  # 左下角
        ])
        rotate_theta = np.radians(240)
        return apriltag_world, rotate_theta, True
    elif apriltag_id == 18:
        apriltag_world = np.array([
            [Site_coordinates[1][0], Site_coordinates[1][1] + tag_size * 0.5, 0.3075 + tag_size * 0.5], # 左上角
            [Site_coordinates[1][0], Site_coordinates[1][1] - tag_size * 0.5, 0.3075 + tag_size * 0.5], # 右上角
            [Site_coordinates[1][0], Site_coordinates[1][1] - tag_size * 0.5, 0.3075 - tag_size * 0.5], # 右下角
            [Site_coordinates[1][0], Site_coordinates[1][1] + tag_size * 0.5, 0.3075 - tag_size * 0.5]  # 左下角
        ])
        rotate_theta = np.radians(180)
        return apriltag_world, rotate_theta, True
    elif apriltag_id == 19:
        actual_x = tag_size * math.cos(np.radians(30))
        actual_y = tag_size * math.sin(np.radians(30))
        apriltag_world = np.array([
            [Site_coordinates[2][0] + actual_x * 0.5, Site_coordinates[2][1] + actual_y * 0.5, 0.3075 + tag_size * 0.5], # 左上角
            [Site_coordinates[2][0] - actual_x * 0.5, Site_coordinates[2][1] - actual_y * 0.5, 0.3075 + tag_size * 0.5], # 右上角
            [Site_coordinates[2][0] - actual_x * 0.5, Site_coordinates[2][1] - actual_y * 0.5, 0.3075 - tag_size * 0.5], # 右下角
            [Site_coordinates[2][0] + actual_x * 0.5, Site_coordinates[2][1] + actual_y * 0.5, 0.3075 - tag_size * 0.5]  # 左下角
        ])
        rotate_theta = np.radians(120)
        return apriltag_world, rotate_theta, True
    elif apriltag_id == 20:
        actual_x = tag_size * math.cos(np.radians(30))
        actual_y = tag_size * math.sin(np.radians(30))
        apriltag_world = np.array([
            [Site_coordinates[3][0] + 0.5 * actual_x, Site_coordinates[3][1] - actual_y * 0.5, 0.3075 + tag_size * 0.5], # 左上角
            [Site_coordinates[3][0] - 0.5 * actual_x, Site_coordinates[3][1] + actual_y * 0.5, 0.3075 + tag_size * 0.5], # 右上角
            [Site_coordinates[3][0] - 0.5 * actual_x, Site_coordinates[3][1] + actual_y * 0.5, 0.3075 - tag_size * 0.5], # 右下角
            [Site_coordinates[3][0] + 0.5 * actual_x, Site_coordinates[3][1] - actual_y * 0.5, 0.3075 - tag_size * 0.5]  # 左下角
        ])
        rotate_theta = np.radians(60)
        return apriltag_world, rotate_theta, True
    elif apriltag_id == 21:
        apriltag_world = np.array([
            [Site_coordinates[4][0], Site_coordinates[4][1] - tag_size * 0.5, 0.3075 + tag_size * 0.5], # 左上角
            [Site_coordinates[4][0], Site_coordinates[4][1] + tag_size * 0.5, 0.3075 + tag_size * 0.5], # 右上角
            [Site_coordinates[4][0], Site_coordinates[4][1] + tag_size * 0.5, 0.3075 - tag_size * 0.5], # 右下角
            [Site_coordinates[4][0], Site_coordinates[4][1] - tag_size * 0.5, 0.3075 - tag_size * 0.5]  # 左下角
        ])
        rotate_theta = np.radians(0)
        return apriltag_world, rotate_theta, True
    elif apriltag_id == 22:
        actual_x = tag_size * math.cos(np.radians(30))
        actual_y = tag_size * math.sin(np.radians(30))
        apriltag_world = np.array([
            [Site_coordinates[5][0] - actual_x * 0.5, Site_coordinates[2][1] - actual_y * 0.5, 0.3075 + tag_size * 0.5], # 左上角
            [Site_coordinates[5][0] + actual_x * 0.5, Site_coordinates[2][1] + actual_y * 0.5, 0.3075 + tag_size * 0.5], # 右上角
            [Site_coordinates[5][0] + actual_x * 0.5, Site_coordinates[2][1] + actual_y * 0.5, 0.3075 - tag_size * 0.5], # 右下角
            [Site_coordinates[5][0] - actual_x * 0.5, Site_coordinates[2][1] - actual_y * 0.5, 0.3075 - tag_size * 0.5]  # 左下角
        ])
        rotate_theta = np.radians(300)
        return apriltag_world, rotate_theta, True
    else:
         apriltag_world = np.array([
            [-1, -1, -1],
            [-1, -1, -1],
            [-1, -1, -1],
            [-1, -1, -1]
         ])
         rotate_theta = -1
         return apriltag_world, rotate_theta, False

def calculate_camera_points(tag_3d_points, image_points, K_M, D_C):
    # 使用 PnP 算法估算相机到标签的距离
    success, rvec, tvec = cv2.solvePnP(tag_3d_points, image_points, K_M, D_C)
            
    if success:
        # 获取旋转矩阵
        rotation_matrix, _ = cv2.Rodrigues(rvec)

        camera_coordinate = -rotation_matrix.T @ tvec

        return camera_coordinate
    
def calculate_car_points(cam_points, theta, cam_num):
    if cam_num == 1:
        R = np.array([
            [np.cos(math.radians(theta)), -np.sin(math.radians(theta))],
            [np.sin(math.radians(theta)), np.cos(math.radians(theta))]
        ])
        relative_position = np.array([-0.28452, -0.04908])
        rotated_position = np.dot(R, relative_position)
        car_points = rotated_position + cam_points
        return car_points
    if cam_num == 2:
        R = np.array([
            [np.cos(math.radians(theta)), -np.sin(math.radians(theta))],
            [np.sin(math.radians(theta)), np.cos(math.radians(theta))]
        ])
        relative_position = np.array([0.28452, -0.04908])
        rotated_position = np.dot(R, relative_position)
        car_points = rotated_position + cam_points
        return car_points
    
def calculate_distance(frame, tag_3d_points, image_points, K_M, D_C):
     success, rvec, tvec = cv2.solvePnP(tag_3d_points, image_points, K_M, D_C)

     if success:
        # 计算距离（tvec 是平移向量）
        distance = math.sqrt(tvec[0][0] ** 2 + tvec[2][0] ** 2)
        # print(f"Distance to AprilTag: {distance:.2f} meters")
        # print("rvec:", rvec)
        # print("tevc:", tvec)

        return distance, tvec

def calculate_yaw(tag_3d_points, image_points, K_M, D_C):
    _, rvec, _ = cv2.solvePnP(tag_3d_points, image_points, K_M, D_C)
    R, _ = cv2.Rodrigues(rvec)
    #从旋转矩阵提取欧拉角（Z-Y-X顺序，返回：Yaw, Pitch, Roll
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)

    singular = sy < 1e-6  # 判断是否接近奇异情况

    if not singular:
        pitch = np.arctan2(R[1, 0], R[0, 0])  # 围绕z轴
        yaw = np.arctan2(-R[2, 0], sy)    # 围绕y轴
        roll = np.arctan2(R[2, 1], R[2, 2]) # 围绕x轴
    else:
        yaw = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        roll = 0

    return np.degrees(yaw), np.degrees(pitch), np.degrees(roll)

def Solving_RTtriangle_degrees(b, c, yaw):
    theta_rad = math.acos(abs(b) / abs(c))
    theta_deg = math.degrees(theta_rad)
    if yaw > 0:
        return theta_deg
    if yaw <= 0:
        return -theta_deg

def Solving_triangle_sides(a, b, theta):
    theta_rad = math.radians(theta)
    c = math.sqrt(a ** 2 + b ** 2 - 2 * a * b * math.cos(theta_rad))
    return c

def Solving_Laws_of_Cosines_degrees(a, b, c):
    cos_theta_rad = (a ** 2 + b ** 2 - c ** 2)/(2 * a * b)
    theta_rad = math.acos(cos_theta_rad)
    theta_deg = math.degrees(theta_rad)
    return theta_deg

def Solving_RTtriangle_RTsides(theta_rad, c, yaw):
    a = math.cos(theta_rad) * c
    b = math.sin(theta_rad) * c
    if yaw > 0:
        return a, b
    if yaw <= 0:
        return -a, b