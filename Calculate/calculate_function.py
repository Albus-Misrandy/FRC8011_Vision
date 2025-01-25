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

def check_id(apriltag_id):
    if apriltag_id == 17:
        apriltag_world = np.array([[Site_coordinates[0][0]], [Site_coordinates[0][1]], [0.3075]])
        rotate_theta = np.radians(240)
        return apriltag_world, rotate_theta, True
    elif apriltag_id == 18:
         apriltag_world = np.array([[Site_coordinates[1][0]], [Site_coordinates[1][1]], [0.3075]])
         rotate_theta = np.radians(180)
         return apriltag_world, rotate_theta, True
    elif apriltag_id == 19:
         apriltag_world = np.array([[Site_coordinates[2][0]], [Site_coordinates[2][1]], [0.3075]])
         rotate_theta = np.radians(120)
         return apriltag_world, rotate_theta, True
    elif apriltag_id == 20:
         apriltag_world = np.array([[Site_coordinates[3][0]], [Site_coordinates[3][1]], [0.3075]])
         rotate_theta = np.radians(60)
         return apriltag_world, rotate_theta, True
    elif apriltag_id == 21:
         apriltag_world = np.array([[Site_coordinates[4][0]], [Site_coordinates[4][1]], [0.3075]])
         rotate_theta = np.radians(0)
         return apriltag_world, rotate_theta, True
    elif apriltag_id == 22:
         apriltag_world = np.array([[Site_coordinates[5][0]], [Site_coordinates[5][1]], [0.3075]])
         rotate_theta = np.radians(300)
         return apriltag_world, rotate_theta, True
    else:
         apriltag_world = np.array([-1, -1, -1])
         rotate_theta = -1
         return apriltag_world, rotate_theta, False
    
def calculate_Rotation_Matrix(theta):
    Rotation_Matrix = np.array([[math.cos(theta), -math.sin(theta), 0],
                                [math.sin(theta), math.cos(theta), 0],
                                [0, 0, 1]])
    # rvec = np.array([0, 0, theta])
    # Rotation_Matrix, _ = cv2.Rodrigues(rvec)
    return Rotation_Matrix

def inverse_camera(tevc, R):
    R_T = np.transpose(R)
    tevc_inv = -R_T.dot(tevc)
    return tevc_inv

def corrcet_camera_coordinate(tevc):
    tevc[[1, 2], :] = tevc[[2, 1], :]
    tevc[0, :] = -tevc[0, :]
    tevc[1, :] = -tevc[1, :]
    return tevc

def calculate_car_position(R_T, cam_in_tag, tag_in_world):
    cam_coordinate = np.dot(R_T, (cam_in_tag - tag_in_world))
    return cam_coordinate
    