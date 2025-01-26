import numpy as np

# 相机内参矩阵
fx_red = 744.03446982
fy_red = 730.63326546
cx_red = 335.24685303
cy_red = 266.72465222

fx_black = 734.82824424
fy_black = 716.99327962
cx_black = 360.33281319
cy_black = 251.27352276

K1 = np.array([
    [fx_red, 0, cx_red],
    [0, fy_red, cy_red],
    [0, 0, 1]
], dtype=np.float32)

K2 = np.array([
    [fx_black, 0, cx_black],
    [0, fy_black, cy_black],
    [0, 0, 1]
], dtype=np.float32)

# 初始化畸变系数（假设无畸变）
dist_coeffs = np.zeros((4, 1), dtype=np.float32)  # 无畸变
dist_coeffs_red = np.array([0.05655601, 0.18168838, -0.00282873, -0.00310767, -1.30886765], dtype=np.float32)
dist_coeffs_black = np.array([2.69155271e-01, -2.50473219e+00, -8.61186702e-04, 2.34220307e-03, 9.40521053e+00])

# 标签的 3D 坐标
tag_size = 0.165  # 标签边长，单位：米
tag_3d_points = np.array([
    [-tag_size / 2, -tag_size / 2, 0],
    [tag_size / 2, -tag_size / 2, 0],
    [tag_size / 2, tag_size / 2, 0],
    [-tag_size / 2, tag_size / 2, 0]
], dtype=np.float32)
