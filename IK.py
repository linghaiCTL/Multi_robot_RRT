from bot import bot_param
from RRT import Node
import numpy as np
import math
from scipy.optimize import minimize

arm_l = [
    bot_param['arm1_l'],
    bot_param['arm2_l'],
    bot_param['arm3_l'],
]

def forward_kinematics(theta, origin):
    # 前向运动学
    x, y, z = origin
    angle = 0
    # arm1
    z=z+arm_l[0]
    face = theta[0]
    #arm2 and 3
    for i in range(1,3):
        angle += theta[i]
        dxy = arm_l[i] * np.cos(angle)
        y = -np.sin(face)*dxy+y
        x = -np.cos(face)*dxy+x
        dz = arm_l[i] * np.sin(angle)
        z+=dz
    return np.array([x, y, z])

def inverse_kinematics(origin, target, theta_init):
    # 逆向运动学
    def loss_fn(theta):
        end = forward_kinematics(theta, origin)
        return np.linalg.norm(end - target)
    res = minimize(loss_fn, theta_init, bounds=[(-np.pi, np.pi), (0, np.pi), (-np.pi, np.pi)])
    if res.success or res.fun < 1e-3:
        return res.x
    else:
        print("[[Error]] IK solve failed.")
        return None

# 弃用了
def compute_wheel_speeds(path, wheel_r=0.1, wheel_base=0.2, dt=0.1):
    v_l_list = []
    v_r_list = []
    theta_list = [0]

    for i in range(1, len(path)):
        x0, y0, _ = path[i - 1]
        x1, y1, _ = path[i]
        # 计算当前位置和前一位置的角度
        dx = x1 - x0
        dy = y1 - y0
        theta = np.arctan2(dy, dx)
        theta_list.append(theta)

        # 线速度 v
        distance = math.hypot(dx, dy)
        v = distance / dt

        # 角速度 omega
        if i >= 2:
            theta_prev = theta_list[-2]
            dtheta = (theta - theta_prev + math.pi) % (2 * math.pi) - math.pi  # wrap to [-pi, pi]
            omega = dtheta / dt
        else:
            omega = 0.0  # 初始时刻无法估计角速度

        # 计算左右轮线速度
        v_l = v - (wheel_base / 2.0) * omega
        v_r = v + (wheel_base / 2.0) * omega

        # 转换为轮子的角速度（rad/s）
        w_l = v_l / wheel_r
        w_r = v_r / wheel_r

        v_l_list.append(w_l)
        v_r_list.append(w_r)

    return v_l_list, v_r_list, theta_list


def IK_trace(bot_trace, rod_end_trace, init_theta):
    thetas = [init_theta]
    for i in range(1, len(bot_trace)):
        new_theta = inverse_kinematics(bot_trace[i], rod_end_trace[i], thetas[-1])
        if new_theta is None:
            print("[[Error]] Unavailable trace")
            return False, thetas
        thetas.append(new_theta)
    return True, thetas
    