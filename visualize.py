import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation, FFMpegWriter, PillowWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from bot import bot_param
import time
import numpy as np

def smooth_path(path, num_interp=5):
    """ 对路径插值，使动画更顺畅"""
    path = np.array(path)
    new_path = []
    for i in range(len(path) - 1):
        p1 = path[i]
        p2 = path[i + 1]
        for alpha in np.linspace(0, 1, num_interp, endpoint=False):
            new_path.append((1 - alpha) * p1 + alpha * p2)
    new_path.append(path[-1])  # 末尾点补上
    return np.array(new_path)


def draw_cube(ax, center, size, color='r', alpha=0.6):
    """ 在ax上画一个立方体（表示小车） """
    x, y, z = center
    s = size / 2.0
    # 8个顶点
    vertices = np.array([
        [x - s, y - s, z],
        [x + s, y - s, z],
        [x + s, y + s, z],
        [x - s, y + s, z],
        [x - s, y - s, z + size],
        [x + s, y - s, z + size],
        [x + s, y + s, z + size],
        [x - s, y + s, z + size],
    ])
    # 六个面
    faces = [
        [vertices[j] for j in [0, 1, 2, 3]],
        [vertices[j] for j in [4, 5, 6, 7]],
        [vertices[j] for j in [0, 1, 5, 4]],
        [vertices[j] for j in [2, 3, 7, 6]],
        [vertices[j] for j in [1, 2, 6, 5]],
        [vertices[j] for j in [0, 3, 7, 4]]
    ]
    poly = Poly3DCollection(faces, facecolors=color, linewidths=0.5, edgecolors='k', alpha=alpha)
    ax.add_collection3d(poly)
    return poly

def draw_obstacles(ax, map_array, wall_height=0.5, cell_size=0.2):
    h, w = map_array.shape
    for y in range(h):
        for x in range(w):
            if map_array[y, x] == 1:
                wx = x * cell_size + cell_size / 2
                wy = y * cell_size + cell_size / 2
                ax.bar3d(wx, wy, 0, dx=cell_size, dy=cell_size, dz=wall_height,
                         color='gray', alpha=0.8, edgecolor='k')


def visualize(map, bot1_path, bot2_path, thetas1, thetas2, stick_path, z, start_point, goal_point, video_output_path=None, rod_length=10, map_size = [100,100,100]):

    # 机械臂每段长度
    arm_l = [bot_param['arm1_l'], bot_param['arm2_l'], bot_param['arm3_l']]

    # 设置 3D 图
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(0, map_size[0])
    ax.set_ylim(0, map_size[1])
    ax.set_zlim(0, map_size[2])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # 创建可更新的线条对象
    arm1_line, = ax.plot([], [], [], 'r-', lw=1, label='Arm 1')
    arm2_line, = ax.plot([], [], [], 'b-', lw=1, label='Arm 2')
    rod_line,  = ax.plot([], [], [], 'k-', lw=1, label='Rod')
    # 初始化立方体（小车）对象
    bot1_cube = draw_cube(ax, (0, 0, 0), size=0.2, color='r')
    bot2_cube = draw_cube(ax, (0, 0, 0), size=0.2, color='b')
    # 绘制障碍物
    draw_obstacles(ax, map, wall_height=3, cell_size=1)
    # 显示起点
    ax.scatter(*start_point, c='green', s=10, label='Start', marker='o')

    # 显示终点
    ax.scatter(*goal_point, c='red', s=10, label='Goal', marker='o')


    def update_cube(poly, center, size):
        """ 更新立方体位置 """
        x, y, z = center
        s = size / 2.0
        vertices = np.array([
            [x - s, y - s, z],
            [x + s, y - s, z],
            [x + s, y + s, z],
            [x - s, y + s, z],
            [x - s, y - s, z + size],
            [x + s, y - s, z + size],
            [x + s, y + s, z + size],
            [x - s, y + s, z + size],
        ])
        faces = [
            [vertices[j] for j in [0, 1, 2, 3]],
            [vertices[j] for j in [4, 5, 6, 7]],
            [vertices[j] for j in [0, 1, 5, 4]],
            [vertices[j] for j in [2, 3, 7, 6]],
            [vertices[j] for j in [1, 2, 6, 5]],
            [vertices[j] for j in [0, 3, 7, 4]]
        ]
        poly.set_verts(faces)
    
    def forward_kinematics(theta, origin):
        """ 计算机械臂段坐标点 """
        x, y, z = origin[0], origin[1], origin[2]
        points = [np.array([x, y, z])]
        angle = 0
        # arm1
        z = z + arm_l[0]
        face = theta[0]
        points.append(np.array([x, y, z]))
        # arm2,3
        for i in range(1,3):
            angle += theta[i]
            dxy = arm_l[i] * np.cos(angle)
            y = -np.sin(face)*dxy+y
            x = -np.cos(face)*dxy+x
            dz = arm_l[i] * np.sin(angle)
            z+=dz
            points.append(np.array([x, y, z]))
        return np.stack(points)
    
    def rod_endpoints(center, theta, length):
        dx = np.cos(theta) * length / 2
        dy = np.sin(theta) * length / 2
        p1 = (center[0] - dx, center[1] - dy, z)
        p2 = (center[0] + dx, center[1] + dy, z)
        return p1, p2

    def update(frame):
        # 小车路径与机械臂姿态
        bot1 = bot1_path[frame]
        bot2 = bot2_path[frame]
        theta1 = thetas1[frame]
        theta2 = thetas2[frame]
        rod_center = stick_path[frame]

        # 更新小车立方体
        update_cube(bot1_cube, (bot1[0], bot1[1], 1.0), size=0.2)
        update_cube(bot2_cube, (bot2[0], bot2[1], 1.0), size=0.2)

        # 正向运动学
        arm1_points = forward_kinematics(theta1, (bot1[0], bot1[1], 2))
        arm2_points = forward_kinematics(theta2, (bot2[0], bot2[1], 2))

        # 更新臂
        arm1_line.set_data(arm1_points[:, 0], arm1_points[:, 1])
        arm1_line.set_3d_properties(arm1_points[:, 2])

        arm2_line.set_data(arm2_points[:, 0], arm2_points[:, 1])
        arm2_line.set_3d_properties(arm2_points[:, 2])

        # 更新杆
        p1, p2 = rod_endpoints(rod_center, rod_center[2], rod_length)
        rod_line.set_data([p1[0], p2[0]], [p1[1], p2[1]])
        rod_line.set_3d_properties([p1[2], p2[2]])
        

        return arm1_line, arm2_line, rod_line, bot1_cube, bot2_cube

    ani = FuncAnimation(fig, update, frames=len(bot1_path), interval=10, blit=False)
    plt.legend()
    if video_output_path is not None:
        if video_output_path.endswith('mp4'):
            writer = FFMpegWriter(fps=30, metadata=dict(artist='Me'), bitrate=1800)
            ani.save(video_output_path, writer=writer)
        elif video_output_path.endswith('gif'):
            ani.save(video_output_path, writer=PillowWriter(fps=30))
        else:
            print('[[Error]] Unavailable video save format')
        return True
    return False


