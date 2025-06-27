from RRT import rrt_planner
from maps import virtual_map
import bot
import IK
import visualize
import numpy as np
import matplotlib.pyplot as plt
import math
import os
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description="Parse map name and save directory.")
    parser.add_argument('--map_name', type=str, default="large_map", help='Name of the map')
    parser.add_argument('--output_dir', type=str, default=None, help='The directory to save output results')
    parser.add_argument('--smooth_degree', type=int, default=2, help='The smooth degree of the final result, the higher this param, the smoother the result, the longer the time cost for IK calculating')
    return parser.parse_args()


if __name__ == '__main__':
    # 一、解析参数，加载地图
    args = parse_args()
    map_dir = 'maps'
    map_name = args.map_name
    output_dir = args.output_dir
    if output_dir is None:
        output_dir = map_name
    map_path = os.path.join(map_dir, map_name + '.json')
    vmap = virtual_map()
    vmap.load(map_path)
    map, start, goal, rod_length, start_pos, z, map_size, init_theta1, init_theta2 = vmap.obs()
    output_dir = os.path.join('results', output_dir)
    os.makedirs(output_dir, exist_ok=True)
    
    # 二、路径规划
    print(f'[[Log]] Performing RRT, for some difficult cases, this might take up about a minute')
    # 因为使用的是随机算法，可能无法找到路径, 设定重复10次尝试
    for i in range(10):
        # (1).RRT算法规划搬运物体的路径
        RRT_step_size = map_size[0]/100
        path = rrt_planner(map, start, goal, rod_length, RRT_step_size)
        
        # (2).计算可行的bot路径
        bot1_path, bot2_path = bot.path_planner(map, path, start_pos, z, rod_length)
        
        if bot1_path is not None:
            # 找到可行路径
            break
        print('[[Log]] Try again to find an available path')
    if bot1_path is None:
        print('[[Error]] Exceed max retry times, fail to find a path in this map')
        # exit(0)
        
    # (3).可视化规划的路径
    output_path = os.path.join(output_dir, 'path.png')
    plt.imshow(map, cmap='gray_r')
    for i, (x, y, theta) in enumerate(path):
        dx = math.cos(theta) * rod_length / 2
        dy = math.sin(theta) * rod_length / 2
        plt.plot([x - dx, x + dx], [y - dy, y + dy], 'b-', label='Rod' if i == 0 else "")
        plt.plot(x, y, 'g.', markersize=2, label='Rod midpoint' if i == 0 else "")
    plt.legend()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    for i, (x, y, _) in enumerate(bot1_path):
        plt.plot(x, y, 'r.', label='Bot1' if i == 0 else "")
    for i, (x, y, _) in enumerate(bot2_path):
        plt.plot(x, y, 'b.', label='Bot1' if i == 0 else "")

    plt.title("RRT Path for Rod")
    plt.legend()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f'[[Log]] RRT is done. The figure showing the planned path can be find at {output_path}')

    # 三、计算运动控制参数
    # (1).对轨迹插值使之更顺畅
    path = visualize.smooth_path(path)
    bot1_path = visualize.smooth_path(bot1_path)
    bot2_path = visualize.smooth_path(bot2_path)
    # (2).计算机械臂末端轨迹
    rodend1=[]
    rodend2=[]
    for (x, y, theta) in path:
        dx = math.cos(theta) * rod_length / 2
        dy = math.sin(theta) * rod_length / 2
        rodend1.append(np.array([x-dx,y-dy, z]))
        rodend2.append(np.array([x+dx,y+dy, z]))
    #（3）IK计算机械臂参数
    print('[[Log]] Now performing IK calculate, this may take about 20 to 300 seconds')
    np_bot1=[]
    np_bot2=[]
    for (x, y, _) in bot1_path:
        np_bot1.append(np.array([x,y,2]))
    for (x, y, _) in bot2_path:
        np_bot2.append(np.array([x,y,2]))
    flag, thetas1 = IK.IK_trace(np_bot1, rodend1, init_theta1)
    flag, thetas2 = IK.IK_trace(np_bot2, rodend2, init_theta2)
    
    # 四、结果可视化
    output_path = os.path.join(output_dir, 'trace.gif')
    flag = visualize.visualize(map, bot1_path, bot2_path, thetas1, thetas2, path, z, [start[0],start[1],0], [goal[0], goal[1], 0], output_path, rod_length, map_size)
    if flag:
        print(f'[[Log]] Animation has done, the output video can be find at {output_path}')
    else:
        print(f'[[Error]] Something went wrong, please check whether the output path isn\' exist or it is none. If is something wrong in IK, please just simply try again')