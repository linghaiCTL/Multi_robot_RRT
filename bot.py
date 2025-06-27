import numpy as np 
from RRT import Node, rod_endpoints, is_collision

# 机器人参数
bot_param={
    'body_size':2,
    'wheel_r':0.1,
    'arm1_l':0.5,
    'arm2_l':2,
    'arm3_l':1,
}

def bot_endpoints(node):
    dx = np.cos(node.theta) * bot_param['body_size'] / 2
    dy = np.sin(node.theta) * bot_param['body_size'] / 2
    p1 = (node.x + dx, node.y + dy)
    p2 = (node.x - dy, node.y + dx)
    p3 = (node.x - dx, node.y - dy)
    p4 = (node.x + dy, node.y - dx)
    return p1, p2, p3, p4

def is_bot_collision(map, bot):
    h, w = map.shape
    p1, p2, p3, p4 = bot_endpoints(bot)

    def in_bounds(p):
        return 0 <= int(p[0]) < w and 0 <= int(p[1]) < h

    def is_free(p):
        x, y = int(round(p[0])), int(round(p[1]))
        if 0 <= x < w and 0 <= y < h:
            return map[y][x] == 0
        return False

    # 计算机器人的四个角
    node_list = [p1, p2, p3, p4]
    for i in range(4):
        j = (i+1) % 4
        pa = node_list[i]
        pb = node_list[j]
        for alpha in np.linspace(0, 1, 10):
            x = pa[0] * (1 - alpha) + pb[0] * alpha
            y = pa[1] * (1 - alpha) + pb[1] * alpha
            if not (in_bounds((x, y)) and is_free((x, y))):
                return True
    return False

# 只规划中心路径，bot角度在别处计算
def path_planner(map, stick_path, start_pos, z, rod_length=10):
    bot1_path = []
    bot2_path = []

    init_node = Node(stick_path[0][0], stick_path[0][1], stick_path[0][2])
    p1, p2 = rod_endpoints(init_node, rod_length)
    bot1_init = np.array([start_pos[0][0], start_pos[0][1]])
    bot2_init = np.array([start_pos[1][0], start_pos[1][1]])
    delta1 = bot1_init - np.array(p1)
    delta2 = bot2_init - np.array(p2)

    max_reach = np.sqrt((bot_param['arm2_l'] + bot_param['arm3_l'])**2 - (z-bot_param['body_size']-bot_param['arm1_l'])**2) - 0.15

    for step in stick_path:
        rod_node = Node(step[0], step[1], step[2])
        p1, p2 = rod_endpoints(rod_node, rod_length)

        # 目标小车位置
        target1 = np.array(p1) + delta1
        target2 = np.array(p2) + delta2

        # 小车节点
        bot1 = Node(target1[0], target1[1], step[2])
        bot2 = Node(target2[0], target2[1], step[2])

        def is_valid(bot, target):
            dist = np.linalg.norm(np.array([bot.x, bot.y]) - np.array(target))
            return dist <= max_reach and not is_bot_collision(map, bot)

        valid1 = is_valid(bot1, p1)
        valid2 = is_valid(bot2, p2)

        # 不合法则在局部搜索可行位姿
        if not valid1:
            found = False
            for _ in range(100):
                offset = np.random.uniform(-0.3, 0.3, 2)
                offset_len = np.random.uniform(0.1, max_reach, 1)
                offset = np.linalg.norm(offset)*offset_len
                new_pos = np.array(p1) + offset
                bot1_try = Node(new_pos[0], new_pos[1], step[2])
                if is_valid(bot1_try, p1):
                    bot1 = bot1_try
                    delta1 = offset  # 更新偏移
                    found = True
                    break
            if not found:
                print("[[Log]] Bot1 failed to find valid pose")
                return None, None

        if not valid2:
            found = False
            for _ in range(100):
                offset = np.random.uniform(-0.3, 0.3, 2)
                new_pos = np.array(p2) + offset
                bot2_try = Node(new_pos[0], new_pos[1], step[2])
                if is_valid(bot2_try, p2):
                    bot2 = bot2_try
                    delta2 = offset
                    found = True
                    break
            if not found:
                print("[[Log]] Bot2 failed to find valid pose")
                return None, None

        bot1_path.append((bot1.x, bot1.y, 0))
        bot2_path.append((bot2.x, bot2.y, 0))

    return bot1_path, bot2_path