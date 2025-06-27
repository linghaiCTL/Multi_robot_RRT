import numpy as np
import matplotlib.pyplot as plt
import math
import random


class Node:
    def __init__(self, x, y, theta, parent=None):
        self.x = x
        self.y = y
        self.theta = theta
        self.parent = parent

def distance(n1, n2):
    dx = n1.x - n2.x
    dy = n1.y - n2.y
    dtheta = min(abs(n1.theta - n2.theta), 2 * math.pi - abs(n1.theta - n2.theta))
    return np.hypot(dx, dy) + 0.5 * dtheta

def steer(from_node, to_node, step_size=1):
    dx = to_node.x - from_node.x
    dy = to_node.y - from_node.y
    dtheta = to_node.theta - from_node.theta

    dist = math.hypot(dx, dy)
    if dist == 0:
        return from_node

    ratio = min(step_size / dist, 5.0)
    new_x = from_node.x + ratio * dx
    new_y = from_node.y + ratio * dy
    new_theta = from_node.theta + ratio * dtheta
    return Node(new_x, new_y, new_theta, from_node)

def rod_endpoints(node, rod_length):
    dx = math.cos(node.theta) * rod_length / 2
    dy = math.sin(node.theta) * rod_length / 2
    p1 = (node.x - dx, node.y - dy)
    p2 = (node.x + dx, node.y + dy)
    return p1, p2

def is_collision(node, map, rod_length):
    h, w = map.shape
    p1, p2 = rod_endpoints(node, rod_length)

    def in_bounds(p):
        return 0 <= int(p[0]) < w and 0 <= int(p[1]) < h

    def is_free(p):
        x, y = int(round(p[0])), int(round(p[1]))
        if 0 <= x < w and 0 <= y < h:
            return map[y][x] == 0
        return False

    for alpha in np.linspace(0, 1, 10):
        x = p1[0] * (1 - alpha) + p2[0] * alpha
        y = p1[1] * (1 - alpha) + p2[1] * alpha
        if not (in_bounds((x, y)) and is_free((x, y))):
            return True
    return False

def check_path_collision(from_node, to_node, map, rod_length, num_checks=100):
    # 连杆碰撞检测
    for i in range(1, num_checks + 1):
        alpha = i / num_checks
        interp_x = from_node.x * (1 - alpha) + to_node.x * alpha
        interp_y = from_node.y * (1 - alpha) + to_node.y * alpha
        interp_theta = from_node.theta * (1 - alpha) + to_node.theta * alpha
        interp_node = Node(interp_x, interp_y, interp_theta)
        if is_collision(interp_node, map, rod_length):
            return True  # 有碰撞
    return False  # 无碰撞

def rrt_planner(map, start, goal, rod_length, step_size, max_iter=50000):
    start_node = Node(*start)
    goal_node = Node(*goal)
    nodes = [start_node]
    find_end = False

    for _ in range(max_iter):
        if random.random() < 0.01:
            rand = goal_node
        else:
            x = random.uniform(0, map.shape[1])
            y = random.uniform(0, map.shape[0])
            theta = random.uniform(0, 2 * math.pi)
            rand = Node(x, y, theta)

        nearest = min(nodes, key=lambda n: distance(n, rand))
        new_node = steer(nearest, rand, step_size)

        if is_collision(new_node, map, rod_length):
            continue
        if check_path_collision(nearest, new_node, map, rod_length):
            continue

        nodes.append(new_node)

        # 终点判断
        if distance(new_node, goal_node) < 5 and not is_collision(goal_node, map, rod_length):
            goal_node.parent = new_node
            nodes.append(goal_node)
            find_end = True
            break

    if not find_end:
        return None
    path = []
    node = goal_node
    while node is not None:
        path.append((node.x, node.y, node.theta))
        node = node.parent
    path.reverse()
    return path


if __name__ == "__main__":
    #  100x100 的地图
    map = np.zeros((100, 100))
    map[30:70, 50] = 1 

    start = (10, 10, 0)
    goal = (90, 90, np.pi/2)
    rod_length = 10

    path = rrt_planner(map, start, goal, rod_length)

    # 可视化
    plt.imshow(map, cmap='gray_r')
    for (x, y, theta) in path:
        dx = math.cos(theta) * rod_length / 2
        dy = math.sin(theta) * rod_length / 2
        plt.plot([x - dx, x + dx], [y - dy, y + dy], 'b-')
        plt.plot(x, y, 'ro', markersize=2)
    plt.title("RRT Path for Rod")
    plt.show()
