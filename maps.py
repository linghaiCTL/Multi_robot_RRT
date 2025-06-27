import json
import os
import numpy as np 

class virtual_map:
    def __init__(self, map=None, start=None, goal=None, rod_length=None, start_pos=None, map_size=None, it1=None, it2=None, z=None):
        self.map = map
        self.start = start
        self.goal = goal
        self.rod_length = rod_length
        self.start_pos = start_pos
        self.z = z
        self.map_size = map_size
        self.it1=it1
        self.it2=it2

    def save(self, save_path):
        data = {
            'map': self.map.tolist(),
            'start': self.start.tolist(),
            'goal': self.goal.tolist(),
            'rod_length': self.rod_length,
            'start_pos': self.start_pos.tolist(),
            'z':self.z,
            'map_size':self.map_size,
            'it1':self.it1.tolist(),
            'it2':self.it2.tolist(),
        }
        with open(save_path, 'w') as f:
            json.dump(data, f)

    def load(self, load_path):
        with open(load_path, 'r') as f:
            data = json.load(f)
        self.map = np.array(data['map'])
        self.start = np.array(data['start'])
        self.goal = np.array(data['goal'])
        self.rod_length = data['rod_length']
        self.start_pos = np.array(data['start_pos'])
        self.z = data['z']
        self.map_size=data['map_size']
        self.it1 = np.array(data['it1'])
        self.it2 = np.array(data['it2'])
    
    def obs(self):
        return self.map, self.start, self.goal, self.rod_length, self.start_pos, self.z, self.map_size, self.it1, self.it2
    
if __name__ == '__main__':
    map_dir='maps'
    
    # map_name = 'large_map'
    
    # map = np.zeros((100, 100))
    # map[30:70, 50] = 1 
    # start = np.array([10, 10, 0])
    # start_pos = np.array([(4 - np.sqrt(2), 10,0), (16 + np.sqrt(2), 10,0)])
    # goal = np.array([90, 90, np.pi/2])
    # rod_length = 10
    # map_size = [100, 100, 100]
    # init_theta1 = np.array([np.pi, np.pi/4, -np.pi/4])
    # init_theta2 = np.array([0, np.pi/4, -np.pi/4])
    
    # map_name = 'small_map'
    
    # map = np.zeros((30, 30))
    # map[10:20, 15] = 1 
    # start = np.array([4, 4, 0])
    # start_pos = np.array([(3, 4, 0), (5, 4,0)])
    # goal = np.array([26, 26, 0])
    # rod_length = 2
    # map_size = [30, 30, 30]
    # init_theta1 = np.array([np.pi, np.pi/3, 2 *np.pi/3])
    # init_theta2 = np.array([0, np.pi/3, 2 * np.pi/3])
    # z = 2.5+np.sqrt(3)
    
    map_name = 'complex_map'
    
    map = np.zeros((100, 100))
    map[0:70, 30] = 1 
    map[30:100, 70] = 1 
    map[50, 40:60] = 1
    start = np.array([10, 10, 0])
    start_pos = np.array([(4 - np.sqrt(2), 10,0), (16 + np.sqrt(2), 10,0)])
    goal = np.array([90, 90, np.pi/2])
    rod_length = 10
    map_size = [100, 100, 100]
    init_theta1 = np.array([np.pi, np.pi/4, -np.pi/4])
    init_theta2 = np.array([0, np.pi/4, -np.pi/4])
    z = 2.5 + np.sqrt(2)
    
    new_map = virtual_map(map, start, goal, rod_length, start_pos, map_size=map_size, it1=init_theta1, it2=init_theta2, z = z)
    new_map.save(os.path.join(map_dir, map_name + '.json'))