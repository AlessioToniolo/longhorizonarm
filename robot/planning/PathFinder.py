import numpy as np
from .Kinematics import Kinematics

class PathFinder:
    def __init__(self, kinematics: Kinematics):
        self.kinematics = kinematics
        
    def find_path(self, start_pos, end_pos, obstacle_pos, obstacle_size):
        mid = (start_pos + end_pos) / 2
        vec_to_target = end_pos - start_pos
        path_direction = vec_to_target / np.linalg.norm(vec_to_target)
        safety_margin = 50
        cleared_size = obstacle_size + safety_margin
        waypoint = mid.copy()
        if abs(path_direction[2]) < 0.8:
            waypoint[2] = obstacle_pos[2] + cleared_size
        else:
            if abs(start_pos[0]) > abs(start_pos[1]):
                waypoint[1] = obstacle_pos[1] + cleared_size
            else:
                waypoint[0] = obstacle_pos[0] + cleared_size
        return [start_pos, waypoint, end_pos]

    def execute_path(self, path, speed_factor=1.0):
        for point in path:
            self.kinematics.move_to_pos_profiled(point[0], point[1], point[2])

"""
next steps:
1. velocity profiling
2. proper collision checking
3. RRT
4. path interpolation (smooooth)
5. real-time obstacle avoidance?
"""