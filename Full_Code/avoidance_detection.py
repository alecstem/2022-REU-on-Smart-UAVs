"""
Contains functions for avoidance path calculation.
"""
import copy

import numpy as np


def euclidean(best_avoidance, goal):
    val = np.sqrt((goal[0]-best_avoidance[0])*(goal[0]-best_avoidance[0]) +
                  (goal[1]-best_avoidance[1])*(goal[1]-best_avoidance[1]) +
                  (goal[2]-best_avoidance[2])*(goal[2]-best_avoidance[2]))
    return val


def find_avoidances(matrix, best_path):
    # x y z
    map_size = [len(matrix[0][0]), len(matrix[0]), len(matrix)]
    dirs = [[1, 1, 1], [1, -1, 1],
            [-1, 1, 1], [-1, -1, 1],
            [1, 1, -1], [1, -1, -1],
            [-1, 1, -1], [-1, -1, -1], [0, 1, 1], [1, 0, 1], [-1, 0, 1], [0, -1, 1], [0, 0, 1], [0, 0, -1]]
    best_avoidances = []
    for path in best_path:
        best_avoidance = []
        for direc in dirs:
            counter = 0
            z = round(path[2])
            y = round(path[1])
            x = round(path[0])
            t = 4
            k = copy.deepcopy(t)
            while 0 <= z < map_size[2]-1 and 0 <= y < map_size[1]-1 and \
                    0 <= x < map_size[0]-1 and matrix[z][y][x] == 0 and t > 0:
                z += direc[2]
                y += direc[1]
                x += direc[0]
                counter = counter + 1
                t = t - 1
            if counter == k:
                best_avoidance.append((euclidean([x, y, z], best_path[-1]), [x, y, z]))
        best_avoidance.sort()
        if len(best_avoidance) > 0:
            best_avoidances.append(best_avoidance[0][1])
        else:
            best_avoidances.append([round(path[0]), round(path[1]), round(path[2])])
    return best_avoidances
