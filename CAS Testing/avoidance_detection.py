"""
Contains functions for avoidance path calculation.
"""
import copy

import numpy as np
import math


def euclidean(best_avoidance, goal):
    val = np.sqrt((goal[0]-best_avoidance[0])*(goal[0]-best_avoidance[0]) +
                  (goal[1]-best_avoidance[1])*(goal[1]-best_avoidance[1]) +
                  (goal[2]-best_avoidance[2])*(goal[2]-best_avoidance[2]))
    return val


def find_avoidances(matrix, best_path, big, thres):
    # x y z
    map_size = [len(matrix[0][0]), len(matrix[0]), len(matrix)]
    print('BIG: ', big)
    if big is False:
        dirs = [[-1.5708], [1.5708]]
    else:
        dirs = [[-1.5708], [1.5708]]



    best_avoidances = []
    for i in range(0, len(best_path)-3):
        best_avoidance = []
        angle = math.atan2(best_path[i + 3][1] - best_path[i][1], best_path[i + 3][0] - best_path[i][0])
        for direc in dirs:
            counter = 0
            z = round(best_path[i][2])
            y = round(best_path[i][1])
            x = round(best_path[i][0])
            t = thres
            k = copy.deepcopy(t)
            while 0 <= z < map_size[2]-1 and 0 <= y < map_size[1]-1 and \
                    0 <= x < map_size[0]-1 and matrix[round(z)][round(y)][round(x)] == 0 and t > 0:
                if len(direc) == 1:
                    z += 0
                    y += 1 * math.sin(angle + direc[0])
                    x += 1 * math.cos(angle + direc[0])
                else:
                    z += direc[2]
                    y += direc[1]
                    x += direc[0]
                counter = counter + 1
                t = t - 1
            if counter == k:
                best_avoidance.append((euclidean([round(x), round(y), round(z)], best_path[-1]), [round(x), round(y), round(z)]))
        best_avoidance.sort()
        if len(best_avoidance) > 0:
            best_avoidances.append(best_avoidance[0][1])
        else:
            if len(best_avoidances) > 0:
                best_avoidances.append(best_avoidances[-1])
            else:
                best_avoidances.append([0, 0, 0])
    best_avoidances.append([0, 0, 0])
    best_avoidances.append([0, 0, 0])
    best_avoidances.append([0, 0, 0])
    return best_avoidances
