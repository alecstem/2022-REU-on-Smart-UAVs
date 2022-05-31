"""
This code visualizes the shortest path in 3D
with the point cloud.
"""

import open3d as o3d
import numpy as np
import bfsonframes
import lidar_to_grid
import new_a_star
from laspy.file import File
import copy
import avoidance_detection
import time

name = "newyork1"
lasfile = name + ".las"
arr = lidar_to_grid.createDEM(name)
ascfile = name + ".asc"
las = File(lasfile, mode="r")
min = las.header.min
max = las.header.max

def move_drone(x_p, y_p, z_p, between):
    d = np.sqrt(
        (x_p[0] - x_p[1]) * (x_p[0] - x_p[1]) + (y_p[0] - y_p[1]) * (y_p[0] - y_p[1]) + (z_p[0] - z_p[1]) * (
                z_p[0] - z_p[1]))
    arr = []
    for n in range(1, between):
        x_new = x_p[0] + ((n / d) * (x_p[1] - x_p[0]))
        y_new = y_p[0] + ((n / d) * (y_p[1] - y_p[0]))
        z_new = z_p[0] + ((n / d) * (z_p[1] - z_p[0]))
        arr.append([x_new, y_new, z_new])
        # sphere2.translate((x_new, y_new, z_new), relative=False)
    return arr


def avoid_obstacle(cur, avoidances, matrix, obj, old_path, old_avoidances, i, vis):
    x = len(matrix[0][0])
    y = len(matrix[0])
    z = len(matrix)
    sphere2 = obj[1]
    new_x = int(avoidances[i][0])
    new_y = int(avoidances[i][1])
    new_z = int(avoidances[i][2])
    old_cur = old_path[i]
    old_avoidance = old_avoidances[i]
    arr = move_drone([cur[0], new_x], [cur[1], new_y], [cur[2], new_z], 3)
    for pos in arr:
        sphere2.translate((pos[0], pos[1], pos[2]), relative=False)
        vis.update_geometry(sphere2)
        # vis.update_geometry(line_set2)
        # vis.update_geometry(avoid_path)
        # vis.update_geometry(obstacle2)
        vis.poll_events()
        vis.update_renderer()
    print('matrix in loop: ', len(matrix[0][0]), len(matrix[0]), len(matrix))
    updated_path = new_a_star.generate_path((old_avoidance[0], old_avoidance[1], old_avoidance[2]), (x - 5, y - 5, 75), (len(matrix[0][0]), len(matrix[0]), len(matrix)), matrix)
    for i in range(len(updated_path)):
        updated_path[i][0] = (updated_path[i][0]+min[0]) * 100
        updated_path[i][1] = (max[1]-updated_path[i][1]) * 100
        updated_path[i][2] = (min[2]+updated_path[i][2]) * 100
    main_loop(updated_path, avoidances, matrix, obj, old_path, old_avoidances)


def find_path():

    x = arr[0]
    y = arr[1]
    buffer = 1
    vbuffer = 0

    matrix = bfsonframes.create3DMatrix(ascfile, int(min[2]), int(max[2]), x, y, buffer, 1)
    matrix = np.array(matrix)

    print(len(matrix))

    # add vertical obstacles based on buffer

    if vbuffer > 0:
        for z1 in range(0, len(matrix)):
            for y1 in range(len(matrix[0])):
                for x1 in range(len(matrix[0][0])):
                    if matrix[z1][y1][x1] == 0:
                        for k in range(1, vbuffer):
                            if z1 - k >= 0:
                                matrix[z1-k][y1][x1] = 0

    # print([x, y, len(matrix)])

    start_node = (x-3, 3, 80)
    end_node = (15, y-15, 40)

    print(matrix[start_node[2]][start_node[1]][start_node[0]])
    print(matrix[end_node[2]][end_node[1]][end_node[0]])

    best_path = new_a_star.generate_path(start_node, end_node,
                                         (len(matrix[0][0]), len(matrix[0]), len(matrix)), matrix)
    # best_path = new_a_star.generate_path((3, 3, 80), (x - 5, y - 5, 40), (len(matrix[0][0]), len(matrix[0]), len(matrix)), matrix)
    # best_path = [[5, 5, 60], [5, 6, 60], [5, 7, 60], [5, 8, 60], [5, 9, 60], [5, 10, 60], [5, 11, 60], [5, 12, 60], [5, 13, 60], [4, 14, 61], [4, 15, 61], [4, 16, 61], [4, 17, 61], [4, 18, 61], [4, 19, 61], [4, 20, 61], [5, 21, 62], [5, 22, 62], [5, 23, 62], [5, 24, 62], [5, 25, 62], [5, 26, 62], [5, 27, 62], [5, 28, 62], [5, 29, 62], [5, 30, 62], [5, 31, 62], [5, 32, 62], [5, 33, 62], [5, 34, 62], [5, 35, 62], [5, 36, 62], [6, 37, 63], [7, 38, 64], [7, 39, 64], [8, 40, 65], [8, 41, 65], [8, 42, 65], [8, 43, 65], [8, 44, 65], [8, 45, 65], [8, 46, 65], [9, 47, 66], [9, 48, 66], [9, 49, 66], [9, 50, 66], [9, 51, 66], [9, 52, 66], [9, 53, 66], [9, 54, 66], [9, 55, 66], [10, 56, 67], [10, 57, 67], [10, 58, 67], [10, 59, 67], [10, 60, 67], [10, 61, 67], [10, 62, 67], [10, 63, 67], [11, 64, 68], [11, 65, 68], [11, 66, 68], [11, 67, 68], [11, 68, 68], [11, 69, 68], [12, 70, 69], [12, 71, 69], [12, 72, 69], [12, 73, 69], [12, 74, 69], [12, 75, 69], [12, 76, 69], [13, 77, 70], [13, 78, 70], [13, 79, 70], [13, 80, 70], [13, 81, 70], [13, 82, 70], [13, 83, 70], [13, 84, 70], [13, 85, 70], [14, 86, 71], [14, 87, 71], [14, 88, 71], [14, 89, 71], [14, 90, 71], [14, 91, 71], [14, 92, 71], [14, 93, 71], [14, 94, 71], [15, 95, 72], [15, 96, 72], [15, 97, 72], [15, 98, 72], [16, 99, 73], [16, 100, 73], [17, 101, 74], [18, 102, 75], [19, 103, 76], [20, 103, 76], [21, 103, 76], [22, 103, 76], [23, 103, 76], [24, 103, 76], [25, 103, 76], [26, 103, 76], [27, 103, 76], [28, 103, 76], [29, 103, 76], [30, 103, 76], [31, 103, 76], [32, 103, 76], [33, 103, 76], [34, 103, 76], [35, 103, 76], [36, 103, 76], [37, 103, 76], [38, 103, 76], [39, 103, 76], [40, 103, 76], [41, 103, 76], [42, 103, 76], [43, 103, 76], [44, 103, 76], [45, 104, 77], [46, 105, 78], [47, 106, 79], [48, 107, 80], [49, 108, 81], [50, 109, 82], [51, 110, 83], [52, 111, 84], [53, 112, 85], [54, 113, 86], [55, 114, 87], [56, 115, 88], [57, 116, 89], [58, 117, 90], [59, 118, 91], [60, 119, 92], [61, 120, 93], [62, 121, 94], [63, 122, 95], [64, 123, 96], [65, 124, 96], [66, 125, 96], [67, 126, 96], [68, 127, 96], [69, 128, 96], [70, 129, 96], [71, 130, 96], [72, 131, 96], [73, 132, 96], [74, 133, 96], [75, 134, 96], [76, 135, 96], [77, 136, 96], [77, 137, 96], [77, 138, 96], [78, 139, 96], [78, 140, 96], [79, 141, 96], [80, 142, 96], [81, 143, 96], [82, 143, 96], [83, 144, 96], [84, 145, 96], [85, 146, 96], [86, 147, 96], [87, 148, 96], [88, 149, 96], [89, 150, 96], [90, 151, 96], [91, 152, 96], [92, 153, 96], [93, 154, 97], [94, 155, 98], [95, 156, 99], [96, 156, 99], [97, 156, 99], [98, 156, 99], [99, 156, 99], [100, 156, 99], [101, 156, 99], [102, 156, 99], [103, 156, 99], [104, 156, 99], [105, 156, 99], [106, 156, 99], [107, 156, 99], [108, 156, 99], [109, 156, 99], [110, 156, 99], [111, 156, 99], [112, 156, 99], [113, 156, 99], [114, 156, 99], [115, 156, 99], [116, 156, 99], [117, 156, 99], [118, 156, 99], [119, 156, 99], [120, 156, 99], [121, 156, 99], [122, 156, 99], [123, 156, 99], [124, 156, 99], [125, 156, 99], [126, 156, 99], [127, 156, 99], [128, 156, 99], [129, 156, 99], [130, 156, 99], [131, 156, 99], [132, 156, 99], [133, 156, 99], [134, 155, 100], [135, 155, 100]]
    # best_path = [[5, 5, 40], [6, 6, 40], [7, 6, 40], [8, 6, 40], [9, 6, 40], [10, 6, 40], [11, 6, 40], [12, 6, 40], [13, 6, 40], [14, 6, 40], [15, 6, 40], [16, 6, 40], [17, 6, 40], [18, 6, 40], [19, 6, 40], [20, 6, 40], [21, 6, 40], [22, 6, 40], [23, 6, 40], [24, 6, 40], [25, 6, 40], [26, 6, 40], [27, 6, 40], [28, 6, 40], [29, 6, 40], [30, 6, 40], [31, 6, 40], [32, 6, 40], [33, 6, 40], [34, 6, 40], [35, 6, 40], [36, 6, 40], [37, 6, 40], [38, 7, 40], [39, 8, 40], [40, 9, 40], [41, 9, 40], [42, 9, 40], [43, 9, 40], [44, 9, 40], [45, 9, 40], [46, 9, 40], [47, 9, 40], [48, 9, 40], [49, 9, 40], [50, 9, 40], [51, 9, 40], [52, 9, 40], [53, 9, 40], [54, 9, 40], [55, 9, 40], [56, 9, 40], [57, 9, 40], [58, 9, 40], [59, 9, 40], [60, 9, 40], [61, 9, 40], [62, 9, 40], [63, 9, 40], [64, 9, 40], [65, 10, 41], [66, 10, 41], [67, 10, 41], [68, 10, 41], [69, 10, 41], [70, 10, 41], [71, 10, 41], [72, 10, 41], [73, 11, 41], [74, 12, 41], [75, 13, 41], [76, 14, 41], [76, 15, 41], [76, 16, 41], [77, 17, 41], [78, 18, 41], [79, 19, 41], [80, 20, 41], [81, 21, 41], [82, 22, 41], [83, 23, 41], [84, 24, 41], [85, 25, 41], [86, 26, 41], [87, 27, 41], [88, 28, 41], [89, 29, 41], [90, 30, 41], [91, 31, 41], [92, 32, 41], [93, 33, 41], [94, 34, 41], [95, 35, 41], [96, 36, 41], [97, 37, 41], [98, 38, 41], [99, 39, 41], [100, 40, 41], [101, 41, 41], [102, 42, 41], [103, 43, 41], [104, 44, 41], [105, 45, 41], [106, 46, 41], [107, 47, 41], [108, 48, 41], [109, 49, 41], [110, 50, 41], [111, 51, 41], [112, 52, 41], [113, 53, 41], [114, 54, 41], [115, 55, 41], [116, 55, 41], [117, 55, 41], [118, 55, 41], [119, 55, 41], [120, 55, 41], [121, 55, 41], [122, 55, 41], [123, 55, 41], [124, 55, 41], [125, 55, 41], [126, 55, 41], [127, 55, 41], [128, 55, 41], [129, 55, 41], [130, 55, 41], [131, 55, 41], [132, 55, 41], [133, 55, 41], [134, 55, 41], [135, 55, 41], [136, 56, 41], [137, 57, 41], [138, 58, 41], [139, 59, 41], [139, 60, 41], [139, 61, 41], [139, 62, 41], [139, 63, 41], [139, 64, 41], [139, 65, 41], [139, 66, 41], [139, 67, 41], [139, 68, 41], [139, 69, 41], [139, 70, 41], [139, 71, 41], [139, 72, 41], [139, 73, 41], [139, 74, 41], [139, 75, 41], [139, 76, 41], [139, 77, 41], [139, 78, 41], [139, 79, 41], [139, 80, 41], [139, 81, 41], [139, 82, 41], [139, 83, 41], [139, 84, 41], [139, 85, 41], [139, 86, 41], [139, 87, 41], [139, 88, 41], [139, 89, 41], [139, 90, 41], [139, 91, 41], [139, 92, 41], [139, 93, 41], [139, 94, 41], [139, 95, 41], [139, 96, 41], [139, 97, 41], [138, 98, 41], [137, 99, 40], [136, 100, 39], [136, 101, 39], [136, 102, 39], [136, 103, 39], [136, 104, 39], [136, 105, 39], [136, 106, 39], [136, 107, 39], [136, 108, 39], [136, 109, 39], [136, 110, 39], [136, 111, 39], [136, 112, 39], [136, 113, 39], [136, 114, 39], [136, 115, 38], [136, 116, 38], [136, 117, 37], [136, 118, 37], [136, 119, 36], [136, 120, 36], [136, 121, 35], [136, 122, 35], [136, 123, 35], [136, 124, 34], [136, 125, 34], [136, 126, 33], [136, 127, 33], [136, 128, 32], [136, 129, 32], [136, 130, 31], [136, 131, 31], [136, 132, 30], [136, 133, 30], [136, 134, 30], [136, 135, 29], [136, 136, 29], [136, 137, 28], [136, 138, 28], [136, 139, 27], [136, 140, 27], [136, 141, 26], [136, 142, 26], [136, 143, 25], [136, 144, 25], [136, 145, 25], [136, 146, 24], [136, 147, 24], [136, 148, 23], [136, 149, 23], [136, 150, 22], [136, 151, 22], [136, 152, 21], [136, 153, 21], [135, 154, 20], [135, 155, 20]]
    # precompute avoidance techniques (array of linesets)

    # best_path = [[5, 155, 20], [6, 155, 21], [7, 154, 22], [8, 153, 23], [9, 152, 24], [10, 151, 25], [11, 150, 26], [12, 149, 27], [13, 148, 28], [14, 147, 29], [15, 146, 30], [16, 145, 31], [17, 144, 32], [18, 143, 33], [18, 142, 34], [18, 141, 35], [18, 140, 36], [18, 139, 37], [18, 138, 38], [18, 137, 39], [18, 136, 40], [18, 135, 41], [18, 134, 42], [18, 133, 43], [18, 132, 44], [18, 131, 45], [18, 130, 46], [18, 129, 47], [18, 128, 48], [18, 127, 49], [18, 126, 50], [18, 125, 51], [18, 124, 52], [18, 123, 53], [18, 122, 54], [18, 121, 55], [18, 120, 56], [18, 119, 57], [19, 118, 58], [20, 117, 59], [21, 116, 60], [22, 115, 61], [23, 114, 62], [24, 113, 63], [25, 112, 64], [26, 111, 65], [27, 110, 66], [28, 109, 67], [29, 108, 68], [30, 107, 69], [31, 106, 70], [32, 105, 71], [33, 104, 72], [34, 103, 73], [35, 102, 74], [36, 101, 75], [37, 100, 76], [38, 100, 77], [39, 100, 78], [40, 100, 79], [41, 100, 80], [42, 100, 81], [43, 100, 82], [44, 100, 83], [45, 100, 84], [46, 100, 85], [47, 100, 86], [48, 100, 87], [49, 100, 88], [50, 100, 89], [51, 100, 90], [52, 100, 91], [53, 100, 92], [54, 100, 93], [55, 100, 94], [56, 100, 95], [57, 100, 96], [58, 100, 97], [59, 100, 98], [60, 100, 99], [61, 100, 100], [62, 100, 101], [63, 100, 102], [64, 100, 103], [65, 100, 103], [66, 99, 104], [67, 98, 105], [68, 97, 106], [69, 96, 107], [70, 96, 107], [71, 95, 108], [72, 94, 109], [73, 93, 110], [74, 92, 111], [75, 91, 112], [76, 90, 113], [77, 89, 114], [78, 88, 115], [79, 87, 116], [80, 86, 117], [81, 85, 118], [82, 84, 119], [83, 83, 120], [84, 82, 121], [85, 81, 122], [86, 80, 123], [87, 79, 124], [88, 78, 125], [89, 77, 126], [90, 76, 127], [91, 75, 128], [92, 74, 129], [93, 73, 129], [94, 72, 129], [95, 71, 129], [96, 70, 129], [97, 69, 129], [98, 68, 129], [98, 67, 129], [99, 66, 128], [100, 65, 127], [101, 64, 126], [102, 63, 125], [103, 62, 124], [104, 61, 123], [105, 60, 122], [106, 59, 121], [107, 58, 120], [108, 57, 119], [109, 56, 118], [110, 55, 117], [111, 54, 117], [112, 53, 116], [113, 52, 116], [114, 51, 115], [115, 50, 115], [116, 49, 115], [117, 48, 114], [117, 47, 114], [118, 46, 113], [118, 45, 113], [118, 44, 113], [119, 43, 112], [119, 42, 112], [120, 41, 111], [120, 40, 111], [121, 39, 110], [121, 38, 110], [121, 37, 110], [122, 36, 109], [122, 35, 109], [123, 34, 108], [123, 33, 108], [124, 32, 107], [124, 31, 107], [124, 30, 107], [125, 29, 106], [125, 28, 106], [126, 27, 105], [126, 26, 105], [126, 25, 105], [127, 24, 104], [127, 23, 104], [128, 22, 103], [128, 21, 103], [129, 20, 102], [130, 19, 101], [131, 18, 100], [132, 17, 100], [132, 16, 100], [133, 15, 100], [133, 14, 100], [134, 13, 100], [134, 12, 100], [134, 11, 100], [135, 10, 100]]

    avoidances = avoidance_detection.find_avoidances(matrix, best_path)

    print(avoidances)

    lines = []

    old_path = copy.deepcopy(best_path)
    old_avoidances = copy.deepcopy(avoidances)

    print('matrix in main: ', len(matrix[0][0]), len(matrix[0]), len(matrix))

    # print(best_path)

    point_data = np.array([las.X, las.Y, las.Z]).transpose()
    # point_data = np.stack([las.X, las.Y, las.Z]).transpose()
    geom = o3d.geometry.PointCloud()
    geom.points = o3d.utility.Vector3dVector(point_data)

    for i in range(len(best_path)):
        best_path[i][0] = (best_path[i][0]+min[0]) * 100
        best_path[i][1] = (max[1]-best_path[i][1]) * 100
        best_path[i][2] = (min[2]+best_path[i][2]) * 100
        avoidances[i][0] = (avoidances[i][0]+min[0]) * 100
        avoidances[i][1] = (max[1]-avoidances[i][1]) * 100
        avoidances[i][2] = (min[2]+avoidances[i][2]) * 100

    # print(best_path)

    for i in range(len(best_path)-1):
        lines.append([i, i + 1])

    return [lines, best_path, geom, matrix, avoidances, old_path, old_avoidances]


def init_vis(lines, best_path, geom):
    colors = [[1, 0, 0] for i in range(len(lines))]

    line_set = o3d.geometry.LineSet()
    line_set2 = o3d.geometry.LineSet()
    avoid_path = o3d.geometry.LineSet()

    vis = o3d.visualization.Visualizer()
    vis.create_window()

    sphere1 = o3d.geometry.TriangleMesh.create_sphere(radius=50.0, resolution=200)
    sphere2 = copy.deepcopy(sphere1).translate((best_path[0][0], best_path[0][1], best_path[0][2]), relative=False)

    obstacle_position = best_path[len(best_path) - 50]
    obstacle1 = o3d.geometry.TriangleMesh.create_sphere(radius=100.0, resolution=200)
    obstacle2 = copy.deepcopy(obstacle1).translate((obstacle_position[0], obstacle_position[1], obstacle_position[2]),
                                                   relative=False)

    line_set.points = o3d.utility.Vector3dVector(best_path)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)

    obstacle2.paint_uniform_color([1, 0.706, 0])

    vis.add_geometry(sphere2)
    vis.add_geometry(geom)
    vis.add_geometry(line_set)
    vis.add_geometry(obstacle2)
    vis.add_geometry(line_set2)
    vis.add_geometry(avoid_path)
    return [obstacle_position, sphere2, line_set2, avoid_path, vis, obstacle2]


def main_loop(best_path, avoidances, matrix, obj, old_path, old_avoidances):
    obstacle_position = obj[0]
    sphere2 = obj[1]
    line_set2 = obj[2]
    avoid_path = obj[3]
    vis = obj[4]
    obstacle2 = obj[5]

    while True:
        for i in range(len(best_path)-3):
            x1 = best_path[i][0]
            x2 = best_path[i+1][0]

            y1 = best_path[i][1]
            y2 = best_path[i+1][1]

            z1 = best_path[i][2]
            z2 = best_path[i+1][2]

            val = np.sqrt(
                (best_path[i][0] - obstacle_position[0]) * (best_path[i][0] - obstacle_position[0]) + (
                        best_path[i][1] - obstacle_position[1]) * (
                        best_path[i][1] - obstacle_position[1]) + (
                        best_path[i][2] - obstacle_position[2]) * (best_path[i][2] - obstacle_position[2]))

            arr = move_drone([x1, x2], [y1, y2], [z1, z2], 3)
            for pos in arr:
                sphere2.translate((pos[0], pos[1], pos[2]), relative=False)
                '''
                vis.update_geometry(sphere2)
                vis.poll_events()
                vis.update_renderer()
                '''
            if val < 500.0:
                sphere2.paint_uniform_color([1, 0.706, 0])
                # avoid_obstacle(best_path[i], avoidances, matrix, obj, old_path, old_avoidances, i, vis)
            else:
                sphere2.paint_uniform_color([0.255, 0.255, 0.255])

            line_set2.points = o3d.utility.Vector3dVector(
                [best_path[i], best_path[i + 1], best_path[i + 2], best_path[i + 3]])
            line_set2.lines = o3d.utility.Vector2iVector([[0, 1], [1, 2], [2, 3]])
            line_set2.colors = o3d.utility.Vector3dVector([[0, 0, 1] for i in range(3)])
            avoid_path.points = o3d.utility.Vector3dVector([best_path[i], [(best_path[i][0] + avoidances[i][0]) / 2,
                                                                           (best_path[i][1] + avoidances[i][1]) / 2,
                                                                           (best_path[i][2] + avoidances[i][2]) / 2]])
            avoid_path.lines = o3d.utility.Vector2iVector([[0, 1]])
            avoid_path.colors = o3d.utility.Vector3dVector([1, 0.3, 0.5] for i in range(0, 1))
            vis.update_geometry(sphere2)
            vis.update_geometry(line_set2)
            vis.update_geometry(avoid_path)
            vis.update_geometry(obstacle2)
            vis.poll_events()
            vis.update_renderer()


components = find_path()
objects = init_vis(components[0], components[1], components[2])
main_loop(components[1], components[4], components[3], objects, components[5], components[6])

