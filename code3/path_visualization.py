"""
This code visualizes the shortest path in 3D
with the point cloud, and contains collision detection loop.
"""

import open3d as o3d
import numpy as np
import bfsonframes
import lidar_to_grid
import new_a_star
import bezier_curving
import las_to_lat_long
from laspy.file import File
import copy
import avoidance_detection
import random

name = "auburn2"
lasfile = name + ".las"
lidar_arr = lidar_to_grid.createDEM(name)
ascfile = name + ".asc"
las = File(lasfile, mode="r")
min = las.header.min
max = las.header.max

x = lidar_arr[0]
y = lidar_arr[1]



num_of_samples = 5000


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


def avoid_obstacle(cur, avoidances, matrix, obj, old_path, old_avoidances, i, vis, new_end_node):
    x = len(matrix[0][0])
    y = len(matrix[0])
    z = len(matrix)
    sphere2 = obj[1]
    new_x = int(avoidances[i][0])
    new_y = int(avoidances[i][1])
    new_z = int(avoidances[i][2])
    arr = move_drone([cur[0], new_x], [cur[1], new_y], [cur[2], new_z], 4)
    updated_path = new_a_star.generate_path((old_avoidances[i][0], old_avoidances[i][1], old_avoidances[i][2]),
                                            new_end_node, (len(matrix[0][0]), len(matrix[0]), len(matrix)), matrix, 0)
    # add bezier curving
    updated_path = bezier_curving.calculate_bezier(updated_path, num_of_samples)
    old_path = copy.deepcopy(updated_path)
    for pos in arr:
        sphere2.translate((pos[0], pos[1], pos[2]), relative=False)
        vis.update_geometry(sphere2)
        vis.poll_events()
        vis.update_renderer()
    old_avoidances = avoidance_detection.find_avoidances(matrix, updated_path)
    avoidances = []
    for i in range(len(updated_path)):
        updated_path[i][0] = (updated_path[i][0]+min[0]) * 100
        updated_path[i][1] = (max[1]-updated_path[i][1]) * 100
        updated_path[i][2] = (min[2]+updated_path[i][2]) * 100
        avoidances.append([(old_avoidances[i][0]+min[0]) * 100, (max[1]-old_avoidances[i][1]) * 100,(min[2]+old_avoidances[i][2]) * 100])



    updated_path.insert(0, [cur[0], cur[1], cur[2]])
    # render new path in visualization
    new_path_line = o3d.geometry.LineSet()
    lines = []
    for i in range(len(updated_path)-1):
        lines.append([i, i+1])
    rand_colors = [random.random(), random.random(), random.random()]
    colors = [rand_colors for i in range(len(lines))]

    new_path_line.points = o3d.utility.Vector3dVector(updated_path)
    new_path_line.lines = o3d.utility.Vector2iVector(lines)
    new_path_line.colors = o3d.utility.Vector3dVector(colors)
    vis.add_geometry(new_path_line)

    # print('path len: ', len(updated_path))
    # print('avoidance len: ', len(avoidances))

    main_loop(updated_path[1:], avoidances[1:], matrix, obj, old_path, old_avoidances, new_end_node)


def rotate_view(vis):
    ctr = vis.get_view_control()
    ctr.rotate(10.0, 0.0)
    return False

def find_path():
    start_node = (x//2, 5, 0)
    end_node = (x//2, y - (y//2), 0)

    buffer = 1

    matrix = bfsonframes.create3DMatrix(ascfile, int(min[2]), int(max[2]), x, y, buffer, 1.0)

    matrix = np.array(matrix)

    tmp = start_node[2]

    if matrix[start_node[2]][start_node[1]][start_node[0]] > 0:
        while matrix[tmp][start_node[1]][start_node[0]] > 0:
            tmp += 1
    # tmp += 10
    new_start_node = start_node[:2] + (tmp,)

    tmp = end_node[2]

    if matrix[end_node[2]][end_node[1]][end_node[0]] > 0:
        while matrix[tmp][end_node[1]][end_node[0]] > 0:
            tmp += 1
    # tmp += 15
    new_end_node = end_node[:2] + (tmp,)

    print(len(matrix))

    # print(matrix[new_start_node[2]][new_start_node[1]][new_start_node[0]])
    # print(matrix[end_node[2]][end_node[1]][end_node[0]])

    best_path = new_a_star.generate_path(new_start_node, new_end_node,
                                    (len(matrix[0][0]), len(matrix[0]), len(matrix)), matrix, 1)
    best_path = bezier_curving.calculate_bezier(best_path, num_of_samples)
    avoidances = avoidance_detection.find_avoidances(matrix, best_path)
    lines = []
    old_path = copy.deepcopy(best_path)
    old_avoidances = copy.deepcopy(avoidances)

    point_data = np.array([las.X, las.Y, las.Z]).transpose()
    geom = o3d.geometry.PointCloud()
    geom.points = o3d.utility.Vector3dVector(point_data)

    for i in range(0, len(best_path)):
        best_path[i][0] = (best_path[i][0]+min[0]) * 100
        best_path[i][1] = (max[1]-best_path[i][1]) * 100
        best_path[i][2] = (min[2]+best_path[i][2]) * 100
        avoidances[i][0] = (avoidances[i][0]+min[0]) * 100
        avoidances[i][1] = (max[1]-avoidances[i][1]) * 100
        avoidances[i][2] = (min[2]+avoidances[i][2]) * 100
        # print(avoidances[i], old_avoidances[i])

    # print(avoidances)

    line_distance = 0.0
    for i in range(0, len(best_path)-1):
        line_distance += np.sqrt((best_path[i+1][0]-best_path[i][0])*(best_path[i+1][0]-best_path[i][0])
                                 +(best_path[i+1][1]-best_path[i][1])*(best_path[i+1][1]-best_path[i][1])
                                 +(best_path[i+1][2]-best_path[i][2])*(best_path[i+1][2]-best_path[i][2]))

    print('distane: ', line_distance)

    for i in range(len(best_path)-1):
        lines.append([i, i + 1])

    test = copy.deepcopy(best_path)
    test2 = copy.deepcopy(old_path)

    # print(las_to_lat_long.convert_to_latlon(test, test2))

    return [lines, best_path, geom, matrix, avoidances, old_path, old_avoidances, new_end_node]


def init_vis(lines, best_path, geom):
    colors = [[1, 0, 0] for i in range(len(lines))]

    line_set = o3d.geometry.LineSet()
    line_set2 = o3d.geometry.LineSet()
    avoid_path = o3d.geometry.LineSet()

    vis = o3d.visualization.Visualizer()
    vis.create_window()

    sphere1 = o3d.geometry.TriangleMesh.create_sphere(radius=50.0, resolution=200)
    sphere2 = copy.deepcopy(sphere1).translate((best_path[0][0], best_path[0][1], best_path[0][2]), relative=False)

    obstacle_positions = [best_path[len(best_path) - (len(best_path)//2)], best_path[len(best_path) - (len(best_path)//3) - 850]]
    obstacle1 = o3d.geometry.TriangleMesh.create_sphere(radius=100.0, resolution=200)
    obstacle2 = copy.deepcopy(obstacle1).translate((obstacle_positions[0][0], obstacle_positions[0][1], obstacle_positions[0][2]),
                                                   relative=False)
    obstacle3 = copy.deepcopy(obstacle1).translate((obstacle_positions[1][0], obstacle_positions[1][1], obstacle_positions[1][2]),
                                                   relative=False)

    line_set.points = o3d.utility.Vector3dVector(best_path)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)

    obstacle2.paint_uniform_color([1, 0.706, 0])

    vis.add_geometry(sphere2)
    vis.add_geometry(geom)
    vis.add_geometry(line_set)
    # vis.add_geometry(obstacle2)
    vis.add_geometry(obstacle3)
    vis.add_geometry(line_set2)
    vis.add_geometry(avoid_path)
    return [obstacle_positions, sphere2, line_set2, avoid_path, vis, obstacle2]


ok = True
ok2 = True


def main_loop(best_path, avoidances, matrix, obj, old_path, old_avoidances, end_node):
    global ok
    global ok2
    obstacle_positions = obj[0]
    sphere2 = obj[1]
    line_set2 = obj[2]
    avoid_path = obj[3]
    vis = obj[4]
    obstacle2 = obj[5]
    vel = 50
    baseline = obstacle_positions[0][0]

    while True:
        for i in range(0, len(best_path)-3, 25):
            obstacle_positions[0][0] += vel
            if obstacle_positions[0][0] <= baseline-1500 or obstacle_positions[0][0] > baseline+1500:
                vel *= -1

            obstacle2.translate((obstacle_positions[0][0], obstacle_positions[0][1], obstacle_positions[0][2]), relative=False)

            sphere2.translate((best_path[i][0], best_path[i][1], best_path[i][2]), relative=False)
            line_set2.points = o3d.utility.Vector3dVector(
                [best_path[i], best_path[i + 1], best_path[i + 2], best_path[i + 3]])
            line_set2.lines = o3d.utility.Vector2iVector([[0, 1], [1, 2], [2, 3]])
            line_set2.colors = o3d.utility.Vector3dVector([[0, 0, 1] for i in range(3)])
            avoid_path.points = o3d.utility.Vector3dVector([best_path[i], [(best_path[i][0] + avoidances[i][0]) / 2,
                                                                           (best_path[i][1] + avoidances[i][1]) / 2,
                                                                           (best_path[i][2] + avoidances[i][2]) / 2]])
            avoid_path.lines = o3d.utility.Vector2iVector([[0, 1]])
            avoid_path.colors = o3d.utility.Vector3dVector([1, 0.3, 0.5] for i in range(0, 1))
            val = np.sqrt(
                (best_path[i][0] - obstacle_positions[0][0]) * (best_path[i][0] - obstacle_positions[0][0]) + (
                        best_path[i][1] - obstacle_positions[0][1]) * (
                        best_path[i][1] - obstacle_positions[0][1]) + (
                        best_path[i][2] - obstacle_positions[0][2]) * (best_path[i][2] - obstacle_positions[0][2]))
            val1 = np.sqrt(
                (best_path[i][0] - obstacle_positions[1][0]) * (best_path[i][0] - obstacle_positions[1][0]) + (
                        best_path[i][1] - obstacle_positions[1][1]) * (
                        best_path[i][1] - obstacle_positions[1][1]) + (
                        best_path[i][2] - obstacle_positions[1][2]) * (best_path[i][2] - obstacle_positions[1][2]))
            vis.update_geometry(sphere2)
            vis.update_geometry(line_set2)
            vis.update_geometry(avoid_path)
            vis.update_geometry(obstacle2)
            vis.poll_events()
            vis.update_renderer()
            # view.camera_local_rotate(50, 50)
            if 0.1 <= val1 < 500.0 and ok:
                sphere2.paint_uniform_color([1, 0.706, 0])
                # need a function to do back conversions to approximate location of obj
                # matrix[round(old_path[i][2])][round(old_path[i][1])][round(old_path[i][0])] = 255
                print('old avoidances: ', old_avoidances)
                ok = False
                avoid_obstacle(best_path[i], avoidances, matrix, obj, old_path, old_avoidances, i, vis, end_node)
            elif 0.1 <= val < 500.0 and ok2:
                sphere2.paint_uniform_color([1, 0.706, 0])
                # need a function to do back conversions to approximate location of obj
                # matrix[round(old_path[i][2])][round(old_path[i][1])][round(old_path[i][0])] = 255
                print('old avoidances: ', old_avoidances)
                ok2 = False
                avoid_obstacle(best_path[i], avoidances, matrix, obj, old_path, old_avoidances, i, vis, end_node)
            else:
                sphere2.paint_uniform_color([0.255, 0.255, 0.255])


components = find_path()
objects = init_vis(components[0], components[1], components[2])
main_loop(components[1], components[4], components[3], objects, components[5], components[6], components[7])
