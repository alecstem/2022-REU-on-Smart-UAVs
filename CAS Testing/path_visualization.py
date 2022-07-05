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

global overall_path

name = "newyork2"
lasfile = name + ".las"
lidar_arr = lidar_to_grid.createDEM(name)
ascfile = name + ".asc"
las = File(lasfile, mode="r")
min = las.header.min
max = las.header.max

x = lidar_arr[0]
y = lidar_arr[1]

num_of_samples = 5000

def euclidean(p1, p2):
    val = np.sqrt((p2[0]-p1[0])*(p2[0]-p1[0]) +
                  (p2[1]-p1[1])*(p2[1]-p1[1]) +
                  (p2[2]-p1[2])*(p2[2]-p1[2]))
    return val


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

obs_dict = {}
def avoid_obstacle(cur, avoidances, matrix, obj, old_path, old_avoidances, i, vis, new_end_node, big, thres, obs_name):
    global obs_dict
    sphere2 = obj[1]
    new_x = int(avoidances[i][0])
    new_y = int(avoidances[i][1])
    new_z = int(avoidances[i][2])
    obs_dict[obs_name] = [old_avoidances[i][0] - old_path[i][0], old_avoidances[i][1] - old_path[i][1], old_avoidances[i][2] - old_path[i][2]]
    print(obs_dict[obs_name])
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
    old_avoidances = avoidance_detection.find_avoidances(matrix, updated_path, big, thres)
    # print(old_avoidances)
    avoidances = []
    for i in range(len(updated_path)):
        updated_path[i][0] = (updated_path[i][0]+min[0]) * 100
        updated_path[i][1] = (max[1]-updated_path[i][1]) * 100
        updated_path[i][2] = (min[2]+updated_path[i][2]) * 100
        avoidances.append([(old_avoidances[i][0]+min[0]) * 100,
                           (max[1]-old_avoidances[i][1]) * 100, (min[2]+old_avoidances[i][2]) * 100])
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



def avoid_obstacle2(cur,matrix, obj, old_avoidances, old_path, i, vis, new_end_node, big, thres, obs_name):
    global obs_dict
    sphere2 = obj[1]

    new_x = round(old_path[i][0] + obs_dict[obs_name][0])
    new_y = round(old_path[i][1] + obs_dict[obs_name][1])
    new_z = round(old_path[i][2] + obs_dict[obs_name][2])
    arr = move_drone([cur[0], new_x], [cur[1], new_y], [cur[2], new_z], 4)
    print('yo: ', (round(old_path[i][0] + obs_dict[obs_name][0]), round(old_path[i][1] + old_avoidances[i][1]),
                                             round(old_path[i][2] + old_avoidances[i][2])))
    updated_path = new_a_star.generate_path((round(old_path[i][0] + obs_dict[obs_name][0]), round(old_path[i][1] + obs_dict[obs_name][1]),
                                             round(old_path[i][2] + obs_dict[obs_name][2])),
                                            new_end_node, (len(matrix[0][0]), len(matrix[0]), len(matrix)), matrix, 0)
    # add bezier curving
    updated_path = bezier_curving.calculate_bezier(updated_path, num_of_samples)
    old_path = copy.deepcopy(updated_path)
    for pos in arr:
        sphere2.translate((pos[0], pos[1], pos[2]), relative=False)
        vis.update_geometry(sphere2)
        vis.poll_events()
        vis.update_renderer()
    old_avoidances = avoidance_detection.find_avoidances(matrix, updated_path, big, thres)
    # print(old_avoidances)
    avoidances = []
    for i in range(len(updated_path)):
        updated_path[i][0] = (updated_path[i][0]+min[0]) * 100
        updated_path[i][1] = (max[1]-updated_path[i][1]) * 100
        updated_path[i][2] = (min[2]+updated_path[i][2]) * 100
        avoidances.append([(old_avoidances[i][0]+min[0]) * 100,
                           (max[1]-old_avoidances[i][1]) * 100, (min[2]+old_avoidances[i][2]) * 100])
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


def find_path():
    #     start_node = (5, 5, 5)
    #     end_node = (x - 80, y - 25, 100)
    start_node = (x - 5, y - 5, 10)
    end_node = (10, 5, 10)

    buffer = 2

    matrix = bfsonframes.create3DMatrix(ascfile, int(min[2]), int(max[2]), x, y, buffer, 1.0)

    matrix = np.array(matrix)

    tmp = start_node[2]

    if matrix[start_node[2]][start_node[1]][start_node[0]] > 0:
        while matrix[tmp][start_node[1]][start_node[0]] > 0:
            tmp += 1
    # tmp += 7
    new_start_node = start_node[:2] + (tmp,)

    tmp = end_node[2]

    if matrix[end_node[2]][end_node[1]][end_node[0]] > 0:
        while matrix[tmp][end_node[1]][end_node[0]] > 0:
            tmp += 1
    # tmp += 7
    new_end_node = end_node[:2] + (tmp,)

    print(len(matrix))

    # print(matrix[new_start_node[2]][new_start_node[1]][new_start_node[0]])
    # print(matrix[end_node[2]][end_node[1]][end_node[0]])

    best_path = new_a_star.generate_path(new_start_node, new_end_node,
                                    (len(matrix[0][0]), len(matrix[0]), len(matrix)), matrix, 0)
    best_path = bezier_curving.calculate_bezier(best_path, num_of_samples)
    avoidances = avoidance_detection.find_avoidances(matrix, best_path, True, 3)
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

    for k in range(0, len(avoidances)):
        # print('old: ', avoidances[k])
        avoidances[k][0] = (avoidances[k][0]+min[0]) * 100
        avoidances[k][1] = (max[1]-avoidances[k][1]) * 100
        avoidances[k][2] = (min[2]+avoidances[k][2]) * 100
        # print('new: ', avoidances[k])



    overall_path = best_path

    # print(avoidances)

    line_distance = 0.0
    for i in range(0, len(best_path)-1):
        line_distance += np.sqrt((best_path[i+1][0]-best_path[i][0])*(best_path[i+1][0]-best_path[i][0])
                                 + (best_path[i+1][1]-best_path[i][1])*(best_path[i+1][1]-best_path[i][1])
                                 + (best_path[i+1][2]-best_path[i][2])*(best_path[i+1][2]-best_path[i][2]))

    print('distance: ', line_distance)

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
    cylinder1 = o3d.geometry.TriangleMesh.create_box(width=1300.0, height=300.0, depth=10000.0)

    mid = random.randrange(100, len(best_path), 200)

    obstacle_positions = [best_path[random.randrange(100, len(best_path), 200)], best_path[random.randrange(100, len(best_path), 200)], best_path[mid]]
    obstacle1 = o3d.geometry.TriangleMesh.create_sphere(radius=100.0, resolution=200)
    obstacle2 = copy.deepcopy(obstacle1).translate((obstacle_positions[0][0], obstacle_positions[0][1], obstacle_positions[0][2]),
                                                   relative=False)
    obstacle3 = copy.deepcopy(obstacle1).translate((obstacle_positions[1][0], obstacle_positions[1][1], obstacle_positions[1][2]),
                                                   relative=False)
    cylinder2 = copy.deepcopy(cylinder1).translate((best_path[mid][0], best_path[mid][1], best_path[mid][2]),
                                                   relative=False)


    line_set.points = o3d.utility.Vector3dVector(best_path)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)

    obstacle2.paint_uniform_color([1, 0.706, 0])

    # vis.add_geometry(sphere2)
    vis.add_geometry(geom)
    # vis.add_geometry(line_set)
    # vis.add_geometry(obstacle2)
    # vis.add_geometry(obstacle3)
    # vis.add_geometry(line_set2)
    # vis.add_geometry(avoid_path)
    # vis.add_geometry(cylinder2)
    return [obstacle_positions, sphere2, line_set2, avoid_path, vis, obstacle2]


ok = True
ok2 = True
ok3 = True

counter = 15

def main_loop(best_path, avoidances, matrix, obj, old_path, old_avoidances, end_node):
    global ok
    global ok2
    global ok3
    global obs_dict
    global counter

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
            '''
            angle = math.atan2(best_path[i + 3][1] - best_path[i][1], best_path[i + 3][0] - best_path[i][0])
            x1 = best_path[i+3][0]
            y1 = best_path[i+3][1]
            z1 = best_path[i+3][2]
            x0 = best_path[i][0]
            y0 = best_path[i][1]
            z0 = best_path[i][2]
            uvec = (y1-y0) / np.sqrt((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0)+(z1-z0)*(z1-z0))
            pitch = math.degrees(math.asin(-uvec))
            print(angle, pitch)
            '''

            obstacle_positions[0][0] += vel
            if obstacle_positions[0][0] <= baseline-900 or obstacle_positions[0][0] > baseline+900:
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
            val = euclidean(obstacle_positions[0], best_path[i])
            val1 = euclidean(obstacle_positions[1], best_path[i])
            val2 = euclidean(obstacle_positions[2], best_path[i])
            vis.update_geometry(sphere2)
            vis.update_geometry(line_set2)
            vis.update_geometry(avoid_path)
            vis.update_geometry(obstacle2)
            vis.poll_events()
            vis.update_renderer()
            # view.camera_local_rotate(50, 50)
            if 0.1 <= val1 < 350.0 and ok:
                sphere2.paint_uniform_color([1, 0.706, 0])
                # ok = False
                if 'obs1' in obs_dict:
                    avoid_obstacle2(best_path[i], matrix, obj, old_avoidances, old_path, i, vis, end_node,
                                   False, 3, 'obs1')
                else:
                    avoid_obstacle(best_path[i], avoidances, matrix, obj, old_path, old_avoidances, i, vis, end_node,
                                   False, 3, 'obs1')
            elif 0.1 <= val < 350.0 and ok2:
                sphere2.paint_uniform_color([1, 0.706, 0])
                # ok2 = False
                if 'obs2' in obs_dict:
                    avoid_obstacle2(best_path[i], matrix, obj, old_avoidances, old_path, i, vis, end_node,
                                   False, 3, 'obs2')
                else:
                    avoid_obstacle(best_path[i], avoidances, matrix, obj, old_path, old_avoidances, i, vis, end_node,
                                   False, 3, 'obs2')
            elif 0.1 <= val2 < 900.0 and ok3:
                sphere2.paint_uniform_color([1, 0.706, 0])
                # ok3 = False
                counter -= 1
                if 'obs3' in obs_dict and counter > 0:
                    avoid_obstacle2(best_path[i], matrix, obj, old_avoidances, old_path, i, vis, end_node,
                                   False, 3, 'obs3')
                else:
                    counter = 15
                    avoid_obstacle(best_path[i], avoidances, matrix, obj, old_path, old_avoidances, i, vis, end_node,
                                   False, 3, 'obs3')
            else:
                sphere2.paint_uniform_color([0.255, 0.255, 0.255])


components = find_path()
objects = init_vis(components[0], components[1], components[2])
main_loop(components[1], components[4], components[3], objects, components[5], components[6], components[7])

