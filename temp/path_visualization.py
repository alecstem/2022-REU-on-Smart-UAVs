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

name = "newyork1"
lasfile = name+".las"
arr = lidar_to_grid.createDEM(name)
ascfile = name+".asc"

x = arr[0]
y = arr[1]
buffer = 1

las = File(lasfile, mode="r")
min = las.header.min
max = las.header.max

matrix = bfsonframes.create3DMatrix(ascfile, int(min[2]), int(max[2]), x, y, buffer)
matrix = np.array(matrix)

# add vertical obstacles based on buffer

for z1 in range(buffer, len(matrix)):
    for y1 in range(len(matrix[0])):
        for x1 in range(len(matrix[0][0])):
            if matrix[z1][y1][x1] == 1 and matrix[z1-buffer][y1][x1] == 0:
                matrix[z1-buffer][y1][x1] = 1


best_path = new_a_star.generate_path((5, y - 5, 30), (x - 5, 5, 100), (x, y, len(matrix)), matrix)
lines = []

point_data = np.array([las.X, las.Y, las.Z]).transpose()
# point_data = np.stack([las.X, las.Y, las.Z]).transpose()
geom = o3d.geometry.PointCloud()
geom.points = o3d.utility.Vector3dVector(point_data)

for i in range(len(best_path)):
    lines.append([i, i + 1])
    best_path[i][0] = (best_path[i][0]+min[0]) * 100
    best_path[i][1] = (max[1]-best_path[i][1]) * 100
    best_path[i][2] = (min[2]+best_path[i][2]) * 100

colors = [[1, 0, 0] for i in range(len(lines))]

line_set = o3d.geometry.LineSet()

line_set.points = o3d.utility.Vector3dVector(best_path)
line_set.lines = o3d.utility.Vector2iVector(lines)
line_set.colors = o3d.utility.Vector3dVector(colors)
# o3d.visualization.draw_geometries([line_set, geom])
o3d.visualization.draw_geometries([line_set, geom])
