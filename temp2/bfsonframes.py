"""
This code will add a buffer (using BFS) to account for drone size.
"""
from PIL import Image, ImageOps, ImageDraw
from laspy.file import File
import queue
import numpy as np


def bfs(matrix, matrix2, buffer):
    visited = matrix2
    # buffer
    t = buffer
    dirs = [[0, 1], [0, -1], [-1, 0], [1, 0]]
    q = queue.Queue()
    for i in range(len(matrix2)):
        for j in range(len(matrix2[0])):
            if matrix2[i][j] == 0:
                q.put([i, j])
    while t > 0:
        n = q.qsize()
        while n > 0:
            top = q.get()
            matrix[top[0]][top[1]] = 0
            for ddir in dirs:
                x1 = top[0]+ddir[0]
                y1 = top[1]+ddir[1]
                if 0 <= x1 < len(matrix) and 0 <= y1 < len(matrix2[0]) and visited[x1][y1] == 1:
                    q.put([x1, y1])
                    visited[x1][y1] = 0
            n = n-1
        t = t-1
    return matrix2

'''
def createGif(start, end, x, y):
    channel_values = open("auburn.asc").read().split()

    frames = []
    img = Image.new('RGB', (x, y), "white")
    pixels = img.load()
    draw = ImageDraw.Draw(img)
    for height in range(start, end):
        matrix = []
        counter = 0
        img1 = Image.new('RGB', (x, y), "white")
        pixels2 = img1.load()
        for y in range(img.height):
            for x in range(img.width):
                if float(channel_values[counter]) >= float(height):
                    pixels2[x, y] = (0, 0, 0)
                    matrix.append(0)
                else:
                    matrix.append(1)
                counter += 1
        matrix = [matrix[i:i + x] for i in range(0, len(matrix), x)]
        # Run BFS
        matrix = bfs(matrix, matrix)
        grid = grid.Grid(matrix=matrix)
        start = grid.node(5, 5)
        end = grid.node(x-5, y-5)
        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        path, runs = finder.find_path(start, end, grid)
        print(height)
        print(path)
        pixels2 = img1.load()
        for i, j in path:
            pixels2[i, j] = (255, 0, 0)
        # img1.show()
        frames.append(img1)

    frames[0].save('cross_auburn_buffer3.gif', save_all=True, append_images=frames[1:], optimize=False, duration=230, loop=0)
'''

'''
def createPath(file, start, end, height, x, y):
    channel_values = open(file).read().split()
    matrix = []
    counter = 0
    for j in range(y):
        for i in range(x):
            if float(channel_values[counter]) >= float(height):
                matrix.append(0)
            else:
                matrix.append(1)
            counter += 1
    matrix = [matrix[i:i + x] for i in range(0, len(matrix), x)]
    # Run BFS
    matrix = bfs(matrix, matrix)
    grid = Grid(matrix=matrix)
    print(matrix)
    start = grid.node(start[0], start[1])
    end = grid.node(end[0], end[1])
    finder = AStarFinder(heuristic=euclidean, diagonal_movement=DiagonalMovement.always)
    path, runs = finder.find_path(start, end, grid)
    return path
'''
'''
def create3DPath(grids, startNode, endNode, x, y, zlen):
    finder = AStarFinder(heuristic=euclidean, diagonal_movement=DiagonalMovement.always)
    grid3D = Grid3D(matrix=grids)
    start = grid3D.node(startNode[0], startNode[1], startNode[2])
    end = grid3D.node(endNode[0], endNode[1], endNode[2])
    path = finder.find_path(start, end, grid3D)
    return path
'''
def create3DMatrix(file, start, end, x, y, buffer, step):
    grids = []
    channel_values = open(file).read().split()
    for height in np.arange(start, end+step, step):
        matrix = []
        counter = 0
        for j in range(y):
            for i in range(x):
                if height - float(channel_values[counter]) <= 25.0:
                    # print(abs(float(channel_values[counter]) - height))
                    matrix.append(255)
                else:
                    matrix.append(0)
                counter += 1
        matrix = [matrix[i:i + x] for i in range(0, len(matrix), x)]
        # Run BFS
        matrix = bfs(matrix, matrix, buffer)
        grids.append(matrix)
    return grids

# from pathfinding.core.diagonal_movement import DiagonalMovement
# from pathfinding.core.grid import Grid
# from pathfinding.finder.a_star import AStarFinder
