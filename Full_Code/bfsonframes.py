"""
This code will add a buffer (using BFS) to account for drone size.
"""
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
            if matrix2[i][j] == 255:
                q.put([i, j])
    while t > 0:
        n = q.qsize()
        while n > 0:
            top = q.get()
            matrix[top[0]][top[1]] = 255
            for ddir in dirs:
                x1 = top[0]+ddir[0]
                y1 = top[1]+ddir[1]
                if 0 <= x1 < len(matrix) and 0 <= y1 < len(matrix2[0]) and visited[x1][y1] == 0:
                    q.put([x1, y1])
                    visited[x1][y1] = 255
            n = n-1
        t = t-1
    return matrix2

def create3DMatrix(file, start, end, x, y, buffer, step):
    grids = []
    channel_values = open(file).read().split()
    for height in np.arange(start, end+step, step):
        matrix = []
        counter = 0
        for j in range(y):
            for i in range(x):
                if height - float(channel_values[counter]) <= 5.0:
                    matrix.append(255)
                else:
                    matrix.append(0)
                counter += 1
        matrix = [matrix[i:i + x] for i in range(0, len(matrix), x)]
        # Run BFS
        matrix = bfs(matrix, matrix, buffer)
        grids.append(matrix)
    return grids
