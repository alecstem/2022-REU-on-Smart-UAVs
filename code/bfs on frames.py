"""
This code will add a buffer (using BFS) to account for drone size.
"""
from PIL import Image, ImageOps, ImageDraw
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
import queue

def bfs(matrix2):
    visited = matrix2
    t = 3
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


START = 213
END = 260
X = 401
Y = 182

channel_values = open("auburn.asc").read().split()

frames = []
img = Image.new('RGB', (X, Y), "white")
pixels = img.load()
draw = ImageDraw.Draw(img)


test = float(START)

for height in range(START, END):
    matrix = []
    counter = 0
    img1 = Image.new('RGB', (X, Y), "white")
    pixels2 = img1.load()
    for y in range(img.height):
        for x in range(img.width):
            if float(channel_values[counter]) >= test:
                pixels2[x, y] = (0, 0, 0)
                matrix.append(0)
            else:
                matrix.append(1)
            counter += 1
    matrix = [matrix[i:i + X] for i in range(0, len(matrix), X)]
    # Run BFS
    # matrix = bfs(matrix)
    grid = Grid(matrix=matrix)
    start = grid.node(5, 5)
    end = grid.node(X-5, Y-5)
    finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
    path, runs = finder.find_path(start, end, grid)
    pixels2 = img1.load()
    for i, j in path:
        pixels2[i, j] = (255, 0, 0)
    # img1.show()
    frames.append(img1)
    test += 1.0

frames[0].save('cross_auburn_buffer2.gif', save_all=True, append_images=frames[1:], optimize=False, duration=230, loop=0)

# 1 = free
# 0 = obstacle
