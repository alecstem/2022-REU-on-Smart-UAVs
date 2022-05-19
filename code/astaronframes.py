"""
This code will execute A* on every cross-sectional frame of a LiDAR scan.
"""
from PIL import Image, ImageOps, ImageDraw
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
import numpy as np

START = 5
END = 150
X = 140
Y = 160

channel_values = open("newyork1.asc").read().split()

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
    # img1.show()
    matrix = [matrix[i:i + X] for i in range(0, len(matrix), X)]
    grid = Grid(matrix=matrix)
    start = grid.node(5, 5)
    end = grid.node(135, 155)
    finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
    path, runs = finder.find_path(start, end, grid)
    pixels2 = img1.load()
    for i, j in path:
        pixels2[i, j] = (255, 0, 0)
    # img1.show()
    frames.append(img1)
    test += 1.0

frames[0].save('cross4.gif',save_all=True, append_images=frames[1:], optimize=False, duration=100, loop=0)


# 1 = free
# 0 = obstacle





