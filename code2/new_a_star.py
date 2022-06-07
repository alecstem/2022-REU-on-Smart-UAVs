import numpy as np
import heapq
import math
import time
import bfsonframes
import lidar_to_grid
from laspy.file import File

# visualization lib
from vedo import *
from vedo import Text, Cube, Line, Grid, merge, show
import natsort
from PIL import Image
import imageio


# PriorityQueue
class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return not self.elements

    def put(self, coordination, priority):
        heapq.heappush(self.elements, (priority, coordination))

    def get(self, ):
        return heapq.heappop(self.elements)[1]


class astar_3d_algo:
    def __init__(self, mapsize, start_point, end_point, cost_map):
        self.map_size = mapsize
        self.start_point = start_point
        self.end_point = end_point
        self.cost_map = cost_map

    # Get neighbors of current position
    def get_neighbors(self, current, matrix):
        neighbors = set()
        # current[2] = z, current[1] = y, current[0]=x
        # print('matrix size:', [len(matrix), len(matrix[0]), len(matrix[0][0])])
        # Right
        if current[0] + 1 < self.map_size[0] and matrix[current[2]][current[1]][current[0]+1] == 0:
            neighbors.add((current[0] + 1, current[1], current[2]))
        # Up
        if current[1] - 1 >= 0 and matrix[current[2]][current[1]-1][current[0]] == 0:
            neighbors.add((current[0], current[1] - 1, current[2]))
        # Left
        if current[0] - 1 >= 0 and matrix[current[2]][current[1]][current[0]-1] == 0:
            neighbors.add((current[0] - 1, current[1], current[2]))
        # Down
        if current[1] + 1 < self.map_size[1] and matrix[current[2]][current[1]+1][current[0]] == 0:
            neighbors.add((current[0], current[1] + 1, current[2]))
        # ↗
        if current[0] + 1 < self.map_size[0] and current[1] - 1 >= 0 and \
                matrix[current[2]][current[1]-1][current[0]+1] == 0:
            neighbors.add((current[0] + 1, current[1] - 1, current[2]))
        # ↘
        if current[0] + 1 < self.map_size[0] and current[1] + 1 < self.map_size[1] and \
                matrix[current[2]][current[1]+1][current[0]+1] == 0:
            neighbors.add((current[0] + 1, current[1] + 1, current[2]))
        # ↖
        if current[0] - 1 >= 0 and current[1] - 1 >= 0 and \
                matrix[current[2]][current[1]-1][current[0]-1] == 0:
            neighbors.add((current[0] - 1, current[1] - 1, current[2]))
        # ↙
        if current[0] - 1 >= 0 and current[1] + 1 < self.map_size[1] and \
                matrix[current[2]][current[1]+1][current[0]-1] == 0:
            neighbors.add((current[0] - 1, current[1] + 1, current[2]))
        # Z-Up
        if current[2] - 1 >= 0 and matrix[current[2]-1][current[1]][current[0]] == 0:
            neighbors.add((current[0], current[1], current[2] - 1))
        # Z-Down
        if current[2] + 1 < self.map_size[2] and matrix[current[2]+1][current[1]][current[0]] == 0:
            neighbors.add((current[0], current[1], current[2] + 1))
        # Z-Up Left
        if current[2] + 1 < self.map_size[2] and current[0] - 1 >= 0 and matrix[current[2]+1][current[1]][current[0]-1] == 0:
            neighbors.add((current[0] - 1, current[1], current[2] + 1))
        # Z-Up Right
        if current[2] + 1 < self.map_size[2] and current[0] + 1 < self.map_size[0] and \
                matrix[current[2]+1][current[1]][current[0]+1] == 0:
            neighbors.add((current[0] + 1, current[1], current[2] + 1))
        # Z-Up Up
        if current[2] + 1 < self.map_size[2] and current[1] - 1 >= 0 and \
                matrix[current[2]+1][current[1]-1][current[0]] == 0:
            neighbors.add((current[0], current[1] - 1, current[2] + 1))
        # Z-Up Down
        if current[2] + 1 < self.map_size[2] and current[1] + 1 < self.map_size[1] and \
                matrix[current[2]+1][current[1]+1][current[0]] == 0:
            neighbors.add((current[0], current[1] + 1, current[2] + 1))
        # Z-Down Left
        if current[2] - 1 >= 0 and current[0] - 1 >= 0 and \
                matrix[current[2]-1][current[1]][current[0]-1] == 0:
            neighbors.add((current[0] - 1, current[1], current[2] - 1))
        # Z-Down Right
        if current[2] - 1 >= 0 and current[0] + 1 < self.map_size[0] and \
                matrix[current[2]-1][current[1]][current[0]+1] == 0:
            neighbors.add((current[0] + 1, current[1], current[2] - 1))
        # Z-Down Up
        if current[2] - 1 >= 0 and current[1] - 1 >= 0 and \
                matrix[current[2]-1][current[1]-1][current[0]] == 0:
            neighbors.add((current[0], current[1] - 1, current[2] - 1))
        # Z-Down Down
        if current[2] - 1 >= 0 and current[1] + 1 < self.map_size[1] and \
                matrix[current[2]-1][current[1]+1][current[0]] == 0:
            neighbors.add((current[0], current[1] + 1, current[2] - 1))
        # Z-Up ↗
        if current[2] + 1 < self.map_size[2] and current[0] + 1 < self.map_size[0] and \
                current[1] - 1 >= 0 and matrix[current[2]+1][current[1]-1][current[0]+1] == 0:
            neighbors.add((current[0] + 1, current[1] - 1, current[2] + 1))
        # Z-Up ↘
        if current[2] + 1 < self.map_size[2] and current[0] + 1 < self.map_size[0] and \
                current[1] + 1 < self.map_size[1] and matrix[current[2]+1][current[1]+1][current[0]+1] == 0:
            neighbors.add((current[0] + 1, current[1] + 1, current[2] + 1))
        # Z-Up ↙
        if current[2] + 1 < self.map_size[2] and current[0] - 1 >= 0 and \
                current[1] + 1 < self.map_size[1] and matrix[current[2]+1][current[1]+1][current[0]-1] == 0:
            neighbors.add((current[0] - 1, current[1] + 1, current[2] + 1))
        # Z-Up ↖
        if current[2] + 1 < self.map_size[2] and current[0] - 1 >= 0 and \
                current[1] - 1 >= 0 and matrix[current[2]+1][current[1]-1][current[0]-1] == 0:
            neighbors.add((current[0] - 1, current[1] - 1, current[2] + 1))
        # Z-Down ↗
        if current[2] - 1 >= 0 and current[0] + 1 < self.map_size[0] and \
                current[1] - 1 >= 0 and matrix[current[2]-1][current[1]-1][current[0]+1] == 0:
            neighbors.add((current[0] + 1, current[1] - 1, current[2] - 1))
        # Z-Down ↘
        if current[2] - 1 >= 0 and current[0] + 1 < self.map_size[0] and \
                current[1] + 1 < self.map_size[1] and matrix[current[2]-1][current[1]+1][current[0]+1] == 0:
            neighbors.add((current[0] + 1, current[1] + 1, current[2] - 1))
        # Z-Down ↙
        if current[2] - 1 >= 0 and current[0] - 1 >= 0 and \
                current[1] + 1 < self.map_size[1] and matrix[current[2]-1][current[1]+1][current[0]-1] == 0:
            neighbors.add((current[0] - 1, current[1] + 1, current[2] - 1))
        # Z-Down ↖
        if current[2] - 1 >= 0 and current[0] - 1 >= 0 and \
                current[1] - 1 >= 0 and matrix[current[2]-1][current[1]-1][current[0]-1] == 0:
            neighbors.add((current[0] - 1, current[1] - 1, current[2] - 1))
        return neighbors

    def get_distance(self, current, end_point):
        distance = np.sqrt(np.abs(current[0] - end_point[0]) ** 2 + np.abs(current[1] - end_point[1]) ** 2 + np.abs(
            current[2] - end_point[2]) ** 2)
        return distance

    '''
    def get_distance(self, current, end_point):  # this is only the manhatten distance
        distance = np.sqrt(abs(current[0] - end_point[0]) ** 2 + abs(current[1] - end_point[1]) ** 2 + abs(
            current[2] - end_point[2]) ** 2)
        return distance
    def get_distance_angles(self, current, end_point):
        distance = np.sqrt(2) * abs(current[0] - end_point[0]) + abs(current[1] - end_point[1]) + abs(
            current[2] - end_point[2])
        return distance
    def get_distance_top_corners(self, current, end_point):
        distance = np.sqrt(3) * abs(current[0] - end_point[0]) + abs(current[1] - end_point[1]) + abs(
            current[2] - end_point[2])
        return distance
    '''

    # Find the path
    def find_path(self, matrix):
        frontier = PriorityQueue()
        frontier.put(self.start_point, 0)

        came_from = dict()
        came_from[self.start_point] = None

        cost_value = dict()
        cost_value[self.start_point] = 0

        # Searching Grid Map
        st = time.time()
        while not frontier.empty():
            current = frontier.get()
            if current == self.end_point:
                break
            neighbors = self.get_neighbors(current, matrix)
            for next in neighbors:
                # if (next[0]<41):
                # print(next[0], next[1], next[2])
                func = (current[0] - next[0]) * (current[0] - next[0]) + (current[1] - next[1]) * (current[1] - next[1]) + (current[2] - next[2]) * (current[2] - next[2])
                # new_cost = cost_value[current] + np.sqrt((current[0]-next[0])*(current[0]-next[0]) + (current[1]-next[1])*(current[1]-next[1]) + (current[2]-next[2])*(current[2]-next[2])) + self.cost_map[next]
                # new_cost = cost_value[current] + self.cost_map[next]
                # print(new_cost)
                try:
                    new_cost = cost_value[current] + (func ** 0.5) + self.cost_map[(next[2], next[1], next[0])]
                    # new_cost = cost_value[current] + self.cost_map[(next[2], next[1], next[0])]
                except IndexError as e:
                    print('current: ', [current])
                    print('next: ', [next])
                    print('map size: ', self.map_size)
                    print(e)
                    # print(matrix[next[2], next[1], next[0]])
                if next not in cost_value or new_cost < cost_value[next]:
                    cost_value[next] = new_cost
                    # Dijkstra's Algorithm
                    # priority = new_cost
                    # A* Algorithm
                    priority = new_cost + self.get_distance(next, self.end_point)
                    frontier.put(next, priority)
                    came_from[next] = current
        et = time.time()
        print((et - st))

        # Reconstruction path --> (Backwards from the goal to the start)
        current = self.end_point
        path = []
        while current != self.start_point:
            path.append(current)
            current = came_from[current]
        path.append(self.start_point)
        path.reverse()
        return path


# --------------------------------------------------------------------------------

def generate_path(start_point, end_point, map_size, matrix):
    # A* Algorithm
    # Define Map

    '''
    for z1 in range(len(matrix)):
        for y1 in range(len(matrix[0])):
            for x1 in range(len(matrix[0][0])):
                if matrix[z1, y1, x1] == 0:
                    matrix[z1, y1, x1] = 255
                else:
                    matrix[z1, y1, x1] = 0
    '''

    # A* Algorithm
    path_finder = astar_3d_algo(map_size, start_point, end_point, matrix)
    path = path_finder.find_path(matrix)
    best_path = [list(item) for item in path]
    return best_path