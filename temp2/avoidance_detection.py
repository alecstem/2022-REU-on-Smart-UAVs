
def find_avoidances(matrix, best_path):
    # x y z
    best_avoidance = []
    map_size = [len(matrix[0][0]), len(matrix[0]), len(matrix)]
    dirs = [[1, 1, 1], [-1, 1, 1], [1, -1, 1], [-1, -1, 1], [1, 1, -1], [-1, 1, -1], [1, -1, -1], [-1, -1, -1]]
    best_avoidances = []
    for path in best_path:
        best_counter = 0
        for direc in dirs:
            counter = 0
            z = path[2]
            y = path[1]
            x = path[0]
            while 0 <= z < map_size[2] and 0 <= y < map_size[1] and 0 <= x < map_size[0] and matrix[z][y][x] == 1:
                z = z + direc[2]
                y = y + direc[1]
                x = x + direc[0]
                counter = counter + 1
            if counter > best_counter:
                best_counter = counter
                best_avoidance = [x, y, z]
        if (len(best_avoidance)>0):
            best_avoidances.append(best_avoidance)
        else:
            best_avoidances.append([path[0], path[1], path[2]])
    return best_avoidances



