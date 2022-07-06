'''This is the code that will convert the pointcloud path to GPS path'''

import pyproj
import utm
import laspy

def convert_to_latlon(best_path, old_path, zone, hem):
    converted = []
    for i in range(len(best_path)):
        best_path[i][0] = best_path[i][0] / 100
        best_path[i][1] = best_path[i][1] / 100
        converted.append(list(utm.to_latlon(best_path[i][0], best_path[i][1], zone, hem)))
        converted[i].append(old_path[i][2])
    return converted
