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

def lat_to_point(input1, input2, num):
    #convert lat and longs to point on point cloud
    print(input1)
    print(input2)
    print(num)
    print('epsg:' + num)
    transformer = pyproj.Transformer.from_crs("epsg:4326", str("epsg:" + num))  # 3857, 4326, 2153
    print(transformer.transform(input1, input2))
    print(utm.from_latlon((input1), (input2)))
    con = (utm.from_latlon((input1), (input2)))
    print(con[0])
    print(con[1])

    temp0 = (max[0]-con[0]) / 100
    temp1 = (con[1]-min[1]) / 100
    print(temp0)
    print(temp1)
    return [temp0, temp1]
