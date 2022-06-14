import requests
import math
import utm
import geo_testing

RADIUS = 6378137.0 # in meters on the equator, will never change

"""
enter the lats: 33.400018
enter the longs: -117.399356
enter the lats: 33.399919
enter the longs: -117.399249
"""


lats1 = input('enter the lats: ')
#print(lats1) #takes in the lats of the desired location
longs1 = input('enter the longs: ')
#print(longs1) #takes in the longs of the desired location
print(utm.from_latlon(float(lats1), float(longs1)))
def lat2y(a): #takes in the lats value and runs it in this equation to change it to the las format
  return math.log(math.tan(math.pi / 4 + math.radians(a) / 2)) * RADIUS

def lon2x(a): #takes in the longs value and runs it in this equation to change it to the las format
  return math.radians(a) * RADIUS


lats2 = input('enter the lats: ')
#print(lats2) #takes in the lats of the desired location
longs2 = input('enter the longs: ')
#print(longs2) #takes in the longs of the desired location


if(float(longs1) > float(longs2)):
  xmin = longs2
  xmax = longs1
else:
  xmin = longs1
  xmax = longs2

if(float(lats1) > float(lats2)):
  ymin = lats2
  ymax = lats1
else:
  ymin = lats1
  ymax = lats2

payload = {'productFormat': 'PointCloud', 'minx': str(xmin), 'miny': str(ymin), 'maxx': str(xmax), 'maxy': str(ymax), 'detail': 'false', 'outputFormat': 'json', 'include_federated': 'true'}
r = requests.get('https://portal.opentopography.org/API/otCatalog?', params=payload) #maybe try the usgs here?  https://portal.opentopography.org/dataCatalog?group=usgs

print(r.url)

response = requests.get(r.url)
print(response.json())

with open('readme1.txt', 'w') as f:
    f.write(str(response.json()))
f.close()

mylines = []                                # Declare an empty list.
with open ('readme1.txt', 'rt') as myfile:    # Open lorem.txt for reading text.
    for myline in myfile:                   # For each line in the file,
        mylines.append(myline.rstrip('\n')) # strip newline and add to list.

myfile.close()

str = (str(response.json()))
arr = str.split('\'')
print(arr[7])#7 is the name
print(arr[25]) #25 for some or 21
print(arr[89])#85 for ny 97 for ca 89 for auburn


lidar_fetcher = geo_testing.OpenTopography(cache_path=r"C:\Users\lcbba\PycharmProjects\pythonProject\laz_files", verbose=True)
lidar_fetcher.run(arr[25])  # USGS LPC AL 25Co B3 2017      25 for alt name, 21 for auburn



#las = laspy.read('C:\\Users\\lcbba\\PycharmProjects\\pythonProject\\laz_files' + '\\' + arr[25])
#las = laspy.convert(las)
#las.write('C:\\Users\\lcbba\\PycharmProjects\\pythonProject\\laz_files' + '\\' + arr[25])

import path_visualization

path_visualization.run(xmin, ymin, xmax, ymax)
