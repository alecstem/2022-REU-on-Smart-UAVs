#this allows for las cordinates to go to lats and longs
# and then back if needed

import math
import pyproj

RADIUS = 6378137.0 # in meters on the equator

lats = input('enter the lats: ')
print(lats)
longs = input('enter the longs: ')
print(longs)

def lat2y(a):
  return math.log(math.tan(math.pi / 4 + math.radians(a) / 2)) * RADIUS

def lon2x(a):
  return math.radians(a) * RADIUS

ts_gm = [float(lats),float(longs)]
print('latitude web mercator y: {} longitude web mercator x: {}'.format(lat2y(ts_gm[0]), lon2x(ts_gm[1])))

y1 = input('enter the y value: ')
print(y1)
x1 = input('enter the x value: ')
print(x1)

transformer = pyproj.Transformer.from_crs("epsg:3857", "epsg:4326")
print('conversion is(lats,longs): ')
print(transformer.transform(x1, y1))
