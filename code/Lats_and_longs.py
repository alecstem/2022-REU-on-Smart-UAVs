#this allows for las cordinates to go to lats and longs
# and then back if needed

import math
import pyproj #this is a library that changes las to lats and longs

RADIUS = 6378137.0 # in meters on the equator, will never change

lats = input('enter the lats: ')
print(lats) #takes in the lats of the desired location
longs = input('enter the longs: ')
print(longs) #takes in the longs of the desired location

def lat2y(a): #takes in the lats value and runs it in this equation to change it to the las format
  return math.log(math.tan(math.pi / 4 + math.radians(a) / 2)) * RADIUS

def lon2x(a): #takes in the longs value and runs it in this equation to change it to the las format
  return math.radians(a) * RADIUS

print('latitude web mercator y: {} longitude web mercator x: {}'.format(lat2y(lats), lon2x(longs)))
#takes uses the lon2x and lat2y def to convert and display

y1 = input('enter the y value: ')
print(y1) #takes in y value of the las format
x1 = input('enter the x value: ')
print(x1) #takes in x value of las format

transformer = pyproj.Transformer.from_crs("epsg:3857", "epsg:4326")
#using the pyproj it changes the type of interprtation to the las format used for Open Topoghpy
print('conversion is(lats,longs): ')
print(transformer.transform(x1, y1)) #displays the x and y for las format
