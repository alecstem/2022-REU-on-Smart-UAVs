#this allows for las cordinates to go to lats and longs
# and then back if needed

import geopandas
from geoapis import lidar
import json
import requests
import math
import pyproj #this is a library that changes las to lats and longs

RADIUS = 6378137.0 # in meters on the equator, will never change

lats1 = input('enter the lats: ')
#print(lats1) #takes in the lats of the desired location
longs1 = input('enter the longs: ')
#print(longs1) #takes in the longs of the desired location

def lat2y(a): #takes in the lats value and runs it in this equation to change it to the las format
  return math.log(math.tan(math.pi / 4 + math.radians(a) / 2)) * RADIUS

def lon2x(a): #takes in the longs value and runs it in this equation to change it to the las format
  return math.radians(a) * RADIUS

print('latitude web mercator y: ', lat2y(float(lats1)), ' longitude web mercator x: ', lon2x(float(longs1)))
#takes uses the lon2x and lat2y def to convert and display

lats2 = input('enter the lats: ')
#print(lats2) #takes in the lats of the desired location
longs2 = input('enter the longs: ')
#print(longs2) #takes in the longs of the desired location

print('latitude web mercator y: ', lat2y(float(lats2)), ' longitude web mercator x: ', lon2x(float(longs2)))
#takes uses the lon2x and lat2y def to convert and display

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

#print('the x min is: ', xmin, ' the y min is: ', ymin, ' the x max is: ', xmax, ' the y max is: ', ymax)

#y1 = input('enter the y value: ')
#print(y1) #takes in y value of the las format
#x1 = input('enter the x value: ')
#print(x1) #takes in x value of las format

#transformer = pyproj.Transformer.from_crs("epsg:3857", "epsg:4326")
#using the pyproj it changes the type of interprtation to the las format used for Open Topoghpy
#print('conversion is(lats,longs): ')
#print(transformer.transform(x1, y1)) #displays the x and y for las format

#response = requests.get("https://portal.opentopography.org/API/otCatalog?productFormat=PointCloud&minx={}&miny={}&maxx={}&maxy={}&detail=false&outputFormat=json".format(str(-117.4)) .format(str(33.4)) .format(str(-115.5)) .format(str(34.9)))
#print(response.status_code)

#print(response.json())

payload = {'productFormat': 'PointCloud', 'minx': str(xmin), 'miny': str(ymin), 'maxx': str(xmax), 'maxy': str(ymax), 'detail': 'false', 'outputFormat': 'json'}
r = requests.get('https://portal.opentopography.org/API/otCatalog?', params=payload)

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
#for element in mylines:                     # For each element in the list,
    #print(element)

#print(mylines[0].find("name"))
myfile.close()

str = (str(response.json()))
arr = str.split('\'')
print(arr[7])

#jsondata = str(response.json())#""""[
 #{
  #"dataset":"datasets",
  #"name": "Blacks Moutain Experiental"
 #},
 #{
  #"name":"Eraser",
  #"unit_price":3
 #}
#]"""

# load the json data
#datasets = json.loads(jsondata)

# Input the item name that you want to search
#dataset = input("Enter an item name:\n")

# Define a function to search the item
#def search_price (dataset):
 #for keyval in datasets:
  #if dataset.lower() == keyval['dataset'].lower():
   #return keyval['name']

# Check the return value and print message
#if (search_price(dataset) != None):
 # print("The price is:", search_price(dataset))
#else:
 # print("Item is not found")


#import pathlib
#import typing

#import geoapis.lidar

#lidar_fetcher = geoapis.lidar.OpenTopography(cache_path=r"local/path/to/folder/to/save/files", verbose=True)
#lidar_fetcher.run(arr[7])#USGS LPC AL 25Co B3 2017






class OpenTopography:
    """ A class to manage fetching LiDAR data from Open Topography
    API details for querying datasets within a search rectangle at:
        https://portal.opentopography.org/apidocs/#/Public/getOtCatalog
    Information for making a `bulk download` of a dataset using the AWS S3 protocol can be found by clicking on bulk
    download under any dataset.
    """

    #def __init__(self, cache_path: typing.Union[str, pathlib.Path],
     #            search_polygon: geopandas.geodataframe.GeoDataFrame, redownload_files: bool = False,
      #           download_limit_gbytes: typing.Union[int, float] = 100, verbose: bool = False):

        #...

    #def run(self):
        #""" Download LiDAR dataset(s) either within a search_polygon, by name, or both """

#lidar.OpenTopography('ot_CL1_WLG_2013_1km_094045.laz')
