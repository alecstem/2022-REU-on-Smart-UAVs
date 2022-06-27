'''
This is the starting point for the process. It connects all the files together and starts all the necessary retrieval of data.
'''
import requests
import os
import time
from selenium import webdriver

"""
32.60276209085292
-85.48741793407983

32.604047626726015
-85.48593539307295
"""


lats1 = input('enter the first latitude: ')
longs1 = input('enter the first longitude: ')

lats2 = input('enter the second latitude: ')
longs2 = input('enter the second longitude: ')


if float(longs1) > float(longs2):
  xmin = longs2
  xmax = longs1
else:
  xmin = longs1
  xmax = longs2

if float(lats1) > float(lats2):
  ymin = lats2
  ymax = lats1
else:
  ymin = lats1
  ymax = lats2

payload = {'productFormat': 'PointCloud', 'minx': str(xmin), 'miny': str(ymin), 'maxx': str(xmax), 'maxy': str(ymax), 'detail': 'false', 'outputFormat': 'json', 'include_federated': 'true'}
r = requests.get('https://portal.opentopography.org/API/otCatalog?', params=payload) #maybe try the usgs here?  https://portal.opentopography.org/dataCatalog?group=usgs

# print(r.url)

response = requests.get(r.url)
# print(response.json())

with open('readme1.txt', 'w') as f:
    f.write(str(response.json()))
f.close()

mylines = []                                # Declare an empty list.
with open ('readme1.txt', 'rt') as myfile:    # Open lorem.txt for reading text.
    for myline in myfile:                   # For each line in the file,
        mylines.append(myline.rstrip('\n')) # strip newline and add to list.

myfile.close()

stri = (str(response.json()))
arr = stri.split('\'')
# print(arr[7])#7 is the name
# print(arr[25]) #25 for some or 21
# print(arr[89])#85 for ny 97 for ca 89 for auburn
# print(arr[129])

temp = arr[7].replace(' ', '_')
# print(temp)

payload1 = {'dsid': str(temp), 'minX': str(xmin), 'minY': str(ymin), 'maxX': str(xmax), 'maxY': str(ymax)}
r1 = requests.get('https://portal.opentopography.org/usgsDataset?', params=payload1) #maybe try the usgs here?  https://portal.opentopography.org/dataCatalog?group=usgs
# print(r1.url)

driver = webdriver.Chrome()
driver.get(r1.url)
driver.maximize_window()
button = driver.find_element_by_link_text('Sign In')
button.click()

i = driver.find_element_by_name('userId')
i.send_keys('lcb0035@uah.edu')

k = driver.find_element_by_name('password')
k.send_keys('Open4me03!')

button1 = driver.find_element_by_name('submit')
button1.click()

button2 = driver.find_element_by_id('lazOutputFormat')
button2.click()

button3 = driver.find_element_by_id('lasOutputFormat')
button3.click()

button4 = driver.find_element_by_id('disableSendMail')
button4.click()

button5 = driver.find_element_by_css_selector('[value="Submit"]')
button5.click()

time.sleep(35)

button6 = driver.find_element_by_css_selector('[download="points.las"]')
button6.click()

time.sleep(5)
if os.path.exists("points.las"):
  os.remove("points.las")
else:
  print("The file does not exist")

os.rename(r"C:\Users\lcbba\Downloads\points.las", r"C:\Users\lcbba\PycharmProjects\full_test\full_test\points.las")

driver.minimize_window()
os.system("taskkill /im chrome.exe /f")


import path_visualization

path_visualization.run(float(lats1), float(longs1), arr[89])
