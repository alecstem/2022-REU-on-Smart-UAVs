'''
This is the starting point for the process. It connects all the files together and starts all the necessary retrieval of data.
'''
import requests
import os
import time
import json
from selenium import webdriver
from selenium.common import NoSuchElementException
from selenium.webdriver.common.by import By


"""
Auburn 2:
enter the first latitude: 32.60276209085292
enter the first longitude: -85.48741793407983
enter the second latitude: 32.604047626726015
enter the second longitude: -85.48593539307295

Sloss Furnance:
enter the first latitude: 33.5198
enter the first longitude: -86.7901
enter the second latitude: 33.5209
enter the second longitude: -86.7927

Statue of Liberty:
enter the first latitude: 40.689640
enter the first longitude: -74.045026
enter the second latitude: 40.688982
enter the second longitude: -74.043711


Disney: 
enter the first latitude: 28.4197
enter the first longitude: -81.5814
enter the second latitude: 28.4193
enter the second longitude: -81.5808
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

''' generates the .json format search for the databases using OpenTopography's API '''

payload = {'productFormat': 'PointCloud', 'minx': str(xmin), 'miny': str(ymin), 'maxx': str(xmax), 'maxy': str(ymax), 'detail': 'false', 'outputFormat': 'json', 'include_federated': 'true'}
r = requests.get('https://portal.opentopography.org/API/otCatalog?', params=payload)

# print(r.url)
''' converts the .json file to a notepad to extract vital information regarding that database'''
response = requests.get(r.url)
# print(response.json())

with open('readme1.txt', 'w') as f:
    f.write(str(response.json()))
f.close()

mylines = []
with open ('readme1.txt', 'rt') as myfile:
    for myline in myfile:
        mylines.append(myline.rstrip('\n'))

myfile.close()

stri = (str(response.json()))

arr = stri.split('\'')

file = json.loads(response.text)
driver = webdriver.Chrome()
def get_las(url):
    ''' This generates the website for the polygon in the dataset generated from the input data and then it signs into
    the website and downloads the .las file '''

    driver.find_element(By.ID, 'lazOutputFormat').click()

    driver.find_element(By.ID, 'lasOutputFormat').click()

    driver.find_element(By.ID, 'disableSendMail').click()

    driver.find_element(By.CSS_SELECTOR, '[value="Submit"]').click()

    result = ' '
    while result == ' ':
        try:
            result = driver.find_element(By.CSS_SELECTOR, 'span#statusId')
        except NoSuchElementException:
            pass
    while driver.find_element(By.CSS_SELECTOR, 'span#statusId').text[0:4] != 'Done':
        try:
            # print(driver.find_element(By.CSS_SELECTOR, 'span#statusId').text)
            time.sleep(.1)
        except NoSuchElementException:
            pass
    # time.sleep(6)
    driver.find_element(By.CSS_SELECTOR, '[download="points.las"]').click()


# name = []
time.sleep(2)
for i in range(0, len(file['Datasets'])):
    if file['Datasets'][i]['Dataset']['identifier']['propertyID'] == 'USGS_3DEP_ID':
        val = file['Datasets'][i]['Dataset']['identifier']['value']
        # print(val)
        url2 = 'https://portal.opentopography.org/usgsDataset?dsid=' + val + '&minX=' + str(xmin) + \
               '&minY=' + str(ymin) + '&maxX=' + str(xmax) + '&maxY=' + str(ymax)
        # print(url2)
        driver.get(url2)
        driver.maximize_window()
        driver.find_element(By.LINK_TEXT, 'Sign In').click()

        driver.find_element(By.NAME, 'userId').send_keys('your_email.edu') # in order to access USGS you must have a .edu email account

        driver.find_element(By.NAME, 'password').send_keys('your_password')

        driver.find_element(By.NAME, 'submit').click()

        if driver.find_element(By.ID, 'validate').text != 'The selection area contains no points.':
            # it is valid
            # print(url2)
            get_las(url2)
            break
    r2 = requests.get('https://portal.opentopography.org/logout')
    driver.get(r2.url)
    # time.sleep(2)
    # break

time.sleep(5)
if os.path.exists("points.las"):
  os.remove("points.las")
else:
  print("The file does not exist")

# route_to_downloaded_file , route_to_destination_where_code_is_located
os.rename(r"path\to\downloads\points.las", r"path\to\destination\points.las")

driver.minimize_window()
os.system("taskkill /im chrome.exe /f")


import path_visualization
''' This passes all of the required information about the file needed to genarate the path(the file must be 
downloaded into the same folder as this code) '''
path_visualization.run(float(lats1), float(longs1), arr[89])
