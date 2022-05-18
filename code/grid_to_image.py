"""
This code generates an image that contains obstructions (in black)
at a specified altitude from LiDAR data.
"""
from PIL import Image, ImageOps


HEIGHT = 215.0
channel_values = open("lidar2.txt").read().split()

img = Image.new('RGB', (297, 425), "white")
pixels = img.load();
counter = 0

for i in range(img.size[0]):
    for j in range(img.size[1]):
        if float(channel_values[counter]) >= HEIGHT:
            # print(counter)
            pixels[i, j] = (0, 0, 0)
        counter += 1
img = ImageOps.flip(img)
img.show()
