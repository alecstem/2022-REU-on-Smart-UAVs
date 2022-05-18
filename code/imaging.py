from PIL import Image, ImageOps

channel_values = open("lidar2.txt").read().split()

img = Image.new('RGB', (297, 425), "white")
pixels = img.load();
counter = 0

for i in range(img.size[0]):
    for j in range(img.size[1]):
        if float(channel_values[counter]) >= 215.0:
            # print(counter)
            pixels[i, j] = (0, 0, 0)
        counter += 1
img = ImageOps.flip(img)
img.show()
