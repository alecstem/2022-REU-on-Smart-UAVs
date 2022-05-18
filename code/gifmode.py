"""
This code creates a .gif file of obstructions from
LiDAR data (in black) for a specified height range.
"""
from PIL import Image, ImageOps, ImageDraw

START = 198
END = 245

channel_values = open("lidar2.txt").read().split()

frames = []
img = Image.new('RGB', (297, 425), "white")
pixels = img.load()
draw = ImageDraw.Draw(img)

test = float(START)

for height in range(START, END):
    counter = 0
    img1 = Image.new('RGB', (297, 425), "white")
    pixels2 = img1.load()
    draw.rectangle([(0, 0), img.size], fill=(height, height, height))
    for i in range(img.size[0]):
        for j in range(img.size[1]):
            if float(channel_values[counter]) >= test:
                pixels2[i, j] = (0, 0, 0)
            counter += 1
    # img = ImageOps.flip(img)
    # img1.show()
    frames.append(img1)
    test += 1.0

print(len(frames))

# imageio.mimsave('cross2.gif', frames)
frames[0].save('cross.gif',save_all=True, append_images=frames[1:], optimize=False, duration=110, loop=0)


