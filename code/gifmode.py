from PIL import Image, ImageOps, ImageDraw

channel_values = open("lidar2.txt").read().split()

images = []
img = Image.new('RGB', (297, 425), "white")
pixels = img.load()
draw = ImageDraw.Draw(img)

test = 200.0

for height in range(200, 210):
    counter = 0
    draw.rectangle([(0, 0), img.size], fill=(255, 255, 255))
    for i in range(img.size[0]):
        for j in range(img.size[1]):
            if float(channel_values[counter]) >= test:
                pixels[i, j] = (0, 0, 0)
            counter += 1
    # img = ImageOps.flip(img)
    img.show()
    images.append(img)
    test += 1.0

print(len(images))

images[1].show()
images[2].show()
images[3].show()

images[0].save('cross.gif', save_all=True, append_images=images[1:], optimize=False, duration=40, loop=0)


