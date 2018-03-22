import cv2  					     # Import opencv library of python

black_pixel_count = 0
white_pixel_count = 0

img = cv2.imread('black_and_white.jpg', 1)
img = cv2.resize(img, (320, 240))    # Resize the image width x height
# x-axis = 320px, y-axis = 240px

# black = 0, white = 255
cv2.imshow('My Image', img)
cv2.waitKey(0)

for y in range(1, 240):
    for x in range(1, 320):
        px = img[y, x]                     # access the particular pixel
        if px[0] == px[1] == px[2] == 0:
            black_pixel_count += 1

        if px[0] == px[1] == px[2] == 255:
            white_pixel_count += 1

print('Black pixel count = ', black_pixel_count)
print('White pixel count = ', white_pixel_count)
