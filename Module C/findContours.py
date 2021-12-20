import cv2 as cv
import numpy as np

bgr_min = np.array([0, 0, 150])
bgr_max = np.array([50, 50, 255])

# image = cv.imread("1.png")
image = cv.imread("1_1.png")


mask = cv.inRange(image, bgr_min, bgr_max)

c, h = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

cv.drawContours(image, c, -1, (0,255,0), 3)
print(len(c))
#print(h)
#print("1")

MaxMin = [-1, float("inf"), -1,  float("inf")]

for i in c:
#    print(i[:,0, 0])
    xMax = max(i[:, 0, 0])
    yMax = max(i[:, 0, 1])
    xMin = min(i[:, 0, 0])
    yMin = min(i[:, 0, 1])
    if xMax > MaxMin[0]:
        MaxMin[0] = xMax
    if xMin < MaxMin[1]:
        MaxMin[1] = xMin
    if yMax > MaxMin[2]:
        MaxMin[2] = yMax
    if yMin < MaxMin[3]:
        MaxMin[3] = yMin
    print(xMax, xMin, yMax, yMin, (xMax+xMin)//2, (yMax+yMin)//2)

print(MaxMin)
print((MaxMin[0] - MaxMin[1]) // 2 + MaxMin[1], (MaxMin[2] - MaxMin[3]) // 2 + MaxMin[3])
cv.imshow('1', image)
cv.waitKey()

