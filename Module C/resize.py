import cv2 as cv

image = cv.imread("switzerland.png")

imageResize = cv.resize(image, (250, 250))

cv.imshow('1', imageResize)

cv.imwrite("switzerland250.png", imageResize)

cv.waitKey()
