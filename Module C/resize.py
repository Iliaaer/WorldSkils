import cv2 as cv

image = cv.imread("switzerland.png")

imageResize = cv.resize(image, (100, 100))

cv.imshow('1', imageResize)

cv.imwrite("switzerlandNew.png", imageResize)

cv.waitKey()
