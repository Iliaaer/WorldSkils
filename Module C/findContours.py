import cv2 as cv
import numpy as np

def detectFlag(image, d=0):
    bgr_min = np.array([0, 0, 240])    
    bgr_max = np.array([80, 80, 255])

    mask = cv.inRange(image, bgr_min, bgr_max)

    contours, h = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    cv.drawContours(image, contours, -1, (0,255,0), 3)

    MaxMin = [-1, float("inf"), -1,  float("inf")]

    
    for contour in contours:
        xMax = max(contour[:, 0, 0])
        yMax = max(contour[:, 0, 1])
        xMin = min(contour[:, 0, 0])
        yMin = min(contour[:, 0, 1])
        
        if xMax > MaxMin[0]:
            MaxMin[0] = xMax
        if xMin < MaxMin[1]:
            MaxMin[1] = xMin
        if yMax > MaxMin[2]:
            MaxMin[2] = yMax
        if yMin < MaxMin[3]:
            MaxMin[3] = yMin
    ##    print(xMax, xMin, yMax, yMin, (xMax+xMin)//2, (yMax+yMin)//2)

    lenContours = len(contours)
    if d:    
        print(image.shape)
        print(len(contours))
        print(MaxMin)
        print((MaxMin[0] - MaxMin[1]) // 2 + MaxMin[1], (MaxMin[2] - MaxMin[3]) // 2 + MaxMin[3])
        cv.imshow('1', image)
        cv.waitKey()
        cv.destroyAllWindows()
    if lenContours == 0:
        return 0
    if lenContours == 1:
        lenContours = "japan"
    elif lenContours == 2:
        lenContours = "switzerland"
    else:
        lenContours = "canada"
    return lenContours, (MaxMin[0] - MaxMin[1]) // 2 + MaxMin[1], (MaxMin[2] - MaxMin[3]) // 2 + MaxMin[3]
    
    

print(detectFlag(cv.imread("Canada.png")))
print(detectFlag(cv.imread("Switzerland.png")))
print(detectFlag(cv.imread("Japan.png")))

cv.destroyAllWindows()
