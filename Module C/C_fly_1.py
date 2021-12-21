# -*- coding: utf-8 -*-

import rospy
from clover import srv
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2 as cv
import math
import numpy as np
import math 
from led_msgs.srv import SetLEDs
from led_msgs.msg import LEDStateArray, LEDState


rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry) 
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger) 
set_leds = rospy.ServiceProxy('led/set_leds', SetLEDs) 

nLeds = 58

bridge = CvBridge()

image_color = rospy.Publisher("/Debug", Image,queue_size=10) 

def led(flag):
    nLeds = 58
    nRed = 10
    nWhite = nLeds - nRed
    if flag == "switzerland":
        nRed = 46
        nWhite = nLeds - nRed
    if flag == "canada":
        nRed = 37
        nWhite = nLeds - nRed

    led_msg = []
    for i in range(nRed):
        led_msg.append(LEDState(i, 255, 0, 0))
    for i in range(nRed, nLeds):
        led_msg.append(LEDState(i, 255, 255, 255))
    set_leds(led_msg)
    rospy.sleep(3)
    
    led_msg = []
    for i in range(nLeds):
        led_msg.append(LEDState(i, 0, 0, 0))
    set_leds(led_msg)
    rospy.sleep(1)


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
        return [0]
    if lenContours == 1:
        lenContours = "japan"
    elif lenContours == 2:
        lenContours = "switzerland"
    else:
        lenContours = "canada"
    return [lenContours, (MaxMin[0] - MaxMin[1]) // 2 + MaxMin[1], (MaxMin[2] - MaxMin[3]) // 2 + MaxMin[3]]
    

def navigate_wait(x=0, y=0, z=0.5, speed=0.25, frame_id='aruco_map', auto_arm=False):
    navigate(x=x, y=y, z=z, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < 0.2:
            break
        rospy.sleep(0.2)
        
    rospy.sleep(3)

led('japan')

navigate_wait(z=1, frame_id='body', auto_arm=True)
navigate_wait(z=0.5)
count = 0
n = 4
detect_count = []

for i in range(1, n+1):
    if count == 3:
        break
    for j in range(n+1):
        if count == 3:
            break
        navigate_wait(x=j*0.75, y=i*0.5)
        
        img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
        detect = detectFlag(img)
        print(detect)
        if len(detect) > 1:
            count += 1
            cv.imwrite(detect[0] + str(count) + ".png", img)
            led(detect[0])
            detect_count.append([detect[0], j, i*0.5])
            
        


navigate_wait()
land()

with open('С_report_fly.txt', 'w') as f:
    for d in detect_count:
        f.write("{} ({}, {})\n".format(*d))


