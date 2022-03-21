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
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)

nLeds = 58
lZ = 1

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
    bgr_min = np.array([0, 0, 210])    
    bgr_max = np.array([80, 80, 255])

    # img = image[80: 160, 120: 200]
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
        cv.imshow('1', img)
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
    

def navigate_wait(x=0, y=0, z=lZ, speed=0.5, frame_id='aruco_map', auto_arm=False):
    navigate(x=x, y=y, z=z, speed=speed, yaw=float('nan'), frame_id=frame_id, auto_arm=auto_arm)
    
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < 0.2:
            break
        rospy.sleep(0.2)
    
    set_position(x=x, y=y, z=z, frame_id=frame_id)
    # rospy.sleep(3)

def go(x, y):
    global detect_count 
    global count
    navigate_wait(x=x, y=y)
    set_position(x=x, y=y, z=lZ, frame_id="aruco_map")
    rospy.sleep(1)
    image = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    img = image.copy()[80: 180, 120: 220]
    
    
    # cv.imwrite("flag" + str(count) + ".png", img)
    detect = detectFlag(img)
    # cv.imwrite("flag" + str(count) + "detect.png", img)
    
    
    print(detect)
    led(detect[0])
    detect_count.append([detect[0], x, y])
    count += 1


# led('japan')

navigate_wait(frame_id='body', auto_arm=True)
set_position(x=0, y=0, z=lZ, frame_id="aruco_map")

count = 0
n = 4
detect_count = []

go(0.5, 2)

go(1.5, 0)

go(2.5, 1)

'''
for i in range(n*2):
    if count == 3:
        break
    for j in range((n-1)*2):
        if count == 3:
            break
        
        if i % 2 == 0:
            print(j*0.5, i)
            navigate_wait(x=j*0.5, y=i)
            set_position(x=j*0.5, y=i, z=0.5, frame_id="aruco_map")
        else:
            print(n-j*0.5, i)
            navigate_wait(x=(n-1)-j*0.5, y=i)
            set_position(x=(n-1)-j*0.5, y=i, z=0.5, frame_id="aruco_map")
        img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
        detect = detectFlag(img)
        print(detect)
        if len(detect) > 1:
            if detect[0] not in[element[0] for element in detect_count]:
                count += 1
                #cv.imwrite(detect[0] + str(count) + ".png", img)
                led(detect[0])
                detect_count.append([detect[0], j, i*0.5])
'''            
        


navigate_wait()
set_position(x=0, y=0, z=0.5, frame_id="aruco_map")
land()

print("="*20)
for i in detect_count:
    print(i)
print("="*20)

with open('C_report_fly.txt', 'w') as f:
    for d in detect_count:
        f.write("{} ({}, {})\n".format(*d))


