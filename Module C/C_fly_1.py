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

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry) 
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger) 


bridge = CvBridge()

image_color = rospy.Publisher("/Debug", Image,queue_size=10) 

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
    

def navigate_wait(x=0, y=0, z=1, speed=0.25, yaw=float('nan'), frame_id='aruco_map', auto_arm=False):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < 0.2:
            break
        rospy.sleep(0.2)
        
    rospy.sleep(5)

navigate_wait(frame_id='body', auto_arm=True)

##navigate_wait(x=1, y=0.5, z=0.5) # Canada
##rospy.sleep(3)
##img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
### image_pub.publish(img)
### cv.imwrite("Canada.png", img)
##print(detectFlag(img))
##
##
##navigate_wait(x=2, y=0.5, z=0.5) # Germany
##rospy.sleep(3)
##img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
### image_pub.publish(img)
### cv.imwrite("Japan.png", img)
##print(detectFlag(img))
##
##navigate_wait(x=3, y=0.5, z=0.5) # French
##rospy.sleep(3)
##img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
### image_pub.publish(img)
### cv.imwrite("Switzerland.png", img)
##print(detectFlag(img))

count = 0
n = 4

for i in range(1, n+1):
    if count == 3:
        break
    for j in range(n):
        if count == 3:
            break
        navigate_wait(x=j*1.5, y=i*0.5, z=0.4)
        rospy.sleep(3)
        
        img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
        detect = detectFlag(img)
        print(detect)
        if len(detect) > 1:
            count += 1
            cv.imwrite(detect[0] + str(count) + ".png", img)
        


navigate_wait()
land()

