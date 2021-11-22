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

image_color = rospy.Publisher("/Debug",Image,queue_size=10) 

def navigate_wait(x=0, y=0, z=1.5, speed=1, yaw=float('nan'), frame_id='aruco_map', auto_arm=False:
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < 0.2:
            break
        rospy.sleep(0.2)

navigate_wait(frame_id='body', auto_arm=True)
navigate_wait(frame_id='aruco_36')
land()
