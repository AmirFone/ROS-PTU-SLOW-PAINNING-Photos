#!/usr/bin/env python
# 
#
# Program that
# moves the pan tilt unit
# to close of program by ^C
# dml 2022
#
#script wrtie my Amir H for Fordham university's Computer Vision and Robotics Lab.
#Email:amirmd6000@gmail.com



import time
import math
import random
import rospy # needed for ROS
import numpy as np # for map arrays
import matplotlib.pyplot as plt
#from matplotlib.image import imread  


# ROS message types
from sensor_msgs.msg import JointState       # ROS DP PTU command
from geometry_msgs.msg import Twist      # ROS Twist message



import cv2
#from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

gCurrentImage = CompressedImage() # make a global variable for the image
gBridge = CvBridge()    # make a ROS to CV bridge object
gLoc = [0,0,0]          # location of robot


# ROS topics

motionTopic='/P2/RosAria/cmd_vel' 
poseTopic = '/P2/RosAria/pose'
imageTopic = '/P2/camera/color/image_raw/compressed' 


motionTopic='/P2/RosAria/cmd_vel' 
ptuCmdTopic = '/P2/ptu/cmd'          # check that these are right
ptuPosTopic = '/joint_states'

#globals

gPTU = [0.0, 0.0]
gCtr=0
gWait=False
flag=False 
gImageList =[]

# get current PT position from PTU
# ignores velocity, effort and finished flag
#
def ptuCallback(data):
    global gPTU,gWait,gCtr,gImageList,flag
    gPTU=data.position
    gCtr += 1
    temp_pan=math.degrees(gPTU[0])
    temp_tilt=math.degrees(gPTU[1])
    if (temp_pan<=120 and temp_pan>=119) and flag==True and len(gImageList)==0:
        gImageList.append(gCurrentImage)	
        print("  <<Image captured for>> ",temp_pan)
    elif (temp_pan<=60 and temp_pan>=59) and flag==True and len(gImageList)==1:
        gImageList.append(gCurrentImage)
        print("  <<Image captured for>> ",temp_pan)
    elif (temp_pan<=0 and temp_pan>=-1) and flag==True and len(gImageList)==2:
        gImageList.append(gCurrentImage)
        print("  <<Image captured for>> ",temp_pan)
    elif (temp_pan<=-57 and temp_pan>=-60) and flag==True and len(gImageList)==3:
        gImageList.append(gCurrentImage)
        print("  <<Image captured for>> ",temp_pan)
    elif (temp_pan<=-117 and temp_pan>=-120) and flag==True and len(gImageList)==4:
        gImageList.append(gCurrentImage)
        print("  <<Image captured for>> ",temp_pan)
    elif (temp_pan<=-179 and temp_pan>=-180) and flag==True:
	flag=False        
	gImageList.append(gCurrentImage)
        print("  <<Image captured for>> ",temp_pan)

    if gCtr>40 and not gWait: # output the PTU position every few secs
        print("PT position [pan,tilt]:",temp_pan,temp_tilt)
        gCtr=0
    return

def callbackImage(img):
    '''Called automatically for each new image'''
    global gCurrentImage, gBridge
    #gCurrentImage = gBridge.imgmsg_to_cv2(img, "bgr8")
    np_arr=np.fromstring(img.data,np.uint8)
    gCurrentImage=cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
    return
#
# publish a command to move the ptu unit
# pub=publisher, seq = message sequence number (monotinic increasing)
#
def camera_view():
    count=0
    for i in gImageList:
    	cv2.imshow('Camera',i)
    	cv2.waitKey(1)
        temp="p2_imageSlowPanning"+str(count)+".jpg"
    	cv2.imwrite(temp,i)
        count+=1
	rospy.sleep(5)
    return

def sendPTcommand(pub,p,t,seq):
    msg = JointState()
    msg.header.frame_id='0'
    msg.header.stamp=rospy.Time.now()
    msg.name=["pan","tilt"]
    msg.position = [p,t]
    msg.velocity=[.3,.3]
    msg.effort=[1,1]
    pub.publish(msg)

    return seq+1


#
# move the PT unit
# ask user for input pan and tilt settings and
# send them to the PT

def PTU_test_node():
    global gCurrentImage,gImageList, gPTU,gWait,gCtr,flag
    '''send pt commands to the pt unit'''
    global gWait
    # all ROS 'nodes' (ie your program) have to do the following
    rospy.init_node('PTU_test_Node', anonymous=True)

    # register as a ROS publisher for the PTU position
    ptu_pub = rospy.Publisher(ptuCmdTopic, JointState, queue_size=10)
    ptu_sub = rospy.Subscriber(ptuPosTopic, JointState,ptuCallback)
    image_sub = rospy.Subscriber(imageTopic,CompressedImage,callbackImage)
    msg_seq_num=0
    while not rospy.is_shutdown() and gWait==False:
	    #msg_seq_num = sendPTcommand(ptu_pub,30,10,msg_seq_num)
	    #rospy.sleep(1)
	    msg_seq_num = sendPTcommand(ptu_pub,0,0,msg_seq_num)
	    while math.degrees(gPTU[0])!=0:
		rospy.sleep(1)
	    print("Camera at origin")
	    rospy.sleep(1)
	    msg_seq_num = sendPTcommand(ptu_pub,math.radians(121.0),0,msg_seq_num)
	    #rospy.sleep(3)
	    while math.degrees(gPTU[0])<= 120.5:
		rospy.sleep(1)
	    print("Camera at 120")
            rospy.sleep(1)
            flag=True
	    msg_seq_num = sendPTcommand(ptu_pub,math.radians(-180.0),0,msg_seq_num) 
	    #rospy.sleep(6)
	    while math.degrees(gPTU[0])>= -179:
		rospy.sleep(1)
	    print("Camera at 180")
            rospy.sleep(1)
            msg_seq_num = sendPTcommand(ptu_pub,0,0,msg_seq_num)
            while math.degrees(gPTU[0])<0:
		rospy.sleep(1)
	    print("Camera back at origin")
    	    gWait=True
    camera_view()
    return

#
# This function is called by ROS when you stop ROS
# Here we use it to send a zero velocity to robot
# in case it was moving when you stopped ROS
#

def callback_shutdown():
    print("Shutting down")
    pub = rospy.Publisher(motionTopic, Twist, queue_size=1)
    msg = Twist()
    msg.angular.z=0.0
    msg.linear.x=0.0
    pub.publish(msg) 
    return



#-------------------------------MAIN  program----------------------
if __name__ == '__main__':
    try:
	rospy.on_shutdown(callback_shutdown)
	PTU_test_node()        
    except rospy.ROSInterruptException:
        pass

