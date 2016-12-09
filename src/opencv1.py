#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import numpy as np


bridge = CvBridge() #Creates obj for sending poop btwn ros and opencv
pub = rospy.Publisher('opencv/center_of_object', Point)

def imagecb(data):
    global bridge

    #Converts Image message to CV image with blue-green-red color order (bgr8)
    try: #if no image, then throw out error but continue
        img_original = bridge.imgmsg_to_cv2(data, "bgr8")


#COLOR TRACKING
        #Converts bgr8 to hsv
        hsv = cv2.cvtColor(img_original, cv2.COLOR_BGR2HSV)

        #Def range of red color in HSV
        lower_red = np.array([-50, 100, 100])
        upper_red = np.array([10, 255, 255])
        #lower_red = np.array([0, 0, 100]) #blue value 95 -130 yellow 10 -60
        #upper_red = np.array([360, 330, 255]) #0-360 produces y b r
        #Def mask using set hsv range
        mask = cv2.inRange(hsv, lower_red, upper_red)
        mask = cv2.erode(mask,None,iterations=5)
        mask = cv2.dilate(mask,None,iterations=5)

        #Mask and original image overlay
        res = cv2.bitwise_and(img_original,img_original, mask= mask)

        #Creating b w image from res  (outputs binary matrices)
        ret, thresh = cv2.threshold(res[:,:,2], 100, 255, cv2.THRESH_BINARY)


        #Find and creat the circle
        cnts = cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        if len(cnts) > 0:
            c=max(cnts,key=cv2.contourArea)
            ((x,y),radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = Point()
            center.x = int(M["m10"] / M["m00"])
            center.y = int(M["m01"] / M["m00"])
            #center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            print center
            #print center.__class__
            height,width,depth = img_original.shape
            print height
            print width
            print depth
            pub.publish(center)
            # if radius > 10:
            #     cv2.circle(img_original,(int(x), int(y)), int(radius), (0,255,255),2)
            #     cv2.circle(img_original,center,5,(0,0,255),-1)


#DISPLAY WHAT CAMERA IS PUBLISHING TO opencv2
        cv2.imshow("colorout", res)
        #cv2.imshow("contout", thresh)
        cv2.imshow("original",img_original)

        cv2.waitKey(20) #Updates with next cached img every 0.01sec

    except CvBridgeError, e:
        print("==[CAMERA MANAGER]==", e)

def listener():
    rospy.init_node('listener',anonymous=True)
    #Initializes node
    rospy.Subscriber("/cameras/right_hand_camera/image",Image,imagecb,queue_size=1)
    #Def node as subscriber with rostopic
    rospy.spin()
    #Loops python until node stopped

if __name__ == '__main__':
    listener()



