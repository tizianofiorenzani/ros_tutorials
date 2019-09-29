#!/usr/bin/env python

# SIMPLE SCRIPT TO TEST CV_BRIDGE PACKAGE
#
#- ON THE RASPI: roslaunch raspicam_node camerav2_1280x960.launch enable_raw:=true
#

import sys

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#--- Define our Class
class image_converter:

    def __init__(self):
        #--- Publisher of the edited frame
        self.image_pub = rospy.Publisher("image_topic",Image,queue_size=1)

        #--- Subscriber to the camera flow
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/raspicam_node/image",Image,self.callback)

    def callback(self,data):  #--- Callback function
    
        #--- Read the frame and convert it using bridge
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        #--- If a valid frame is received, draw a circle and write HELLO WORLD
        (rows,cols,channels) = cv_image.shape
        if cols > 20 and rows > 20:
            #--- Circle
            cv2.circle(cv_image, (500,500), 200, 255)
            
            #--- Text
            text = "HELLO WORLD"
            cv2.putText(cv_image, text, (100, 150), cv2.FONT_HERSHEY_SIMPLEX, 5, [0,0,200], 5)

        #--- Optional: show the image on a window (comment this for the Raspberry Pi)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        #--- Publish the modified frame to a new topic
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

#--------------- MAIN LOOP
def main(args):
    #--- Create the object from the class we defined before
    ic = image_converter()
    
    #--- Initialize the ROS node
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        
    #--- In the end remember to close all cv windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
        main(sys.argv)
