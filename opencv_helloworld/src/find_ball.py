#!/usr/bin/env python

#- ON THE RASPI: roslaunch raspicam_node camerav2_320x240.launch enable_raw:=true
#

#--- Allow relative importing
if __name__ == '__main__' and __package__ is None:
    from os import sys, path
    sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
    
import sys
import rospy
import cv2
import time

from std_msgs.msg           import String
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError
from include.blob_detector  import *


class BlobDetector:

    def __init__(self, thr_min, thr_max, blur=15, blob_params=None, detection_window=None):
    
        self.set_threshold(thr_min, thr_max)
        self.set_blur(blur)
        self.set_blob_params(blob_params)
        self.detection_window = detection_window
        
        self._t0 = time.time()
        
        self.blob_point = Point()
    
        print (">> Publishing image to topic image_blob")
        self.image_pub = rospy.Publisher("image_blob",Image,queue_size=1)
        self.mask_pub = rospy.Publisher("image_mask",Image,queue_size=1)
        print (">> Publishing position to topic point_blob")
        self.blob_pub  = rospy.Publisher("point_blob",Point,queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/raspicam_node/image",Image,self.callback)
        print ("<< Subscribed to topic /raspicam_node/image")
        
    def set_threshold(self, thr_min, thr_max):
        self._threshold = [thr_min, thr_max]
        
    def set_blur(self, blur):
        self._blur = blur
      
    def set_blob_params(self, blob_params):
        self._blob_params = blob_params
        
    def get_blob_relative_position(self, cv_image, keyPoint):
        (rows,cols,channels) = cv_image.shape
        cols = float(cols)
        rows = float(rows)
        # print(rows, cols)
        center_x    = 0.5*cols
        center_y    = 0.5*rows
        # print(center_x)
        x = (keyPoint.pt[0] - center_x)/(center_x)
        y = (keyPoint.pt[1] - center_y)/(center_y)
        return(x,y)
        
    def callback(self,data):
        #--- Assuming image is 320x240
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
        
            keypoints, mask   = blob_detect(cv_image, self._threshold[0], self._threshold[1], self._blur, blob_params=self._blob_params)
            cv_image    = draw_keypoints(cv_image, keypoints) 

            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
                self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, "8UC1"))
            except CvBridgeError as e:
                print(e)            

            for i, keyPoint in enumerate(keypoints):
                # x = keyPoint.pt[0]
                # y = keyPoint.pt[1]
                s = keyPoint.size
                # print ("kp %d: s = %3d   x = %3d  y= %3d"%(i, s, x, y))
                _x, _y = self.get_blob_relative_position(cv_image, keyPoint)
                print ("kp %d: s = %3d   x = %.2f  y= %.2f"%(i, s, _x, _y))

                if not self.detection_window is None:
                    if (self.detection_window[0] <= _x <= self.detection_window[1]) and (self.detection_window[2] <= _y <= self.detection_window[3]):
                        x, y = _x, _y
                        print ("window kp %d: s = %3d   x = %.2f  y= %.2f"%(i, s, x, y))
                        self.blob_point.x = x
                        self.blob_point.y = y
                        self.blob_pub.publish(self.blob_point)                        
                else:
                    x, y = _x, _y
                    # print ("kp %d: s = %3d   x = %.2f  y= %.2f"%(i, s, x, y))
                    
            fps = 1.0/(time.time()-self._t0)
            self._t0 = time.time()
            
def main(args):
    blue_min = (77,40,0)
    blue_max = (101, 255, 255) 
    # blue_min = (82,31,62)
    # blue_max = (106, 116, 193)     
    blue_min = (55,40,0)
    blue_max = (150, 255, 255)     
    
    blur     = 5
    min_size = 10
    max_size = 40
    
    #--- detection window respect to camera frame in [-1, 1] (x positive right, y positive down)
    x_min   = -0.8
    x_max   =  0.8
    y_min   = -0.1
    y_max   = 0.9
    
    detection_window = [x_min, x_max, y_min, y_max]
    
    params = cv2.SimpleBlobDetector_Params()
         
    # Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 100;
     
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 10
    params.maxArea = 20000
     
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1
     
    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.2
     
    # Filter by Inertia
    params.filterByInertia =True
    params.minInertiaRatio = 0.4    

    ic = BlobDetector(blue_min, blue_max, blur, params, detection_window)
    rospy.init_node('blob_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
