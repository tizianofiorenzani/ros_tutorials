# Standard imports
import cv2
import numpy as np;
 
# Read image
im  = cv2.imread("blob.jpg")

# Blur image to remove noise
im_g=cv2.GaussianBlur(im, (15, 15), 0)
hsv = cv2.cvtColor(im_g, cv2.COLOR_BGR2HSV)

#cv2.imshow("Gaussian", im_g)
#cv2.waitKey(0)

blue_min = (77,40,0)
blue_max = (101, 255, 255)

blue    = cv2.inRange(hsv,blue_min, blue_max)

#cv2.imshow("blue threshold", blue)
#cv2.waitKey(0)

# Bitwise-AND of mask and purple only image - only used for display
res = cv2.bitwise_and(im_g, im_g, mask= blue)

#cv2.imshow("blue bitwise", res)
#cv2.waitKey(0)

# dilate makes the in range areas larger
blue = cv2.dilate(blue, None, iterations=2)
blue = cv2.erode(blue, None, iterations=2)

#cv2.imshow("blue dilate", blue)
#cv2.waitKey(0)

if True:
    # Set up the SimpleBlobdetector with default parameters.
    params = cv2.SimpleBlobDetector_Params()
     
    # Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 100;
     
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 30
    params.maxArea = 20000
     
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1
     
    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.5
     
    # Filter by Inertia
    params.filterByInertia =True
    params.minInertiaRatio = 0.5
     
    detector = cv2.SimpleBlobDetector_create(params)

    # Detect blobs.
    reversemask=255-blue
    #cv2.imshow("reversemask", reversemask)
    #cv2.waitKey(0)    
    keypoints = detector.detect(reversemask)

# Draw detected blobs as red circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
 
# Show keypoints
cv2.imshow("Keypoints", im_with_keypoints)
cv2.waitKey(0)
