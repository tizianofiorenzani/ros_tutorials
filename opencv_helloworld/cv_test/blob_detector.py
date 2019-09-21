# Standard imports
import cv2
import numpy as np;

def blob_detect(image, hsv_min, hsv_max, gauss_kernel, blob_params=None):

    # Blur image to remove noise
    im_g    = cv2.GaussianBlur(image, (gauss_kernel, gauss_kernel), 0)
    hsv     = cv2.cvtColor(im_g, cv2.COLOR_BGR2HSV)
    mask    = cv2.inRange(hsv,hsv_min, hsv_max)

    # dilate makes the in range areas larger
    mask = cv2.dilate(mask, None, iterations=2)
    mask = cv2.erode(mask, None, iterations=2)

    if blob_params is None:
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
         
    else:
        params = blob_params     

    detector = cv2.SimpleBlobDetector_create(params)

    # Detect blobs.
    reversemask = 255-mask
    keypoints = detector.detect(reversemask)

    return keypoints

def draw_keypoints(image, keypoints, line_color=(0,0,255)):
    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), line_color, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
 
    # Show keypoints
    cv2.imshow("Keypoints", im_with_keypoints)
    cv2.waitKey(0)

#----- TEST
if __name__=="__main__":
    # Read image
    image_list = []
    image_list.append(cv2.imread("blob.jpg"))
    image_list.append(cv2.imread("blob2.jpg"))
    image_list.append(cv2.imread("blob3.jpg"))
    blue_min = (77,40,0)
    blue_max = (101, 255, 255) 
    
    for image in image_list:
        keypoints = blob_detect(image, blue_min, blue_max, 15, blob_params=None)
        draw_keypoints(image, keypoints)
