"""
Library for detecting a blob based on a color range filter in HSV space

   0------------------> x (cols)
   |
   |
   |         o center
   |
   |
   V y (rows)


"""


# Standard imports
import cv2
import numpy as np;

#---------- Blob detecting function: returns keypoints and mask
def blob_detect(image,                  #-- The frame (cv standard)
                hsv_min,                #-- minimum threshold of the hsv filter [h_min, s_min, v_min]
                hsv_max,                #-- maximum threshold of the hsv filter [h_max, s_max, v_max]
                gauss_kernel=0,         #-- Kernel size of the gaussian filter (default 0)
                blob_params=None,       #-- blob parameters (default None)
                search_window=None,     #-- window where to search as [x_min, y_min, x_max, y_max] adimensional (0.0 to 1.0) starting from top left corner
                imshow=False
               ):

    #- Blur image to remove noise
    if gauss_kernel > 0: 
        image    = cv2.GaussianBlur(image, (gauss_kernel, gauss_kernel), 0)
        #- Show result
        if imshow:
            cv2.imshow("Gaussian Blur", mask)
        
    #- Search window
    if search_window is None: search_window = [0.0, 0.0, 1.0, 1.0]
    
    #- Convert image from BGR to HSV
    hsv     = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    #- Apply HSV threshold
    mask    = cv2.inRange(hsv,hsv_min, hsv_max)
    
    #- Show HSV Mask
    if imshow:
        cv2.imshow("HSV Mask", mask)
    
    #- dilate makes the in range areas larger
    mask = cv2.dilate(mask, None, iterations=2)
    mask = cv2.erode(mask, None, iterations=2)
    
    #- Show dilate/erode mask
    if imshow:
        cv2.imshow("Dilate/Erode Mask", mask)
    
    #- Cut the image using the search mask
    mask = cut_image(mask, search_window)
    
    if imshow:
        cv2.imshow("Cropping Mask", mask)

    #- build default blob detection parameters, if none have been provided
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

    #- Apply blob detection
    detector = cv2.SimpleBlobDetector_create(params)

    # Reverse the mask: blobs are black on white
    reversemask = 255-mask
    
    if imshow:
        cv2.imshow("Reverse Mask", reversemask)
        
    keypoints = detector.detect(reversemask)

    return keypoints, reversemask

#---------- Draw detected blobs: returns the image
def draw_keypoints(image,                   #-- Input image
                   keypoints,               #-- CV keypoints
                   line_color=(0,0,255),    #-- line's color (b,g,r)
                   imshow=False             #-- show the result
                  ):
    
    #-- Draw detected blobs as red circles.
    #-- cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), line_color, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
 
    if imshow:
        # Show keypoints
        cv2.imshow("Keypoints", im_with_keypoints)
        
    return(im_with_keypoints)

#---------- Draw search window: returns the image
def draw_window(image,              #- Input image
                window_adim,        #- window in adimensional units
                color=(255,0,0),    #- line's color
                line=5,             #- line's thickness
                imshow=False        #- show the image
               ):
    
    rows = image.shape[0]
    cols = image.shape[1]
    
    x_min_px    = int(cols*window_adim[0])
    y_min_px    = int(rows*window_adim[1])
    x_max_px    = int(cols*window_adim[2])
    y_max_px    = int(rows*window_adim[3])  
    
    #-- Draw a rectangle from top left to bottom right corner
    image = cv2.rectangle(image,(x_min_px,y_min_px),(x_max_px,y_max_px),color,line)
    
    if imshow:
        # Show keypoints
        cv2.imshow("Keypoints", image)

    return(image)

#---------- Apply search window: returns the image
def cut_image(image, window_adim=[0.0, 0.0, 1.0, 1.0]):
    rows = image.shape[0]
    cols = image.shape[1]
    x_min_px    = int(cols*window_adim[0])
    y_min_px    = int(rows*window_adim[1])
    x_max_px    = int(cols*window_adim[2])
    y_max_px    = int(rows*window_adim[3])    
    
    #--- Initialize the mask as a black image
    mask = np.zeros(image.shape,np.uint8)
    
    #--- Copy the pixels from the original image corresponding to the window
    mask[y_min_px:y_max_px,x_min_px:x_max_px] = image[y_min_px:y_max_px,x_min_px:x_max_px]   
    
    #--- return the mask
    return(mask)
    

#----------- TEST
if __name__=="__main__":

    #--- Define HSV limits
    blue_min = (77,40,0)
    blue_max = (101, 255, 255) 
    
    #--- Define area limit [x_min, y_min, x_max, y_max] adimensional (0.0 to 1.0) starting from top left corner
    window = [0.25, 0.25, 0.65, 0.75]
    
    #-- IMAGE_SOURCE: either 'camera' or 'imagelist'
    SOURCE = 'video'
    #SOURCE = 'camera'
    
    if SOURCE == 'video':
        cap = cv2.VideoCapture(0)
        while(True):
            # Capture frame-by-frame
            ret, frame = cap.read()
            
            #-- Detect keypoints
            keypoints, _ = blob_detect(frame, blue_min, blue_max, gauss_kernel=3, 
                                        blob_params=None, search_window=window, imshow=False)
            #-- Draw search window
            frame     = draw_window(frame, window)

            #-- click ENTER on the image window to proceed
            draw_keypoints(frame, keypoints, imshow=True)

            #-- press q to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
    else:
        #-- Read image list from file:
        image_list = []
        image_list.append(cv2.imread("blob.jpg"))
        image_list.append(cv2.imread("blob2.jpg"))
        image_list.append(cv2.imread("blob3.jpg"))

        for image in image_list:
            #-- Detect keypoints
            keypoints, _ = blob_detect(image, blue_min, blue_max, gauss_kernel=5, 
                                        blob_params=None, search_window=window, imshow=False)
            #-- Draw search window
            image     = draw_window(image, window, imshow=True)

            #-- click ENTER on the image window to proceed
            draw_keypoints(image, keypoints, imshow=True)
            
            #-- enter to proceed, q to quit
            if cv2.waitKey(0) & 0xFF == ord('q'):
                break
            
        
    
    
