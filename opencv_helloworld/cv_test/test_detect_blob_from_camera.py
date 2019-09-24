import cv2
from blob_detector import *

blue_min = (77,40,0)
blue_max = (101, 255, 255) 

cap = cv2.VideoCapture(0)

# cap = cv2.VideoCapture('vtest.avi')

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    keypoints = blob_detect(frame, blue_min, blue_max, 15, blob_params=None)
    draw_keypoints(frame, keypoints)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
    

