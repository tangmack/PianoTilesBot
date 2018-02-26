import cv2
import numpy as np
from settings import *

import time


PianoCalibration = np.load('PianoCalibration.npz')

HighMid = PianoCalibration['HighMid']
L1 = PianoCalibration['L1']
L2 = PianoCalibration['L2']
L3 = PianoCalibration['L3']
L4 = PianoCalibration['L4']

print L1
print L2
print L3
print L4

print HighMid


# Create a VideoCapture object and read from input file
# If the input is the camera, pass 0 instead of the video file name
cap = cv2.VideoCapture('my_video.h264')
 
# Check if camera opened successfully
if (cap.isOpened()== False):
    print("Error opening video stream or file")
 
# Read until video is completed
while(cap.isOpened()):
  

    ret, frame = cap.read()
    if ret == True:


        if frame[HighMid, L1, 0] > 150:
            cv2.circle(frame,(L1,HighMid), 5, AQUA, -1)
        else:
            cv2.circle(frame,(L1,HighMid), 5, VIOLET, -1)

        if frame[HighMid, L2, 0] > 150:
            cv2.circle(frame,(L2,HighMid), 5, AQUA, -1)
        else:
            cv2.circle(frame,(L2,HighMid), 5, VIOLET, -1)

        if frame[HighMid, L3, 0] > 150:
            cv2.circle(frame,(L3,HighMid), 5, AQUA, -1)
        else:
            cv2.circle(frame,(L3,HighMid), 5, VIOLET, -1)

        if frame[HighMid, L4, 0] > 150:
            cv2.circle(frame,(L4,HighMid), 5, AQUA, -1)
        else:
            cv2.circle(frame,(L4,HighMid), 5, VIOLET, -1)
        

        cv2.imshow('Frame',frame)
 
        # Press Q on keyboard to  exit
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break

    else:
        break
 


 
# When everything done, release the video capture object
cap.release()
 
# Closes all the frames
cv2.destroyAllWindows()
