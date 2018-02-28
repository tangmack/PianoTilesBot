##cap = cv2.VideoCapture('chaplin.mp4')

import cv2
import numpy as np
from settings import *

def nothing(x):
    pass


PianoCalibration = np.load('PianoCalibration.npz')

LastHighMid = PianoCalibration['HighMid']
LastH3 = PianoCalibration['H3']
LastH2 = PianoCalibration['H2']
LastH1 = PianoCalibration['H1']
LastL1 = PianoCalibration['L1']
LastL2 = PianoCalibration['L2']
LastL3 = PianoCalibration['L3']
LastL4 = PianoCalibration['L4']

cv2.namedWindow('Calibration')
cv2.createTrackbar('HighMid', 'Calibration',LastHighMid,HEIGHT,nothing)
cv2.createTrackbar('H3', 'Calibration',LastH3, HEIGHT, nothing)
cv2.createTrackbar('H2', 'Calibration',LastH2, HEIGHT, nothing)
cv2.createTrackbar('H1', 'Calibration',LastH1, HEIGHT, nothing)
cv2.createTrackbar('L1', 'Calibration',LastL1,WIDTH,nothing)
cv2.createTrackbar('L2', 'Calibration',LastL2,WIDTH,nothing)
cv2.createTrackbar('L3', 'Calibration',LastL3,WIDTH,nothing)
cv2.createTrackbar('L4', 'Calibration',LastL4,WIDTH,nothing)


# Create a VideoCapture object and read from input file
# If the input is the camera, pass 0 instead of the video file name
cap = cv2.VideoCapture('my_video.h264')
 
# Check if camera opened successfully
if (cap.isOpened()== False):
    print("Error opening video stream or file")

ret, frame = cap.read()
# Read until video is completed
while(1):

    # Capture frame-by-frame
##    ret, frame = cap.read()
    if ret == True:

        HighMid = cv2.getTrackbarPos('HighMid','Calibration')
        H3 = cv2.getTrackbarPos('H3','Calibration')
        H2 = cv2.getTrackbarPos('H2','Calibration')
        H1 = cv2.getTrackbarPos('H1','Calibration')
        L1 = cv2.getTrackbarPos('L1','Calibration')
        L2 = cv2.getTrackbarPos('L2','Calibration')
        L3 = cv2.getTrackbarPos('L3','Calibration')
        L4 = cv2.getTrackbarPos('L4','Calibration')

        cv2.line(frame,(0,HighMid),(240,HighMid),WHITE,1)
        cv2.line(frame,(0,H3),(240,H3),VIOLET,1)
        cv2.line(frame,(0,H2),(240,H2),DGREEN,1)
        cv2.line(frame,(0,H1),(240,H1),AQUA,1)
        cv2.line(frame,(L1,0),(L1,HEIGHT),RED,1)
        cv2.line(frame,(L2,0),(L2,HEIGHT),GREEN,1)
        cv2.line(frame,(L3,0),(L3,HEIGHT),BLUE,1)
        cv2.line(frame,(L4,0),(L4,HEIGHT),PINK,1)
 
        # Display the resulting frame
        cv2.imshow('Frame',frame)
 
        # Press Q on keyboard to  exit
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break

        if cv2.waitKey(25) & 0xFF == ord("s"):
            np.savez('PianoCalibration.npz', HighMid=HighMid, H3=H3,\
            H2=H2,H1=H1,L1=L1,L2=L2,L3=L3,L4=L4)
        
    else:
        break

# When everything done, release the video capture object
cap.release()
 
# Closes all the frames
cv2.destroyAllWindows()
