import cv2
import numpy as np
from settings import *

import time

##timeEntry = False
##t0 = time.time()
#initialize time, just to get into main's scope

class TimeMarker():
    def __init__(self):
        self.last_time = time.time()
        self.t1 = time.time()

        self.LastLane = 0
        
    def markTime(self,Lane):
    ##    if timeEntryBool == True:
        if(Lane != self.LastLane):
            self.t1 = time.time()
            #1.329
##            print "self.t1 ",self.t1, ( 1.0 / (self.t1 - self.last_time) )
##            print self.LastLane
##            print(self.t1 - self.last_time)
            print( 1.0 / (self.t1 - self.last_time) )
            self.LastLane = Lane
            self.last_time = time.time()
##        else:
##            t0 = time.time()
##            timeEntryBool = True
    
    

def checkHeight(H,TimeObject):
    if frame[H, L1, 0] > 150:
        cv2.circle(frame,(L1,H), 5, AQUA, -1)
    else:
        TimeObject.markTime(1)
        cv2.circle(frame,(L1,H), 5, RED, -1)
        
    if frame[H, L2, 0] > 150:
        cv2.circle(frame,(L2,H), 5, AQUA, -1)
    else:
        TimeObject.markTime(2)
        cv2.circle(frame,(L2,H), 5, RED, -1)
        
    if frame[H, L3, 0] > 150:
        cv2.circle(frame,(L3,H), 5, AQUA, -1)
    else:
        TimeObject.markTime(3)
        cv2.circle(frame,(L3,H), 5, RED, -1)
        
    if frame[H, L4, 0] > 150:
        cv2.circle(frame,(L4,H), 5, AQUA, -1)
    else:
        TimeObject.markTime(4)
        cv2.circle(frame,(L4,H), 5, RED, -1)


PianoCalibration = np.load('PianoCalibration.npz')

HighMid = PianoCalibration['HighMid']
H3 = PianoCalibration['H3']
H2 = PianoCalibration['H2']
H1 = PianoCalibration['H1']
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

timer = TimeMarker()
 
# Check if camera opened successfully
if (cap.isOpened()== False):
    print("Error opening video stream or file")

##tL1 = time.time()
##tL2 = time.time()
##tL3 = time.time()
##tL4 = time.time()
# Read until video is completed
while(cap.isOpened()):
  

    ret, frame = cap.read()
    if ret == True:


        checkHeight(HighMid,timer)
##        checkHeight(H3,TimeObject)
        

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
