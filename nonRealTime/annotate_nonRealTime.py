import cv2
import numpy as np
from settings import *

import time

from collections import deque

##import threading


##def worker(num,Lane):
##    time.sleep(.97 / num)
##
####    time.sleep(.26)
##
##    if Lane == 1:
####        cv2.circle(frame,(L1,H2), 10, RED, -1)
##        print "tap lane 1"
##
##    elif Lane == 2:
####        cv2.circle(frame,(L2,H2), 10, RED, -1)
##        print "tap lane 2"
##    elif Lane == 3:
####        cv2.circle(frame,(L3,H2), 10, RED, -1)
##        print "tap lane 3"
##    elif Lane == 4:
####        cv2.circle(frame,(L4,H2), 10, RED, -1)
##        print "tap lane 4"
##    return
    

class TimeMarker():
    def __init__(self):
        self.last_time = time.time()
        self.t1 = time.time()

        self.LastLane = 0

##        self.circular_queue = \
##        deque(np.repeat(AVG_INIT,AVG_SIZE),maxlen=AVG_SIZE)
        self.circular_queue = deque([3.2,3.2,3.2], maxlen=AVG_SIZE)

        self.currentAVG = 3.0

        self.time_sleep_1 = 10.0 #initialize to a large number so we won't tap
        self.time_sleep_2 = 10.0
        self.time_sleep_3 = 10.0
        self.time_sleep_4 = 10.0

        self.doubleFlag = False

    def movingAverage(self,values):
        weights = np.repeat(1.0,AVG_SIZE)/AVG_SIZE
        smas = np.convolve(values,weights,'valid')
        return smas[0]
        
    def markTime(self,Lane):
    ##    if timeEntryBool == True:
        self.t1 = time.time()
        self.currentMeasure = 0.97 / (self.t1 - self.last_time)
##        print currentMeasure
        if(Lane != self.LastLane):

            ######################### self.t1 reset is important!
##            self.t1 = time.time()
            #1.329 inches on calipers

##            currentMeasure = 0.97 / (self.t1 - self.last_time)

##            t = threading.Thread(target = worker, args=(self.currentAVG,Lane,))
##            threads.append(t)
##            t.start()

            if Lane == 1:
##        cv2.circle(frame,(L1,H2), 10, RED, -1)
##                print "tap lane 1"
                self.time_sleep_1 = time.time() + 2.7 / self.currentAVG

            elif Lane == 2:
##        cv2.circle(frame,(L2,H2), 10, RED, -1)
##                print "tap lane 2"
                self.time_sleep_2 = time.time() + 2.7 / self.currentAVG
            elif Lane == 3:
##        cv2.circle(frame,(L3,H2), 10, RED, -1)
##                print "tap lane 3"
                self.time_sleep_3 = time.time() + 2.7 / self.currentAVG
            elif Lane == 4:
##        cv2.circle(frame,(L4,H2), 10, RED, -1)
##                print "tap lane 4"
                self.time_sleep_4 = time.time() + 2.7 / self.currentAVG

            

            

            if abs((self.circular_queue[-1] - self.currentMeasure)) < 1:
##            self.circular_queue.append( 1.0 / (self.t1 - self.last_time) )
                self.circular_queue.append( self.currentMeasure )
                self.currentAVG = self.movingAverage(self.circular_queue)
##            print self.currentAVG
##                print self.currentAVG
##            print( 1.0 / (self.t1 - self.last_time) )
            
            self.LastLane = Lane
            self.last_time = time.time()

##        elif (currentMeasure < self.currentAVG) and (self.doubleFlag == False):
##            doubleFlag = True
####            print "double tile detected"
##            print time.time()
##            if Lane == 1:
####        cv2.circle(frame,(L1,H2), 10, RED, -1)
####                print "tap lane 1"
##                self.time_sleep_1 = time.time() + .97 / self.currentAVG
##
##            elif Lane == 2:
####        cv2.circle(frame,(L2,H2), 10, RED, -1)
####                print "tap lane 2"
##                self.time_sleep_2 = time.time() + .97 / self.currentAVG
##            elif Lane == 3:
####        cv2.circle(frame,(L3,H2), 10, RED, -1)
####                print "tap lane 3"
##                self.time_sleep_3 = time.time() + .97 / self.currentAVG
##            elif Lane == 4:
####        cv2.circle(frame,(L4,H2), 10, RED, -1)
####                print "tap lane 4"
##                self.time_sleep_4 = time.time() + .97 / self.currentAVG

    def respondWaitTimes(self):
        if time.time() >= self.time_sleep_1:
            cv2.circle(frame,(L1,H1), 10, GREEN, -1)
            self.time_sleep_1 = time.time() + 10.0
        elif time.time() >= self.time_sleep_2:
            cv2.circle(frame,(L2,H1), 10, GREEN, -1)
            self.time_sleep_2 = time.time() + 10.0
        elif time.time() >= self.time_sleep_3:
            cv2.circle(frame,(L3,H1), 10, GREEN, -1)
            self.time_sleep_3 = time.time() + 10.0
        elif time.time() >= self.time_sleep_4:
            cv2.circle(frame,(L4,H1), 10, GREEN, -1)
            self.time_sleep_4 = time.time() + 10.0
            #set to high number so we don't trigger again
            
    
    

def checkHeight(H,TimeObject):
    if frame[H, L1, 0] > 150:
        cv2.circle(frame,(L1,H), 5, AQUA, -1)
    else:
        TimeObject.markTime(1)
        cv2.circle(frame,(L1,H), 5, RED, -1)
        return
        
    if frame[H, L2, 0] > 150:
        cv2.circle(frame,(L2,H), 5, AQUA, -1)
    else:
        TimeObject.markTime(2)
        cv2.circle(frame,(L2,H), 5, RED, -1)
        return
        
    if frame[H, L3, 0] > 150:
        cv2.circle(frame,(L3,H), 5, AQUA, -1)
    else:
        TimeObject.markTime(3)
        cv2.circle(frame,(L3,H), 5, RED, -1)
        return
        
    if frame[H, L4, 0] > 150:
        cv2.circle(frame,(L4,H), 5, AQUA, -1)
    else:
        TimeObject.markTime(4)
        cv2.circle(frame,(L4,H), 5, RED, -1)
        return


PianoCalibration = np.load('PianoCalibration.npz')

HighMid = PianoCalibration['HighMid']
H3 = PianoCalibration['H3']
H2 = PianoCalibration['H2']
H1 = PianoCalibration['H1']
L1 = PianoCalibration['L1']
L2 = PianoCalibration['L2']
L3 = PianoCalibration['L3']
L4 = PianoCalibration['L4']

threads = []

# Create a VideoCapture object and read from input file
# If the input is the camera, pass 0 instead of the video file name
cap = cv2.VideoCapture('my_video.h264')

fourcc = cv2.cv.CV_FOURCC(*'XVID')
out = cv2.VideoWriter('annotatedOutput.avi',fourcc, 30.0, (240,320))
##video = cv2.VideoWriter('annotatedOut.avi',-1,1,(240,320))

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
        timer.respondWaitTimes()

        cv2.line(frame,(0,H1),(240,H1),VIOLET,1)

        out.write(frame)
##        video.write(frame)
        

        cv2.imshow('Frame',frame)

        # Press Q on keyboard to  exit
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break

    else:
        break
## 


 
# When everything done, release the video capture object
cap.release()

out.release()

 
# Closes all the frames
cv2.destroyAllWindows()

