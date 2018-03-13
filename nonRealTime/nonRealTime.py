import cv2
import numpy as np
from settings import *

import time

from collections import deque

import threading

##timeEntry = False
##t0 = time.time()
#initialize time, just to get into main's scope

import RPi.GPIO as GPIO

LedPin = 11    # pin11
LedPin2 = 12

LedPin3 = 13
LedPin4 = 15

def setup():
  GPIO.setmode(GPIO.BOARD)       # Numbers GPIOs by physical location
  GPIO.setup(LedPin, GPIO.OUT)   # Set LedPin's mode is output
  GPIO.output(LedPin, GPIO.HIGH) # Set LedPin high(+3.3V) to turn on led

  GPIO.setup(LedPin2, GPIO.OUT)   # Set LedPin's mode is output
  GPIO.output(LedPin2, GPIO.HIGH) # Set LedPin high(+3.3V) to turn on led

  GPIO.setup(LedPin3, GPIO.OUT)   # Set LedPin's mode is output
  GPIO.output(LedPin3, GPIO.HIGH) # Set LedPin high(+3.3V) to turn on led

  GPIO.setup(LedPin4, GPIO.OUT)   # Set LedPin's mode is output
  GPIO.output(LedPin4, GPIO.HIGH) # Set LedPin high(+3.3V) to turn on led

def worker(num,Lane):
    time.sleep(3*(.97 / num))

##    time.sleep(.26)

    if Lane == 1:
##        cv2.circle(frame,(L1,H2), 10, RED, -1)
        print "tap lane 1"
        GPIO.output(LedPin, GPIO.HIGH)  # led on
        time.sleep(0.01)

        GPIO.output(LedPin, GPIO.LOW)  # led on
        time.sleep(0.01)

    elif Lane == 2:
##        cv2.circle(frame,(L2,H2), 10, RED, -1)
        print "tap lane 2"
        GPIO.output(LedPin2, GPIO.HIGH)  # led on
        time.sleep(0.01)

        GPIO.output(LedPin2, GPIO.LOW)  # led on
        time.sleep(0.01)
    elif Lane == 3:
##        cv2.circle(frame,(L3,H2), 10, RED, -1)
        print "tap lane 3"
        GPIO.output(LedPin3, GPIO.HIGH)  # led on
        time.sleep(0.01)

        GPIO.output(LedPin3, GPIO.LOW)  # led on
        time.sleep(0.01)
        
    elif Lane == 4:
##        cv2.circle(frame,(L4,H2), 10, RED, -1)
        print "tap lane 4"
        GPIO.output(LedPin4, GPIO.HIGH)  # led on
        time.sleep(0.01)

        GPIO.output(LedPin4, GPIO.LOW)  # led on
        time.sleep(0.01)
        
    return
    
##    print "Worker: ",num

class TimeMarker():
    def __init__(self):
        self.last_time = time.time()
        self.t1 = time.time()

        self.LastLane = 0

##        self.circular_queue = \
##        deque(np.repeat(AVG_INIT,AVG_SIZE),maxlen=AVG_SIZE)
        self.circular_queue = deque([3.2,3.2,3.2], maxlen=AVG_SIZE)

        self.currentAVG = 0


    def movingAverage(self,values):
        weights = np.repeat(1.0,AVG_SIZE)/AVG_SIZE
        smas = np.convolve(values,weights,'valid')
        return smas[0]
        
    def markTime(self,Lane):
    ##    if timeEntryBool == True:
        if(Lane != self.LastLane):
            self.t1 = time.time()
            #1.329 inches on calipers

            currentMeasure = 0.97 / (self.t1 - self.last_time)

            t = threading.Thread(target = worker, args=(self.currentAVG,Lane,))
            threads.append(t)
            t.start()

            if abs((self.circular_queue[-1] - currentMeasure)) < 1:
##            self.circular_queue.append( 1.0 / (self.t1 - self.last_time) )
                self.circular_queue.append( currentMeasure )
                self.currentAVG = self.movingAverage(self.circular_queue)
            
            self.LastLane = Lane
            self.last_time = time.time()


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

setup() # setup GPIO's

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

        cv2.line(frame,(0,H1),(240,H1),VIOLET,1)
        

        cv2.imshow('Frame',frame)

        # Press Q on keyboard to  exit
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break

    else:
        break
 

GPIO.output(LedPin, GPIO.LOW)   # led off
GPIO.output(LedPin2, GPIO.LOW)   # led off
GPIO.output(LedPin3, GPIO.LOW)   # led off
GPIO.output(LedPin4, GPIO.LOW)   # led off
GPIO.cleanup()                  # Release resource
 
# When everything done, release the video capture object
cap.release()
 
# Closes all the frames
cv2.destroyAllWindows()
