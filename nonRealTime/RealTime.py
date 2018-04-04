#todo: speed, code should only have if(lane) once in loop

import cv2
import numpy as np
from settings import *

import time

from collections import deque

import threading
import logging

##timeEntry = False
##t0 = time.time()
#initialize time, just to get into main's scope

import RPi.GPIO as GPIO

delay_time = .02

LedPin = 11    # pin11
LedPin2 = 12

LedPin3 = 13
LedPin4 = 15

def setup():
    GPIO.setmode(GPIO.BOARD)       # Numbers GPIOs by physical location
    GPIO.setup(LedPin, GPIO.OUT)   # Set LedPin's mode is output
    GPIO.output(LedPin, GPIO.LOW) # Set LedPin high(+3.3V) to turn on led

    GPIO.setup(LedPin2, GPIO.OUT)   # Set LedPin's mode is output
    GPIO.output(LedPin2, GPIO.LOW) # Set LedPin high(+3.3V) to turn on led

    GPIO.setup(LedPin3, GPIO.OUT)   # Set LedPin's mode is output
    GPIO.output(LedPin3, GPIO.LOW) # Set LedPin high(+3.3V) to turn on led

    GPIO.setup(LedPin4, GPIO.OUT)   # Set LedPin's mode is output
    GPIO.output(LedPin4, GPIO.LOW) # Set LedPin high(+3.3V) to turn on led

def worker(num,Lane):
    time.sleep(3*(0.97 / num))

##    time.sleep(.26)

    if Lane == 1:
##        cv2.circle(frame,(L1,H2), 10, RED, -1)
        print "tap lane 1"
        GPIO.output(LedPin, GPIO.HIGH)  # led on
        time.sleep(delay_time)

        GPIO.output(LedPin, GPIO.LOW)  # led on
        time.sleep(delay_time)

    elif Lane == 2:
##        cv2.circle(frame,(L2,H2), 10, RED, -1)
        print "tap lane 2"
        GPIO.output(LedPin2, GPIO.HIGH)  # led on
        time.sleep(delay_time)

        GPIO.output(LedPin2, GPIO.LOW)  # led on
        time.sleep(delay_time)
    elif Lane == 3:
##        cv2.circle(frame,(L3,H2), 10, RED, -1)
        print "tap lane 3"
        GPIO.output(LedPin3, GPIO.HIGH)  # led on
        time.sleep(delay_time)

        GPIO.output(LedPin3, GPIO.LOW)  # led on
        time.sleep(delay_time)
        
    elif Lane == 4:
##        cv2.circle(frame,(L4,H2), 10, RED, -1)
        print "tap lane 4"
        GPIO.output(LedPin4, GPIO.HIGH)  # led on
        time.sleep(delay_time)

        GPIO.output(LedPin4, GPIO.LOW)  # led on
        time.sleep(delay_time)


def doubleTapWorker(num,Lane,c):
    time.sleep(3*(0.97 / num))

##    print Lane

##    time.sleep(.26)
##    print Lane

    if Lane == 1:
        print "tap tap lane 1"
        GPIO.output(LedPin, GPIO.HIGH)  # led on
        time.sleep(delay_time)

        GPIO.output(LedPin, GPIO.LOW)  # led on
        time.sleep(delay_time)
        c.set_val(0)

    elif Lane == 2:
####        cv2.circle(frame,(L2,H2), 10, RED, -1)
        print "tap tap lane 2"
        GPIO.output(LedPin2, GPIO.HIGH)  # led on
        time.sleep(delay_time)

        GPIO.output(LedPin2, GPIO.LOW)  # led on
        time.sleep(delay_time)
        c.set_val(0)
    elif Lane == 3:
##        cv2.circle(frame,(L3,H2), 10, RED, -1)
        print "tap tap lane 3"
        GPIO.output(LedPin3, GPIO.HIGH)  # led on
        time.sleep(delay_time)

        GPIO.output(LedPin3, GPIO.LOW)  # led on
        time.sleep(delay_time)
        c.set_val(0)
        
    elif Lane == 4:
##        cv2.circle(frame,(L4,H2), 10, RED, -1)
        print "tap tap lane 4"
        GPIO.output(LedPin4, GPIO.HIGH)  # led on
        time.sleep(delay_time)

        GPIO.output(LedPin4, GPIO.LOW)  # led on
        time.sleep(delay_time)
        c.set_val(0)

    
class Counter(object):
    def __init__(self, start=0):
        self.lock = threading.Lock()
        self.value = start #no doubles at start
    def set_val(self,val):
##        logging.debug('Waiting for lock')
        self.lock.acquire()
        try:
##            logging.debug('Acquired lock')
            self.value = val
##            logging.debug('Waiting for worker threads')
##            print self.value
        finally:
            self.lock.release()
            
    def get_val(self):
        self.lock.acquire()
        try:
            return self.value
        finally:
            self.lock.release()
        
class TimeMarker():
    def __init__(self):
        self.last_time = time.time()
        self.t1 = time.time()

        self.LastLane = 0

##        self.circular_queue = \
##        deque(np.repeat(AVG_INIT,AVG_SIZE),maxlen=AVG_SIZE)
        self.circular_queue = deque([3.2,3.2,3.2], maxlen=AVG_SIZE)

        self.currentAVG = 3.2

##        self.doubleTap = np.zeros((5,), dtype=int)

##    def get_currentAVG():
##        return self.currentAVG
##    def set_doubleTap(self,lane,number):
##        self.doubleTap[lane] = number


    def movingAverage(self,values):
        weights = np.repeat(1.0,AVG_SIZE)/AVG_SIZE
        smas = np.convolve(values,weights,'valid')
        return smas[0]
        
    def markTime(self,BlackLane):
    ##    if timeEntryBool == True:
        if(BlackLane != self.LastLane):
            self.t1 = time.time()
            #1.329 inches on calipers

            currentMeasure = .97 / (self.t1 - self.last_time)

            t = threading.Thread\
                (target = worker, args=(self.currentAVG,BlackLane,))
            threads.append(t)
            t.start()

            if abs((self.circular_queue[-1] - currentMeasure)) < 1:
##            self.circular_queue.append( 1.0 / (self.t1 - self.last_time) )
                self.circular_queue.append( currentMeasure )
                self.currentAVG = self.movingAverage(self.circular_queue)
            
            self.LastLane = BlackLane
            self.last_time = time.time()

            return True
        else:
            return False

##        else: # there is a possibility of double/triple block
            
    def checkHeight(self,H):
        if frame[H, L1, 0] > 150:
            pass
##            cv2.circle(frame,(L1,H), 5, AQUA, -1)
        else:
            return 1
##            self.markTime(1)
##            cv2.circle(frame,(L1,H), 5, RED, -1)
          
        if frame[H, L2, 0] > 150:
            pass
##            cv2.circle(frame,(L2,H), 5, AQUA, -1)
        else:
            return 2
##            self.markTime(2)
##            cv2.circle(frame,(L2,H), 5, RED, -1)
##            return 2
          
        if frame[H, L3, 0] > 150:
            pass
##            cv2.circle(frame,(L3,H), 5, AQUA, -1)
        else:
            return 3
##            self.markTime(3)
##            cv2.circle(frame,(L3,H), 5, RED, -1)
##            return 3
          
        if frame[H, L4, 0] > 150:
            pass
##            cv2.circle(frame,(L4,H), 5, AQUA, -1)
        else:
##            self.markTime(4)
##            cv2.circle(frame,(L4,H), 5, RED, -1)
            return 4

##        return 0 # no black lane found!

    def try_tap(self,lane,counterObj):
##        print counterObj.get_val(), lane
##        print lane
##        if(lane == 1):
##            counterObj.set_val(1)
##            a = threading.Thread\
##            (target = doubleTapWorker, args=(self.currentAVG,lane,counterObj,))
##            a.start()
##            return
##            print " lane is 1"
        
        if( counterObj.value == 0):# if not already currently double tapping lane #
##            print lane

##            self.doubleTap[lane] = 1 #might need thread locking
            counterObj.set_val(1)
            a = threading.Thread\
            (target = doubleTapWorker, args=(self.currentAVG,lane,counterObj,))
##            threads.append(a)
            a.start() # note this can still cause more than one threads named "a"
        else:
            pass

        

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

##t1 = threading.Thread(target = worker, args=(self.currentAVG,black_lane_hm,))
##                threads.append(t)

# Create a VideoCapture object and read from input file
# If the input is the camera, pass 0 instead of the video file name
cap = cv2.VideoCapture('my_video.h264')

timer = TimeMarker()

counter1 = Counter()
##counter1.set_val(1)
counter2 = Counter()
counter3 = Counter()
counter4 = Counter()

triplecounter1 = Counter()
triplecounter2 = Counter()
triplecounter3 = Counter()
triplecounter4 = Counter()
 
# Check if camera opened successfully
if (cap.isOpened()== False):
    print("Error opening video stream or file")


# Read until video is completed
while(cap.isOpened()):
  
    ret, frame = cap.read()
    if ret == True:

##        print counter1.get_val()

        black_lane_hm = timer.checkHeight(HighMid)

##        print "black lane hm ",black_lane_hm

##        if(black_lane == 0): #todo: find good way to deal with this
##            print ("error, no black lane found")

        if( timer.markTime( black_lane_hm ) == False ):
            #double or triple possibility, check h3 AND HighMid

##            print "double or triple possibility"

            # move H3 down 10 pixels and check it
            black_lane_h3 = timer.checkHeight(H3 + 10)
            
            if( black_lane_h3 == black_lane_hm ):
##                if(black_lane_h3 == 1): timer.try_tap(black_lane_hm, counter1)
                if(black_lane_h3 == 1): timer.try_tap(black_lane_hm,counter1)
                elif(black_lane_h3 == 2): timer.try_tap(black_lane_hm,counter2)
                elif(black_lane_h3 == 3): timer.try_tap(black_lane_hm,counter3)
                elif(black_lane_h3 == 4): timer.try_tap(black_lane_hm,counter4)
                
##                elif(black_
##                print""

                black_lane_h2 = timer.checkHeight(H2 + 10)

                if( (black_lane_h2 == black_lane_hm) and (black_lane_h2 == black_lane_h3) ):
##                    print "triple detected in lane ",black_lane_h2
                    if(black_lane_h2 == 1): timer.try_tap(black_lane_hm,triplecounter1)
                    elif(black_lane_h2 == 2): timer.try_tap(black_lane_hm,triplecounter2)
                    elif(black_lane_h2 == 3): timer.try_tap(black_lane_hm,triplecounter3)
                    elif(black_lane_h2 == 4): timer.try_tap(black_lane_hm,triplecounter4)##
##            if (black_lane_h2 == black_lane_hm):
##                print "triple detected in lane ",black_lane_h2

        
                
##        timer.markTime( timer.checkHeight(HighMid) )
        # and don't pass in timer anymore
        
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
