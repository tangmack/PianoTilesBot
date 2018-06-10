from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np

from settings import *

import time
from collections import deque
import threading
import logging
import RPi.GPIO as GPIO
import sys

delay_time_down = .04
delay_time_release = .01

LedPin = 11    # pin11
LedPin2 = 12

LedPin3 = 15
LedPin4 = 13

def setup():
    GPIO.setwarnings(False)
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

    time.sleep(2.75*num)

    if Lane == 1:
        GPIO.output(LedPin, GPIO.HIGH)  
        time.sleep(delay_time_down)

        GPIO.output(LedPin, GPIO.LOW)  
        time.sleep(delay_time_release)

    elif Lane == 2:
        GPIO.output(LedPin2, GPIO.HIGH)  
        time.sleep(delay_time_down)

        GPIO.output(LedPin2, GPIO.LOW)  
        time.sleep(delay_time_release)
    elif Lane == 3:
        GPIO.output(LedPin3, GPIO.HIGH)  
        time.sleep(delay_time_down)

        GPIO.output(LedPin3, GPIO.LOW)  
        time.sleep(delay_time_release)
        
    elif Lane == 4:
        GPIO.output(LedPin4, GPIO.HIGH)  
        time.sleep(delay_time_down)

        GPIO.output(LedPin4, GPIO.LOW)  
        time.sleep(delay_time_release)

def worker_simple(num,Lane):
    time.sleep(num)#do not multiply by lower time factor

    if Lane == 1:
        print "tap lane 1"
        GPIO.output(LedPin, GPIO.HIGH)  
        time.sleep(delay_time_down)

        GPIO.output(LedPin, GPIO.LOW)  
        time.sleep(delay_time_release)

    elif Lane == 2:
        print "tap lane 2"
        GPIO.output(LedPin2, GPIO.HIGH)  
        time.sleep(delay_time_down)

        GPIO.output(LedPin2, GPIO.LOW)  
        time.sleep(delay_time_release)
    elif Lane == 3:
        print "tap lane 3"
        GPIO.output(LedPin3, GPIO.HIGH)  
        time.sleep(delay_time_down)

        GPIO.output(LedPin3, GPIO.LOW)  
        time.sleep(delay_time_release)
        
    elif Lane == 4:
        print "tap lane 4"
        GPIO.output(LedPin4, GPIO.HIGH)  
        time.sleep(delay_time_down)

        GPIO.output(LedPin4, GPIO.LOW)  
        time.sleep(delay_time_release)


def doubleTapWorker(num,Lane,c):
    time.sleep(2.75* num)

    if Lane == 1:
        GPIO.output(LedPin, GPIO.HIGH)  
        time.sleep(delay_time_down)

        GPIO.output(LedPin, GPIO.LOW)  
        time.sleep(delay_time_release)
        c.set_val(0)

    elif Lane == 2:
        GPIO.output(LedPin2, GPIO.HIGH)  
        time.sleep(delay_time_down)

        GPIO.output(LedPin2, GPIO.LOW)  
        time.sleep(delay_time_release)
        c.set_val(0)
    elif Lane == 3:
        GPIO.output(LedPin3, GPIO.HIGH)  
        time.sleep(delay_time_down)

        GPIO.output(LedPin3, GPIO.LOW)  
        time.sleep(delay_time_release)
        c.set_val(0)
        
    elif Lane == 4:
        GPIO.output(LedPin4, GPIO.HIGH)  
        time.sleep(delay_time_down)

        GPIO.output(LedPin4, GPIO.LOW)  
        time.sleep(delay_time_release)
        c.set_val(0)
    
class Counter(object):
    def __init__(self, start=0):
        self.lock = threading.Lock()
        self.value = start #no doubles at start
    def set_val(self,val):
        self.lock.acquire()
        try:
            self.value = val
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

        self.circular_queue = deque([0.3,0.3,0.3,0.3,0.3,0.3,0.3], maxlen=AVG_SIZE)

        self.currentAVG = 0.3

        # need to put a "fake" .01 case at the end to prevent index out of bounds
        self.tier_arr = np.array([.17,.13,.11,.08,0])
        self.Lower_arr = np.array([0.80,.70,.65,.60,.60])
        self.tier_i = 0
        
        self.my_lower_time = 0.85

        self.sec_per_sec = .004 # tile acceleration factor in seconds per second

        self.time_of_start = time.time()

        self.i_stage = 0
        
        #border time between stage (index) and stage (index+1)
        self.time_array = [46,80,600]

        self.A = 0.29 # stage 0
        self.A_array = [0.33,0.33] # [stage 1,stage 2,...]

    def movingAverage(self,values):
        weights = np.repeat(1.0,AVG_SIZE)/AVG_SIZE
        smas = np.convolve(values,weights,'valid')
        return smas[0]
        
    def markTime(self,BlackLane):
        if(BlackLane != self.LastLane):
            self.t1 = time.time()
            currentMeasure = (self.t1 - self.last_time)
        
            #lower time factor, only do this once.
            if (self.currentAVG <= self.tier_arr[self.tier_i]):
                print "lowering time factor to ",self.tier_i
                self.my_lower_time = self.Lower_arr[self.tier_i]
                self.tier_i += 1


##            t = threading.Thread\
##                (target = worker, args=(self.currentAVG*self.my_lower_time,BlackLane,))
##            t = threading.Thread\
##                (target = worker, args=(0.3,BlackLane,))
            time_elap = time.time() - self.time_of_start

            if time_elap > self.time_array[self.i_stage]:
                self.A = self.A_array[self.i_stage]
                self.i_stage += 1
                print "now in stage ",self.i_stage, "at time ",time_elap

            #TODO possibly: make time_elap used in code below
            t = threading.Thread\
                (target = worker, args=(self.A - (time.time() - self.time_of_start) * self.sec_per_sec,BlackLane,))
            threads.append(t)
            t.start()
            

                

##            if not (self.in_stage2):
##                if (time_elap > self.stage_two):
##                    print "###########################STAGE TWO"
##                    self.in_stage2 = True
##            
##            if(time_elap > self.stage_two):
##                t = threading.Thread\
##                    (target = worker, args=(0.33 - (time.time() - self.time_of_start) * self.sec_per_sec2,BlackLane,))
##                threads.append(t)
##                t.start()
##            else:
##                
##                t = threading.Thread\
##                    (target = worker, args=(0.29 - (time.time() - self.time_of_start) * self.sec_per_sec,BlackLane,))
##                threads.append(t)
##                t.start()

            if abs((self.circular_queue[-1] - currentMeasure)) < .08 and currentMeasure < self.circular_queue[-1]:
                print currentMeasure
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
        else:
            return 1
          
        if frame[H, L2, 0] > 150:
            pass
        else:
            return 2
          
        if frame[H, L3, 0] > 150:
            pass
        else:
            return 3
          
        if frame[H, L4, 0] > 150:
            pass
        else:
            return 4

##        return 0 # no black lane found!

    def try_tap(self,lane,counterObj):
        
        if( counterObj.value == 0):# if not already currently double tapping lane #
            counterObj.set_val(1)
##            a = threading.Thread\
##            (target = doubleTapWorker, args=(self.currentAVG*self.my_lower_time,lane,counterObj,))
##            a = threading.Thread\
##            (target = doubleTapWorker, args=(0.3,lane,counterObj,))
            time_elapsed = time.time() - self.time_of_start
            
            if time_elapsed > self.time_array[self.i_stage]:
                self.A = self.A_array[self.i_stage]
                self.i_stage += 1
                print "double or triple triggered now in stage ",self.i_stage,"at time ",time_elapsed
                
            a = threading.Thread\
            (target = doubleTapWorker, args=(self.A - (time_elapsed) * self.sec_per_sec,lane,counterObj,))
            a.start()

        else:
            pass

    def simple_tap(self,BlackLane,number): 
        t = threading.Thread\
                (target = worker_simple, args=(number,BlackLane,))
        threads.append(t)
        t.start()

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
 
###################### camera setup
camera = PiCamera()
camera.resolution = (WIDTH, HEIGHT)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(WIDTH, HEIGHT))

# allow the camera to warmup
time.sleep(0.1)
#####################

for img in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    frame = img.array

    black_lane_h2 = timer.checkHeight(H2) #int (1,2,3 or 4) storing black lane number
    black_lane_h3 = timer.checkHeight(H3)
    
    if(black_lane_h2 == 1): timer.simple_tap(black_lane_h2,0)
    elif(black_lane_h2 == 2): timer.simple_tap(black_lane_h2,0)
    elif(black_lane_h2 == 3): timer.simple_tap(black_lane_h2,0)
    elif(black_lane_h2 == 4): timer.simple_tap(black_lane_h2,0)

    if(black_lane_h3 == 1): timer.simple_tap(black_lane_h3,.33) #tried .303, then bumped up to .33
    elif(black_lane_h3 == 2): timer.simple_tap(black_lane_h3,.33)
    elif(black_lane_h3 == 3): timer.simple_tap(black_lane_h3,.33)
    elif(black_lane_h3 == 4): timer.simple_tap(black_lane_h3,.33)

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

    break #only do this loop once


for img in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
##    print "currentAVG is :",timer.currentAVG
    frame = img.array

    black_lane_hm = timer.checkHeight(HighMid) #int (1,2,3 or 4) storing black lane number

    if( timer.markTime( black_lane_hm ) == False ):
    #double or triple possibility, check h3 AND HighMid

        # move H3 down 10 pixels and check it
        black_lane_h3 = timer.checkHeight(H3 + 6)
                
        if( black_lane_h3 == black_lane_hm ):
##            print "at least double detected"
            if(black_lane_h3 == 1): timer.try_tap(black_lane_hm,counter1)
            elif(black_lane_h3 == 2): timer.try_tap(black_lane_hm,counter2)
            elif(black_lane_h3 == 3): timer.try_tap(black_lane_hm,counter3)
            elif(black_lane_h3 == 4): timer.try_tap(black_lane_hm,counter4)

            black_lane_h2 = timer.checkHeight(H2 + 6)

            if( (black_lane_h2 == black_lane_hm) and (black_lane_h2 == black_lane_h3) ):
##                print "triple in lane ",black_lane_h2
                if(black_lane_h2 == 1): timer.try_tap(black_lane_hm,triplecounter1)
                elif(black_lane_h2 == 2): timer.try_tap(black_lane_hm,triplecounter2)
                elif(black_lane_h2 == 3): timer.try_tap(black_lane_hm,triplecounter3)
                elif(black_lane_h2 == 4): timer.try_tap(black_lane_hm,triplecounter4)##

    rawCapture.truncate(0)

            # Press Q on keyboard to  exit
    if cv2.waitKey(25) & 0xFF == ord('q'):
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
