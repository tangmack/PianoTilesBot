# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

import numpy as np

from settings import *

def nothing(x):
    pass

PongCalibration = np.load('PianoCalibration.npz')

LastHighMid = PongCalibration['HighMid']
LastL1 = PongCalibration['L1']
LastL2 = PongCalibration['L2']
LastL3 = PongCalibration['L3']
LastL4 = PongCalibration['L4']

cv2.namedWindow('Calibration')
cv2.createTrackbar('HighMid', 'Calibration',LastHighMid,HEIGHT,nothing)
cv2.createTrackbar('L1', 'Calibration',LastL1,WIDTH,nothing)
cv2.createTrackbar('L2', 'Calibration',LastL2,WIDTH,nothing)
cv2.createTrackbar('L3', 'Calibration',LastL3,WIDTH,nothing)
cv2.createTrackbar('L4', 'Calibration',LastL4,WIDTH,nothing)

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (WIDTH, HEIGHT)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(WIDTH, HEIGHT))

# allow the camera to warmup
time.sleep(0.1)

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    HighMid = cv2.getTrackbarPos('HighMid','Calibration')
    L1 = cv2.getTrackbarPos('L1','Calibration')
    L2 = cv2.getTrackbarPos('L2','Calibration')
    L3 = cv2.getTrackbarPos('L3','Calibration')
    L4 = cv2.getTrackbarPos('L4','Calibration')

    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array

    cv2.line(image,(0,HighMid),(240,HighMid),WHITE,1)
    cv2.line(image,(L1,0),(L1,HEIGHT),RED,1)
    cv2.line(image,(L2,0),(L2,HEIGHT),GREEN,1)
    cv2.line(image,(L3,0),(L3,HEIGHT),BLUE,1)
    cv2.line(image,(L4,0),(L4,HEIGHT),PINK,1)



    # show the frame
    cv2.imshow("Frame", image)
    key = cv2.waitKey(1) & 0xFF

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    if key == ord("s"):
        np.savez('PianoCalibration.npz', HighMid=HighMid, \
            L1=L1,L2=L2,L3=L3,L4=L4)

    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
	break

