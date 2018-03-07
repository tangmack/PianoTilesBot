# omxplayer video.h264

import picamera

with picamera.PiCamera() as camera:
    camera.resolution = (240,320)
    camera.start_recording('my_video.h264')
    camera.wait_recording(50)
    camera.stop_recording()
