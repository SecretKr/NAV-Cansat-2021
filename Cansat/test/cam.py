from picamera import PiCamera
from time import sleep

camera = PiCamera()

camera.resolution = (1920, 1080)
camera.framerate = 15
camera.start_recording('video.h264')
sleep(10)
camera.stop_recording()