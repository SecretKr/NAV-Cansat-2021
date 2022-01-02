from picamera import PiCamera
from PIL import Image
from time import sleep
import pigpio

servo_type = 270
sl = 13
sr = 12
pi = pigpio.pi()
pi.set_mode(sl, pigpio.OUTPUT)
pi.set_mode(sr, pigpio.OUTPUT)

def sa(a,b):
  pi.set_servo_pulsewidth(sl,500+2000*int(a)/servo_type)
  pi.set_servo_pulsewidth(sr,500+2000*int(b)/servo_type)
def sa1(a):
  pi.set_servo_pulsewidth(sl,500+2000*int(a)/servo_type)
  pi.set_servo_pulsewidth(sr,500+2000*int(a)/servo_type)

camera = PiCamera()
camera.resolution = (1280, 720)
camera.framerate = 60

while True:
    camera.capture('tem.jpg', use_video_port=True)
    img = Image.open('tem.jpg')
    color = img.getpixel((320,200))
    lg = color[1] > color[0] and color[1] > color[2]
    color = img.getpixel((960,200))
    rg = color[1] > color[0] and color[1] > color[2]
    if lg > rg:
        sa(0,270)
    elif lg < rg:
        sa(270,0)
    else:
        sa1(135)
    print(lg, rg)
    sleep(1)