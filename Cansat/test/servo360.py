import pigpio
from time import sleep

servo_type = 270
sl = 13
sr = 12
pi = pigpio.pi()
pi.set_mode(sl, pigpio.OUTPUT)
pi.set_mode(sr, pigpio.OUTPUT)

rotate = 800
def sa1(a):
  if(a<0):
    pi.set_servo_pulsewidth(sl,1000)
    a-=35
    sleep(-a/rotate)
    pi.set_servo_pulsewidth(sl,1500)

  if(a>0):
    pi.set_servo_pulsewidth(sl,2000)
    sleep(a/rotate)
    pi.set_servo_pulsewidth(sl,1500)

def sa2(a):
  if(a<0):
    pi.set_servo_pulsewidth(sr,1000)
    a-=35
    sleep(-a/rotate)
    pi.set_servo_pulsewidth(sr,1500)

  if(a>0):
    pi.set_servo_pulsewidth(sr,2000)
    sleep(a/rotate)
    pi.set_servo_pulsewidth(sr,1500)


try:
  while True:
    sa1(360)
    sleep(1)
    sa1(-360)
    sleep(1)
except KeyboardInterrupt:
  pl.stop()
  pr.stop()
  GPIO.cleanup()