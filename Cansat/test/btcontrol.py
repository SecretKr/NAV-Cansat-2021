from evdev import InputDevice, categorize, ecodes
import RPi.GPIO as GPIO
from time import sleep
import pigpio

servo_type = 270
sl = 13
sr = 12
pi = pigpio.pi()
pi.set_mode(sl, pigpio.OUTPUT)
pi.set_mode(sr, pigpio.OUTPUT)

def sa(a,b):
  b = servo_type - b
  pi.set_servo_pulsewidth(sl,500+2000*int(a)/servo_type)
  pi.set_servo_pulsewidth(sr,500+2000*int(b)/servo_type)
def sa1(a):
  pi.set_servo_pulsewidth(sl,500+2000*int(a)/servo_type)
  pi.set_servo_pulsewidth(sr,500+2000*int(a)/servo_type)

gamepad = InputDevice('/dev/input/event0')

print(gamepad)

try:
    for event in gamepad.read_loop():
        if event.type == ecodes.EV_KEY:
            print(event.code,event.value)
        elif event.type == ecodes.EV_ABS:
            absevent = categorize(event)
            t = ecodes.bytype[absevent.event.type][absevent.event.code]
            v = absevent.event.value
            print(t,v)
            center = 1
            if t == "ABS_HAT0Y":
                if v == -1:
                    sa(270,0)
                    center = 0
                if v == 1:
                    sa(0,270)
                    center = 0
            if t == "ABS_HAT0X":
                if v == -1:
                    sa(270,270)
                    center = 0
                if v == 1:
                    sa(0,0)
                    center = 0
            if center: sa1(135) 
except KeyboardInterrupt:
    pi.stop()
