import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
from time import sleep

GPIO.setup(26, GPIO.OUT)
u = 0.1

def beep(t):
    GPIO.output(26,1)
    sleep(t)
    GPIO.output(26,0)
    sleep(u)


while 1:
    beep(u)
    beep(u)
    beep(u)
    sleep(2*u)
    beep(3*u)
    beep(3*u)
    beep(3*u)
    sleep(2*u)
    beep(u)
    beep(u)
    beep(u)
    sleep(10*u)