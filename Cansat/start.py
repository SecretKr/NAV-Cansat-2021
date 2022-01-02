import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
import time
import os

GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_UP)
input = GPIO.input(11)
while 1:
    if GPIO.input(11):
        print("run")
        os.system("sudo python3  /home/pi/cansat/cansat.py")
        break