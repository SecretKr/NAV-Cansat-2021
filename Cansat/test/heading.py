import FaBo9Axis_MPU9250
from time import sleep
from math import atan2
import sys
import serial

PI = 3.14159265
mpu9250 = FaBo9Axis_MPU9250.MPU9250()

f = open("/home/pi/cansat/gycal.txt", "r")
oy = float(f.readline())
oz = float(f.readline())
north = float(f.readline())
f.close()

while True:
    ma = mpu9250.readMagnet()

    angle = atan2(ma['z']+oz,ma['y']+oy) * 180 / PI
    angle += north
    if angle < -180: angle+=360
    if angle > 180: angle-=360
    
    angle = "%.3f" %angle
    print(angle)

    sleep(0.2)
