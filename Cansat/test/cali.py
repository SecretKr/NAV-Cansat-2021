import FaBo9Axis_MPU9250
from time import sleep
from math import atan2
import sys

PI = 3.14159265
mpu9250 = FaBo9Axis_MPU9250.MPU9250()

print("calibrating...")
ma = mpu9250.readMagnet()
may = ma['y']
miy = ma['y']
maz = ma['z']
miz = ma['z']
for i in range(0,5):
	ma = mpu9250.readMagnet()
	print(ma['y'],ma['z'], end = "\r")
	sleep(0.2)
	
for i in range(0,100):
	ma = mpu9250.readMagnet()
	print(ma['y'],ma['z'], end = "\r")
	may = max(may,ma['y'])
	miy = min(miy,ma['y'])
	maz = max(maz,ma['z'])
	miz = min(miz,ma['z'])
	sleep(0.2)

oy = -(may+miy)/2
oz = -(maz+miz)/2
print("offset Y: %.2f"%oy)
print("offset Z: %.2f"%oz)

input("point cansat to North")
avg = 0
ax = 0
ay = 0
az = 0
for i in range(0,5):
	ma = mpu9250.readMagnet()
	ac = mpu9250.readAccel()
	angle = atan2(ma['z']+oz,ma['y']+oy) * 180 / PI
	avg += angle
	ax += ac['x']
	ay += ac['y']
	az += ac['z']
	sleep(0.2)

ax /= 5
ay /= 5
az /= 5
ax = "%.3f" %-ax
ay = "%.3f" %(1-ay)
az = "%.3f" %-az
avg /= 5
avg = "%.2f" %-avg
print("north =",avg)

oy = "%.2f" %oy
oz = "%.2f" %oz
f = open("/home/pi/cansat/gycal.txt", "w")
mm = oy+"\n"+oz+"\n"+avg+"\n"+ax+"\n"+ay+"\n"+az
f.write(mm)
f.close()

print("finished")
