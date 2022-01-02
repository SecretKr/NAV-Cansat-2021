from datetime import datetime
from picamera import PiCamera
from ina219 import INA219
import FaBo9Axis_MPU9250
import RPi.GPIO as GPIO
from time import sleep
from math import atan2
GPIO.setmode(GPIO.BCM)
from PIL import Image
import numpy as np
import serial
import base64
import pigpio
import smbus
import time
import math
import sys
import PIL
import os

servo_type = 270
sl = 13
sr = 12
pi = pigpio.pi()
pi.set_mode(sl, pigpio.OUTPUT)
pi.set_mode(sr, pigpio.OUTPUT)

def sa(a,b):
  a = servo_type-a
  pi.set_servo_pulsewidth(sl,500+2000*int(a)/servo_type)
  pi.set_servo_pulsewidth(sr,500+2000*int(b)/servo_type)

camera = PiCamera()
camera.resolution = (1280, 720)
camera.framerate = 30
sensor = 6
buzz = 26
led = 4
u = 0.1
launch = 11
GPIO_TRIGGER = 18
GPIO_ECHO = 24
GPIO.setwarnings(False)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
GPIO.setup(buzz, GPIO.OUT)
GPIO.setup(led, GPIO.OUT)
GPIO.setup(sensor, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(launch, GPIO.IN, pull_up_down=GPIO.PUD_UP)

pd = 0
edis = 0
pressure = 0
temp = 0
def beep(t):
    GPIO.output(26,1)
    GPIO.output(4,1)
    sleep(t)
    GPIO.output(26,0)
    GPIO.output(4,0)
    sleep(t)
def bmpp():
    global temp,pressure
    bus = smbus.SMBus(1)
    try:
        b1 = bus.read_i2c_block_data(0x76, 0x88, 24)
        dig_T1 = b1[1] * 256 + b1[0]
        dig_T2 = b1[3] * 256 + b1[2]
        if dig_T2 > 32767 :
            dig_T2 -= 65536
        dig_T3 = b1[5] * 256 + b1[4]
        if dig_T3 > 32767 :
            dig_T3 -= 65536
        dig_P1 = b1[7] * 256 + b1[6]
        dig_P2 = b1[9] * 256 + b1[8]
        if dig_P2 > 32767 :
            dig_P2 -= 65536
        dig_P3 = b1[11] * 256 + b1[10]
        if dig_P3 > 32767 :
            dig_P3 -= 65536
        dig_P4 = b1[13] * 256 + b1[12]
        if dig_P4 > 32767 :
            dig_P4 -= 65536
        dig_P5 = b1[15] * 256 + b1[14]
        if dig_P5 > 32767 :
            dig_P5 -= 65536
        dig_P6 = b1[17] * 256 + b1[16]
        if dig_P6 > 32767 :
            dig_P6 -= 65536
        dig_P7 = b1[19] * 256 + b1[18]
        if dig_P7 > 32767 :
            dig_P7 -= 65536
        dig_P8 = b1[21] * 256 + b1[20]
        if dig_P8 > 32767 :
            dig_P8 -= 65536
        dig_P9 = b1[23] * 256 + b1[22]
        if dig_P9 > 32767 :
            dig_P9 -= 65536
        dig_H1 = bus.read_byte_data(0x76, 0xA1)
        b1 = bus.read_i2c_block_data(0x76, 0xE1, 7)
        dig_H2 = b1[1] * 256 + b1[0]
        if dig_H2 > 32767 :
            dig_H2 -= 65536
        dig_H3 = (b1[2] &  0xFF)
        dig_H4 = (b1[3] * 16) + (b1[4] & 0xF)
        if dig_H4 > 32767 :
            dig_H4 -= 65536
        dig_H5 = (b1[4] / 16) + (b1[5] * 16)
        if dig_H5 > 32767 :
            dig_H5 -= 65536
        dig_H6 = b1[6]
        if dig_H6 > 127 :
            dig_H6 -= 256
        bus.write_byte_data(0x76, 0xF2, 0x01)
        bus.write_byte_data(0x76, 0xF4, 0x27)
        bus.write_byte_data(0x76, 0xF5, 0xA0)
        data = bus.read_i2c_block_data(0x76, 0xF7, 8)
        adc_p = ((data[0] * 65536) + (data[1] * 256) + (data[2] & 0xF0)) / 16
        adc_t = ((data[3] * 65536) + (data[4] * 256) + (data[5] & 0xF0)) / 16
        adc_h = data[6] * 256 + data[7]
        var1 = ((adc_t) / 16384.0 - (dig_T1) / 1024.0) * (dig_T2)
        var2 = (((adc_t) / 131072.0 - (dig_T1) / 8192.0) * ((adc_t)/131072.0 - (dig_T1)/8192.0)) * (dig_T3)
        t_fine = (var1 + var2)
        cTemp = (var1 + var2) / 5120.0
        fTemp = cTemp * 1.8 + 32
        var1 = (t_fine / 2.0) - 64000.0
        var2 = var1 * var1 * (dig_P6) / 32768.0
        var2 = var2 + var1 * (dig_P5) * 2.0
        var2 = (var2 / 4.0) + ((dig_P4) * 65536.0)
        var1 = ((dig_P3) * var1 * var1 / 524288.0 + ( dig_P2) * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * (dig_P1)
        p = 1048576.0 - adc_p
        p = (p - (var2 / 4096.0)) * 6250.0 / var1
        var1 = (dig_P9) * p * p / 2147483648.0
        var2 = p * (dig_P8) / 32768.0
        pressure = (p + (var1 + var2 + (dig_P7)) / 16.0) / 100
        var_H = ((t_fine) - 76800.0)
        var_H = (adc_h - (dig_H4 * 64.0 + dig_H5 / 16384.0 * var_H)) * (dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * var_H * (1.0 + dig_H3 / 67108864.0 * var_H)))
        humidity = var_H * (1.0 -  dig_H1 * var_H / 524288.0)
        if humidity > 100.0 :
            humidity = 100.0
        elif humidity < 0.0 :
            humidity = 0.0

        temp = "%.2f" %cTemp
        pressure = "%.2f" %pressure
    except:
        temp = ""
        pressure = ""
def GPS_Info():
    global NMEA_buff
    global lat_in_degrees
    global long_in_degrees
    global time
    nmea_time = []
    nmea_latitude = []
    nmea_longitude = []
    nmea_time = NMEA_buff[0]                    #extract time from GPGGA string
    nmea_latitude = NMEA_buff[1]                #extract latitude from GPGGA string
    nmea_longitude = NMEA_buff[3]
    t =nmea_time               #extract longitude from GPGGA string
    
    gpstime =  str((int(t[0]+t[1])+7)%24),":",t[2],t[3],":",t[4],t[5]
    
    lat = float(nmea_latitude)                  #convert string into float for calculation
    longi = float(nmea_longitude)               #convertr string into float for calculation
    
    lat_in_degrees = convert_to_degrees(lat)    #get latitude in degree decimal format
    long_in_degrees = convert_to_degrees(longi) #get longitude in degree decimal format
def convert_to_degrees(raw_value):
    decimal_value = raw_value/100.00
    degrees = int(decimal_value)
    mm_mmmm = (decimal_value - int(decimal_value))/0.6
    position = degrees + mm_mmmm
    position = "%.6f" %(position)
    return position
def distance():
    global pd
    GPIO.output(GPIO_TRIGGER, True)
    sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    StartTime = time.time()
    StopTime = time.time()
    tmo = StartTime
    edis = 1
    while GPIO.input(GPIO_ECHO) == 0 and edis:
        StartTime = time.time()
        sleep(0.00001)
        if time.time()-tmo >= 0.06:
            edis = 0
    if edis:
        while GPIO.input(GPIO_ECHO) == 1:
            StopTime = time.time()
        TimeElapsed = StopTime - StartTime
        distance = TimeElapsed * 17150
        distance = "%.2f" % (distance/100)
        pd = distance
        return distance
    else:
        return pd

ina = INA219(0.1)
ina.configure()

gpgga_info = "$GNGGA,"
ser = serial.Serial(
    port='/dev/ttyS0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0.02
)
GPGGA_buffer = 0
NMEA_buff = 0
lat_in_degrees = ""
long_in_degrees = ""
gpstime = ""

PI = 3.14159265
mpu9250 = FaBo9Axis_MPU9250.MPU9250()
ti=0
counter=0
GPIO.output(buzz,1)
GPIO.output(led,1)
sleep(0.1)
GPIO.output(buzz,0)
sleep(0.9)
GPIO.output(led,0)

now = datetime.now()
current_time = now.strftime("%H-%M-%S")
filen = str(current_time)
finum = 0
vdnum = 0
milli_sec = int(round(time.time() * 1000))
smilli = milli_sec
lmillis = 0
nakono = 0

f = open("/home/pi/cansat/gycal.txt", "r")
calmy = float(f.readline())
calmz = float(f.readline())
north = float(f.readline())
calax = float(f.readline())
calay = float(f.readline())
calaz = float(f.readline())
f.close()

def pmm():
    milli_sec = int(round(time.time() * 1000))
    print("start",milli_sec%100000)

while GPIO.input(launch) == 0:
    ti+=1
    ti = round(ti,2)
    mm = str(ti)
#GPS
    try:
        received_data = (str)(ser.readline())
        GPGGA_data_available = received_data.find(gpgga_info)
        if (GPGGA_data_available>0):
            print("GPS!!")
            GPGGA_buffer = received_data.split("$GNGGA,",1)[1]  #store data coming after "$GPGGA," string 
            NMEA_buff = (GPGGA_buffer.split(','))               #store comma separated data in buffer
            GPS_Info()                                          #get time, latitude, longitude
            mm+= ','+lat_in_degrees+','+long_in_degrees
        else:
            mm+= ','+lat_in_degrees+','+long_in_degrees
    except:
        mm+= ','+lat_in_degrees+','+long_in_degrees
#MPU BMP
    try:
        ac = mpu9250.readAccel()
        ma = mpu9250.readMagnet()
        mm+= ","+"%.3f" % (ac['x']+calax)+","+"%.3f" % (ac['y']+calay)+","+"%.3f" % (ac['z']+calaz)
        angle = atan2(ma['z']+calmz,ma['y']+calmy) * 180 / PI
        angle += north
        if angle < -180: angle+=360
        if angle > 180: angle-=360
        angle = "%d" %angle
        mm+= ","+angle
    except:
        mm+= ",,,,"
#BMP
    bmpp()
    try:
        alt = 44331.5 - 4946.62 * (float(pressure)*100) ** (0.190263)
        alt = "%.2f" %alt
    except:
        alt = ""
    try:
        temp = int(str("%d" %float(temp)))
    except:
        temp = ''
    mm+= ","+str(temp)+","+str(alt)
#Ultrasonic
    Dis = distance()
    if Dis > 700: Dis = 700
    Dis = str(Dis)
    mm+= ","+Dis

#Sensor
    s1 = GPIO.input(sensor)
    mm+= ","+str(s1)

#Battery
    V = ina.voltage()
    I = ina.current()
    percent = "%d" %((V-6)/(2.2)*100)
    if int(percent)>100: percent = "100"
    if int(percent)<0: percent = "0"
    mm+=","+percent

    camera.capture('tem.jpg', use_video_port=True)
    picture = Image.open('tem.jpg')
    picture.thumbnail((128,128), Image.ANTIALIAS)
    picture.save("s_tem.jpg",optimize=True,quality=10)
    with open("s_tem.jpg", "rb") as img_file:
        simg = "img,"+str(base64.b64encode(img_file.read()).decode('utf-8'))+",,"
    try:
        ser.write(bytes(mm,'utf-8'))
        ser.write(b"\n")
        ser.write(bytes(simg,'utf-8'))
        ser.write(b"\n")
    except:
        print("send error")
    print(mm)

    milli_sec = int(round(time.time() * 1000))
    sleep((1000 - milli_sec % 1000)/1000)











bmpp()
sleep(1)
bmpp()
spacey = 0
percentMin = 100
try:
    spacey = (44331.5 - 4946.62 * (float(pressure)*100) ** (0.190263))
except:
    spacey = 0

launch = 0
#camera.start_recording('camera/'+filen+' ('+str(vdnum)+').h264')

while True:
    ti+=1
    ti = round(ti,2)
    mm = str(ti)
    mmf = str(ti)
#GPS
    try:
        received_data = (str)(ser.readline())
        GPGGA_data_available = received_data.find(gpgga_info)
        if (GPGGA_data_available>0):
            print("GPS!!")
            GPGGA_buffer = received_data.split("$GNGGA,",1)[1]  #store data coming after "$GPGGA," string 
            NMEA_buff = (GPGGA_buffer.split(','))               #store comma separated data in buffer
            GPS_Info()                                          #get time, latitude, longitude
            mm+= ','+lat_in_degrees+','+long_in_degrees
            mmf+= ','+lat_in_degrees+','+long_in_degrees+','+gpstime
        else:
            #mm+= ',n/a,n/a'
            #mmf+= ',n/a,n/a,n/a'
            mm+= ','+lat_in_degrees+','+long_in_degrees
            mmf+= ','+lat_in_degrees+','+long_in_degrees+','+gpstime
    except:
        #mm+= ',n/a,n/a'
        #mmf+= ',n/a,n/a,n/a'
        mm+= ','+lat_in_degrees+','+long_in_degrees
        mmf+= ','+lat_in_degrees+','+long_in_degrees+','+gpstime
#MPU
    try:
        ac = mpu9250.readAccel()
        gy = mpu9250.readGyro()
        ma = mpu9250.readMagnet()
        mm+= ","+"%.3f" % (ac['x']+calax)+","+"%.3f" % (ac['y']+calay)+","+"%.3f" % (ac['z']+calaz)
        mmf+= ","+"%.3f" % (ac['x']+calax)+","+"%.3f" % (ac['y']+calay)+","+"%.3f" % (ac['z']+calaz)
        mmf+= ","+str(gy['x'])+","+str(gy['y'])+","+str(gy['z'])
        mmf+= ","+str(ma['x'])+","+str(ma['y'])+","+str(ma['z'])
        angle = atan2(ma['z']+calmz,ma['y']+calmy) * 180 / PI
        angle += north
        if angle < -180: angle+=360
        if angle > 180: angle-=360
        angle = "%d" %angle
        mm+= ","+angle
        mmf+= ","+angle
    except:
        mm+= ",,,,"
        mmf+= ",,,,,,,,,,"
#BMP
    bmpp()
    try:
        alt = (44331.5 - 4946.62 * (float(pressure)*100) ** (0.190263))-spacey
        alt = "%.2f" %alt
    except:
        alt = ""
    try:
        temp = int(str("%d" %float(temp)))
    except:
        temp = ''
    mm+= ","+str(temp)+","+str(alt)
    mmf+= ","+str(temp)+","+str(pressure)+","+str(alt)
#Ultrasonic
    Dis = distance()
    if float(Dis) > 7: Dis = "7"
    Dis = str(Dis)
    mm+= ","+Dis
    mmf+= ","+Dis

#Sensor
    s1 = GPIO.input(sensor)
    mm+= ","+str(s1)
    mmf+= ","+str(s1)

#Battery
    V = ina.voltage()
    I = ina.current()
    percent = (V-6)/(2.2)*100
    if percent > percentMin: percent = percentMin
    else: percentMin = percent
    percent = "%d" %percent
    if int(percent)>100: percent = "100"
    if int(percent)<0: percent = "0"
    mm+=","+percent
    mmf+= ","+percent+","+"%.2f" %V+","+"%.1f"%I

#Servo
    camera.capture('tem.jpg', use_video_port=True)
    img = Image.open('tem.jpg')
    red = 0
    green = 0
    blue = 0
    for i in range(520,761,10):
        for j in range(0,241,10):
            nino = img.getpixel((i,j))
            red += nino[0]
            green += nino[1]
            blue += nino[2]
    red = int(red/576)
    green = int(green/576)
    blue = int(blue/576)

    redl = 0
    greenl = 0
    bluel = 0
    for i in range(0,181,10):
        for j in range(0,181,10):
            nino = img.getpixel((i,j))
            redl += nino[0]
            greenl += nino[1]
            bluel += nino[2]
    redl = int(redl/324)
    greenl = int(greenl/324)
    bluel = int(blue/576)

    redr = 0
    greenr = 0
    bluer = 0
    for i in range(1099,1280,10):
        for j in range(0,181,10):
            nino = img.getpixel((i,j))
            redr += nino[0]
            greenr += nino[1]
            bluer += nino[2]
    redr = int(redr/324)
    greenr = int(greenr/324)
    bluer = int(blue/576)

    print(redl,greenl,red,green,redr,greenr)
    
    if green > red-30 and green > blue-30 and green > 50:
        lg = greenl > redl and greenl > bluel
        rg = greenr > redr and greenr > bluer
        if lg > rg:
            sa(0,270)
            mm+=",0,270"
            mmf+=",0,270"
        elif lg < rg:
            sa(270,0)
            mm+=",270,0"
            mmf+=",270,0"
        else:
            sa(135,135)
            mm+=",135,135"
            mmf+=",135,135"
    else:
        sa(135,135)
        mm+=",135,135"
        mmf+=",135,135"

#Launch
    vec = math.sqrt(pow(ac['x']+calax,2)+pow(ac['y']+calay,2)+pow(ac['z']+calaz,2))
    if vec >= 2: 
        launch = 1
        lmillis = int(round(time.time() * 1000))
    if int(round(time.time() * 1000)) - lmillis >= 30000 and launch == 1:
        launch = 2
        nakono = int(round(time.time() * 1000))
    if launch == 2:
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
    if int(round(time.time() * 1000)) - nakono >= 20000 and launch == 2: launch = 0
    mm += ","+str(launch)
    mmf += ","+str(launch)


    fie = open(str("/home/pi/cansat/log/"+filen+" ("+str(finum)+").csv"), "a")
    now = datetime.now()
    current_time = now.strftime("%H:%M:%S")
    mmf+= ","+current_time
    fie.write(mmf+'\n')
    fie.close()
    #if os.stat("/home/pi/cansat/log/"+filen+" ("+str(finum)+").csv").st_size >= 4096: finum += 1

    picture = Image.open('tem.jpg')
    picture.thumbnail((96,96), Image.ANTIALIAS)
    picture.save("s_tem.jpg",optimize=True,quality=10)
    with open("s_tem.jpg", "rb") as img_file:
        simg = "img,"+str(base64.b64encode(img_file.read()).decode('utf-8'))+",,"
    try:
        ser.write(bytes(mm,'utf-8'))
        ser.write(b"\n")
        ser.write(bytes(simg,'utf-8'))
        ser.write(b"\n")
    except:
        print("send error")
    print(mm)

    
    milli_sec = int(round(time.time() * 1000))
    sleep((1000 - milli_sec % 1000)/1000)

    # if(milli_sec - smilli >= 300000):
    #     camera.stop_recording()
    #     smilli = milli_sec
    #     vdnum += 1
    #     camera.start_recording('camera/'+filen+' ('+str(vdnum)+').h264')

