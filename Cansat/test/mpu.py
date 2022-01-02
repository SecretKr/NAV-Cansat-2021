import FaBo9Axis_MPU9250
from time import sleep
import sys
import serial

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
    
    time =  str((int(t[0]+t[1])+7)%24),":",t[2],t[3],":",t[4],t[5]
    
    lat = float(nmea_latitude)                  #convert string into float for calculation
    longi = float(nmea_longitude)               #convertr string into float for calculation
    
    lat_in_degrees = convert_to_degrees(lat)    #get latitude in degree decimal format
    long_in_degrees = convert_to_degrees(longi) #get longitude in degree decimal format
    
#convert raw NMEA string into degree decimal format   
def convert_to_degrees(raw_value):
    decimal_value = raw_value/100.00
    degrees = int(decimal_value)
    mm_mmmm = (decimal_value - int(decimal_value))/0.6
    position = degrees + mm_mmmm
    position = "%.4f" %(position)
    return position
    

gpgga_info = "$GNGGA,"
ser = serial.Serial(
    port='/dev/ttyS0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)
GPGGA_buffer = 0
NMEA_buff = 0
lat_in_degrees = 0
long_in_degrees = 0
time = ""

mpu9250 = FaBo9Axis_MPU9250.MPU9250()
ti=0
counter=0

while True:
    ti+=0.1
    ti = round(ti,2)
    mm = str(ti)
    mmf = str(ti)
#GPS
    received_data = (str)(ser.readline())
    GPGGA_data_available = received_data.find(gpgga_info)
    if (GPGGA_data_available>0):
        print("GPS!!")
        GPGGA_buffer = received_data.split("$GNGGA,",1)[1]  #store data coming after "$GPGGA," string 
        NMEA_buff = (GPGGA_buffer.split(','))               #store comma separated data in buffer
        GPS_Info()                                          #get time, latitude, longitude
        mm+= ','+lat_in_degrees+','+long_in_degrees+','+time
        mmf+= ','+lat_in_degrees+','+long_in_degrees+','+time
#MPU BMP

    ac = mpu9250.readAccel()
    gy = mpu9250.readGyro()
    ma = mpu9250.readMagnet()

    mmf+= ","+str(ac['x'])+","+str(ac['y'])+","+str(ac['z'])
    mmf+= ","+str(ac['x'])+","+str(ac['y'])+","+str(ac['z'])
    mmf+= ","+str(gy['x'])+","+str(gy['y'])+","+str(gy['z'])
    mm+= ","+str(ma['x'])+","+str(ma['y'])+","+str(ma['z'])
    print(mm)
    #ser.write(bytes(mm,'utf-8'))
    #ser.write(b"\n")

    sleep(0.1)
