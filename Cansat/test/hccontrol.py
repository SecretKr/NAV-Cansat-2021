from time import sleep
import serial
import pigpio

servo_type = 270
sl = 13
sr = 12
pi = pigpio.pi()
pi.set_mode(sl, pigpio.OUTPUT)
pi.set_mode(sr, pigpio.OUTPUT)

def sa(a,b):
    b = servo_type-b
    pi.set_servo_pulsewidth(sl,500+2000*int(a)/servo_type)
    pi.set_servo_pulsewidth(sr,500+2000*int(b)/servo_type)
def sa1(a):
    pi.set_servo_pulsewidth(sl,500+2000*int(a)/servo_type)
    pi.set_servo_pulsewidth(sr,500+2000*int(a)/servo_type)

ser = serial.Serial(
    port='/dev/ttyS0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0.02
)

while True:
    received_data = (str)(ser.readline())
    if (received_data):
        d = received_data.split(',')
        sa(d[0],d[1])