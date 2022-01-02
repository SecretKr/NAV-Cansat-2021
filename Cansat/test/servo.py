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

if __name__ == '__main__':
  try:
    while True:
      o = input()
      sa(o,o)
  except KeyboardInterrupt:
    pi.stop()