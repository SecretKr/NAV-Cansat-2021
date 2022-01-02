from time import sleep
from ina219 import INA219
from gpiozero import CPUTemperature
from datetime import datetime


ina = INA219(0.1)
ina.configure()


v = ina.voltage()
i = ina.current()
p = ina.power()
m = '{0:0.2f}V {1:0.1f}mA'.format(v, i)
percent = "%d" %((v-5.5)/(2.7)*100)
if int(percent)>100: percent = "100"
if int(percent)<0: percent = "0"
percent += '%'
cpu = CPUTemperature()
now = datetime.now()

print("Battery  ",m)
print("         ",percent)
print("CPU Temp ","%.1f" %cpu.temperature,"C")
print("Time     ",now.strftime("%H:%M:%S"))