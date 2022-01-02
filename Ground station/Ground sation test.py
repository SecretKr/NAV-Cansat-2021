import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from array import *

s = serial.Serial('COM3', 9600, timeout=100)
s.flush()
xp = []
y = [[],[],[],[],[],[],[],[],[],[]]
nx = 0
ny = 0
r = [0,0,0,0,0,0,0,0,0,0]

fig, (ac,tem,alt,ralt) = plt.subplots(nrows=4,ncols=1)

def animate(i):
    global nx,ny,xp,y,r
    nx += 1
    ny += 1
    ss = s.readline()
    try:
        ss = ss.decode()
        ss = ss.rstrip()
        re = ss.split(',')
    except:
        pass
    print(ss)
    for j in range(0,10):
        try:
            r[j] = float(re[j])
        except:
            pass
    xp.append(re[0])
    for j in range(1,10):
        y[j].append(r[j])
    if(len(xp) >= 80):
        xp.pop(0)
        for j in range(1,10):
            y[j].pop(0)

    ac.cla()
    tem.cla()
    alt.cla()
    ralt.cla()
    ac.plot(xp,y[3],label='x', color='#f00')
    ac.plot(xp,y[4],label='y', color='#0f0')
    ac.plot(xp,y[5],label='z', color='#00f')
    ac.set_xticks([])
    tem.plot(xp,y[6],label='temp', color='#aaa')
    tem.set_xticks([])
    alt.plot(xp,y[7],label='altitude', color='#aaa')
    alt.set_xticks([])
    ralt.plot(xp,y[8],label='real altitude', color='#aaa')
    ralt.set_xticks([])

    #ma.set_xticks([])
    

ani = FuncAnimation(plt.gcf(), animate, interval=50)

plt.tight_layout()
plt.show()
