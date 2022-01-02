from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
from PIL import Image, ImageTk
from time import sleep
import tkinter as tk
from array import *
import webbrowser
import base64
import serial


padding = 20
FontFamily = "Helvetica"
FontSize = 34

s = serial.Serial('COM6', 9600, timeout=100)
s.flush()



xp = []
y = [[],[],[],[],[],[],[],[],[],[]]
nx = 0
ny = 0
r = [0,0,0,0,0,0,0,0,0,0] 

def callback(url):
    webbrowser.open_new(url)

root = tk.Tk()

canvas = tk.Canvas(root, width=600,height=300)
canvas.grid(columnspan=4)
canvas.grid_columnconfigure(1,weight=2)

gps = tk.Label(root, text="13.123456 100.123456", font=(FontFamily, FontSize))
gps.grid(columnspan=2, column=0, row=1,padx=padding,pady=padding)
gps.bind("<Button-1>", lambda e: callback("http://www.google.com/maps/search/?api=1&query="+"%.6f,%.6f" %(lat,lon)))
lat = 13.123456
lon = 100.123456
battery = tk.Label(root, text="batt", font=(FontFamily, FontSize))
battery.grid(column=0,row=2,padx=padding,pady=padding)
temp = tk.Label(root, text="temp", font=(FontFamily, FontSize))
temp.grid(column=1,row=2,padx=padding,pady=padding)
heading = tk.Label(root, text="heading", font=(FontFamily, FontSize))
heading.grid(column=2,row=1,padx=padding,pady=padding)
rheight = tk.Label(root, text="rheight", font=(FontFamily, FontSize))
rheight.grid(column=3,row=1,padx=padding,pady=padding)
servopos = tk.Label(root, text="90,90", font=(FontFamily, FontSize))
servopos.grid(column=2,row=2,padx=padding,pady=padding)
sensor = tk.Label(root, text="sensor", font=(FontFamily, FontSize))
sensor.grid(column=3,row=2,padx=padding,pady=padding)
img = Image.open("C:\\Users\\kraiw\\OneDrive\\Desktop\\Cansat 2021\\Ground station\\start.jpg")
img = img.resize((500,500),Image.ANTIALIAS)
tkimage = ImageTk.PhotoImage(img)
simg = tk.Label(root, image=tkimage)
simg.grid(column=2,row=0,columnspan=2,rowspan=1)


fig, (ac,tem,alt,ralt) = plt.subplots(nrows=4,ncols=1)

def animate(i):
    global nx,ny,xp,y,r,lat,lon,imgdata
    nx += 1
    ny += 1
    flag = 1
    while flag:
        try:
            ss = s.readline()
            ss = ss.decode()
            ss = ss.rstrip()
            re = ss.split(',')
            flag = 0
        except:
            pass
    print(ss)
    try:
        if(re[0] != "img"):
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
            tem.plot(xp,y[7],label='temp', color='#aaa')
            tem.set_xticks([])
            alt.plot(xp,y[8],label='altitude', color='#aaa')
            alt.set_xticks([])
            ralt.plot(xp,y[9],label='real altitude', color='#aaa')
            ralt.set_xticks([])

            #ma.set_xticks([])

            mm = "%.6f %.6f" %(lat,lon)
            gps.config(text=mm)
            #lat = lat + 0.000001
            #lon = lon + 0.000001
            lat = r[1]
            lon = r[2]
            battery.config(text=str(re[11]))
            temp.config(text=str(re[7]))
            heading.config(text=str(re[6]))
            rheight.config(text=str(re[9]))
            #servopos.config(text=re[12]+','+re[13])
            sensor.config(text=str(re[10]))
            img = Image.open("C:\\Users\\kraiw\\OneDrive\\Desktop\\Cansat 2021\\Ground station\\img.jpg")
            img = img.resize((500,500),Image.ANTIALIAS)
            tkimage = ImageTk.PhotoImage(img)
            simg.config(image=tkimage)
            simg.photo = tkimage
        else:
            imgstring = re[1]
            imgdata = base64.b64decode(imgstring)
            with open("C:\\Users\\kraiw\\OneDrive\\Desktop\\Cansat 2021\\Ground station\\img.jpg", 'wb') as f:
                f.write(imgdata)
    except:
        pass
    
plotcanvas = FigureCanvasTkAgg(fig, root)
plotcanvas.get_tk_widget().grid(column=0, row=0,columnspan=2,rowspan=1)
ani = FuncAnimation(plt.gcf(), animate, interval=50)

root.mainloop()
