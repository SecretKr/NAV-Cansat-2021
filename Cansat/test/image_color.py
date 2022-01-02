import cv2
import time
import matplotlib.pyplot as plt
import numpy as np
import math
#-------------------------------------------------------------------------
#function
def rgb_to_hsv(r, g, b):
    r = float(r)
    g = float(g)
    b = float(b)
    high = max(r, g, b)
    low = min(r, g, b)
    h, s, v = high, high, high

    d = high - low
    s = 0 if high == 0 else d/high

    if high == low:
        h = 0.0
    else:
        h = {
            r: (g - b) / d + (6 if g < b else 0),
            g: (b - r) / d + 2,
            b: (r - g) / d + 4,
        }[high]
        h /= 6

    return [math.floor(h*360), math.floor(s*255), math.floor(v)]
#-------------------------------------------------------------------------
index = int(input("loop"))
cap = cv2.VideoCapture(0)
con =0
cap.set(3,640)
cap.set(4,460)
if cap is None or not cap.isOpened():
    print("could not found")
while(cap.isOpened()):
    ret,frame = cap.read()
    if con<index:
        if ret==True:
            miku = np.array(frame)
            num_row = miku.shape
            new_array =[]
            b=[]
            last=[]
            v=[]
            #สลับค่าสี
            for c in range(0,num_row[0]):
                for i in range(0,num_row[1]):
                    b.append([miku[c,i,2],miku[c,i,1],miku[c,i,0]])
                new_array.append(b)
                b=[]
            plt.imshow(new_array)
            plt.show()
            new_array = np.array(new_array)
            for index in range(0,num_row[0]):
                for value in range(0,num_row[1]):
                    a = rgb_to_hsv(new_array[index,value,0],new_array[index,value,1],new_array[index,value,2])
                    v.append(a)
                last.append(v)
                v=[]
            last=np.array(last)
            value = new_array.shape
            q1=0
            for c in range(0,math.floor(value[0]/2),1):
                for i in range(math.floor(value[1]/2),value[1],1):
                    if last[c,i,0]>=80 and last[c,i,0]<=150:
                        q1+=1
                        new_array[c,i,0]=255
            q2=0
            for c in range(0,math.floor(value[0]/2),1):
                for i in range(0,math.floor(value[1]/2),1):
                    if last[c,i,0]>=80 and last[c,i,0]<=150:
                        q2+=1
                        new_array[c,i,0]=255
            q3=0
            for c in range(math.floor(value[0]/2),value[0],1):
                for i in range(0,math.floor(value[1]/2)):
                    if last[c,i,0]>=80 and last[c,i,0]<=150:
                        q3+=1
                        new_array[c,i,0]=255
            q4=0
            for c in range(math.floor(value[0]/2),value[0],1):
                for i in range(math.floor(value[1]/2),value[1],1):
                    if last[c,i,0]>=80 and last[c,i,0]<=150:
                        q4+=1
                        new_array[c,i,0]=255
            size_quardant = math.ceil((value[0]*value[1])/4)
            condition = math.floor(size_quardant/6)
            a = []
            if q1>=condition :
                a.append(1)
            else:
                a.append(0)
            if q2>=condition :
                a.append(1)
            else:
                a.append(0)
            if q3>=condition :
                a.append(1)
            else:
                a.append(0)
            if q4>=condition :
                a.append(1)
            else:
                a.append(0)
            print(a)
            time.sleep(8)
            con+=1
    else:
        cv2.destroyAllWindows()
        break
