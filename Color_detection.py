import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11,GPIO.IN,pull_up_dowm=GPIO.PUD_UP)
key = False

def musky(frame,c):

    h = c[0]
    s = c[1]
    v = c[2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([h-40,s-40,v-40])
    upper_blue = np.array([h+40,s+40,v+40])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    res = cv2.bitwise_and(frame,frame, mask= mask)
    return res
   

def find(frame):
   hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
   c = hsv[x,y]
   c2 = hsv[x+10,y]
   c3 = hsv[x,y+10]
   c3 = hsv[x-10,y]
   c4 = hsv[x,y-10]
   k[0]=int((c[0]+c1[0]+c2[0]+c3[0]+c4[0])/5)
   k[1]=int((c[1]+c1[1]+c2[1]+c3[1]+c4[1])/5)
   k[2]=int((c[2]+c1[2]+c2[2]+c3[2]+c4[2])/5)
   return k
   
while(True):
   
   if GPIO.input(11)==0:
         print("RoboLamp ready to rock")
         if key == False:
              key = True
              sleep(.5)
              cap = cv2.VideoCapture(0)
              x = int((cap.get(3)/2))
              y = int((cap.get(4)/2))
              time.sleep(4)
              ret, frame1 = cap.read()
              c = find(frame1)            
              
              while(True):
                   ret, frame2 = cap.read()
                   img = musky(frame2,c)
                   cv2.imshow('cap',frame2)
                   cv2.imshow('img',img)
                   if GPIO.input(11)==0:
                            key = False
                            break
                   if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
              cap.release()
              cv2.destroyAllWindows()
              break
         break



   


 

 

