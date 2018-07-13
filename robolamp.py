import time
import numpy as np
import threading 
import cv2
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(3,GPIO.OUT)
GPIO.setup(5,GPIO.OUT)
GPIO.setup(7,GPIO.OUT)
p = GPIO.PWM(3,50)
q = GPIO.PWM(5,50)
r = GPIO.PWM(7,50)
p.start(7)
q.start(7)
r.start(7)
a1=105
a2=105
a3=105


def musky(frame,c):

    h = c[0]
    s = c[1]
    v = c[2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([h-40,s-40,v-40])
    upper_blue = np.array([h+40,s+40,v+40])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    return mask
	

def find(frame):
   hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
   c = hsv[x,y]
   c1 = hsv[x+10,y]
   c2 = hsv[x,y+10]
   c3 = hsv[x-10,y]
   c4 = hsv[x,y-10]
   k[0]=int((int(c[0])+int(c1[0])+int(c2[0])+int(c3[0])+int(c4[0]))/5)
   k[1]=int((int(c[1])+int(c1[1])+int(c2[1])+int(c3[1])+int(c4[1]))/5)
   k[2]=int((int(c[2])+int(c1[2])+int(c2[2])+int(c3[2])+int(c4[2]))/5)
   return k
   
	
class cam:
      def __init__(self):
          self.cap   =cv2.VideoCapture(0)
          self.cap.set(3,550)
          self.cap.set(4,400)
          (ret,self.f)=self.cap.read()
          self.stop = False
      def start(self):
          t=threading.Thread(target=self.update,args=())
          t.daemon=True
          t.start()
          return self
      def update(self):
          while True:
            if self.stop:
               return False 
            else:
               (ret,self.f) = self.cap.read()
      def read(self):
          #self.t.join()
          return cv2.GaussianBlur(self.f,(55,51),0)
          
      def get(self,a):
          return(self.cap.get(a))
      def terminate(self):
          self.stop = True
		  
	  
cam1 = cam().start()
time.sleep(4)
frame4 = cam1.read()
c = find(frame4) 	
frame1 = cam1.read()
cx = int(cam1.get(3)/2)
cy = int(cam1.get(4)/2)
sx = int(cam1.get(3)/2)
sy = int(cam1.get(4)/2)
g = 1
area = 0
area2=0


def s(area,area2):
        if(area2 > ((area*0.2)+area)):
                dc2 = 1./15.*(a2 + 2)
                q.ChangeDutyCycle(dc2)
        elif(area2 < (area-(area*0.2))):
                dc2 = 1./15.*(a2 - 2)
                q.ChangeDutyCycle(dc2)

while(1):
    frame2 = cam1.read()
    img1 = musky(frame1,c)
    img2 = musky(frame2,c)
    
    diff = cv2.absdiff(img1,img2)
    kernel = np.ones((480,640), np.uint8)
    erosion = cv2.erode(diff,kernel,iterations = 1)
    dilate = cv2.dilate(diff,kernel,iterations = 1)
    image, contours, hierarchy = cv2.findContours(diff,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frame1, contours, -1, (0,0,255), 5)
    l = len(contours)
    print (l)
    
   
    if l > 0:
         k = max(contours,key=cv2.contourArea)
         area = cv2.contourArea(k)
         M = cv2.moments(k)
         if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                cv2.circle(frame1,(cx,cy),5, (0,255,0), -1)
                print ("{} {}".format(cx,cy))

         if(((cx-sx)**2)+((cy-sy)**2)>400):
                if (cx > sx and cy < sy):
                        a1 =  a1 - g
                        a3 =  a3 - g
                elif (cx>sx and cy==sy):
                        a1 = a1
                        a3 = a3 - g

                elif (cx > sx and cy > sy):
                        a1 =  a1 + g
                        a3 =  a3 - g

                elif (cx == sx and cy>sy):
                        a1 =  a1 + g
                        a3 = a3

                elif (cx < sx and cy > sy):
                        a1 =  a1 + g
                        a3 =  a3 + g
   
                elif (cx<sx and cy == sy):   
                        a1 = a1
                        a3 = a3 + g
		
                elif (cx < sx and cy < sy):
                        a1 =  a1 - g
                        a3 =  a3 + g

                elif (cx==sx and cy<sy):
                        a1 = a1 - g
                        a3 = a3
                dc2 = 1./15.*(a2)
                elif(dc1 > 9):
                        a2 = a2 - g
                elif(dc1 < 4):
                        a2 = a2 + g
                dc1 = 1./15.*(a1)
                dc2 = 1./15.*(a2)
                dc3 = 1./15.*(a3)
                p.ChangeDutyCycle(dc1)
                q.ChangeDutyCycle(dc2)
                r.ChangeDutyCycle(dc3)
                s(area,area2)	
    cv2.imshow('frame',frame1)
    frame1 = frame2
    area2 = area
    k = cv2.waitKey(5)
    if k == 27:
        cam1.terminate()
        break
cap.release()
cv2.destroyAllWindows()

