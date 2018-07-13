import cv2
import numpy as np

cap = cv2.VideoCapture(0)

ret, frame1 = cap.read()
ret, frame2 = cap.read()

def musky(frame):

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([110,50,50])
    upper_blue = np.array([130,255,255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    res = cv2.bitwise_and(frame,frame, mask= mask)
    return res
   
  while(1):
    
    img1 = musky(cv2.GaussianBlur(frame1,(55,51),0))
    img2 = musky(cv2.GaussianBlur(frame2,(55,51),0))
    diff = cv2.absdiff(img1,img2)
    img3 = cv2.cvtColor(diff,cv2.COLOR_BGR2GRAY)
    kernel = np.kones((480,640),np.uint8)
    erosion = cv2.erode(,kernel,iterations = 5)
    dilation = cv2.dilate(mask,kernel,iterations = 5)
	    
    image, contours, hierarchy = cv2.findContours(img3,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
    cv2.drawContours(frame1, contours, -1, (0,0,255), 3)
    l = len(contours)
    print (l)


    
    if l > 0:
    	for i in range(l):
        	 area = cv2.contourArea(contours[i])
        	 if i is 0:
        	       Max=area
        	       k=0
        	 elif (area>Max):
        	       k=i
        	       Max=area
    	cnt = contours[k]
    	M = cv2.moments(cnt)
    	#print (contours[0])
    	if M['m00'] > 0:
        	cx = int(M['m10']/M['m00'])
        	cy = int(M['m01']/M['m00'])
        
        	cv2.circle(frame1,(cx,cy),5, (0,255,0), -1)
        	print ("{} {}".format(cx,cy))


    cv2.imshow('frame',frame1)
    cv2.imshow('diff',img3)
    ret, frame3 = cap.read()
    frame1 = frame2
    frame2 = frame3
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()

