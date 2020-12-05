#! /usr/bin/python
import numpy as np
import cv2

kernel = np.ones((9,9),np.uint8)
lower=np.array([0,50,46])
upper=np.array([5,250,255])
def find_ball(img):
    
    #img=cv2.bilateralFilter(img,1,25,60)
    img=cv2.GaussianBlur(img,(11,11),0)
    hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,lower,upper)
    mask=cv2.erode(mask,kernel,iterations=2)
    mask=cv2.dilate(mask,kernel,iterations=3)
    cnts=cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(cnts)>0:
	c=max(cnts,key=cv2.contourArea)
	((x,y),radius)=cv2.minEnclosingCircle(c)
	M=cv2.moments(c)
	center=(int(M["m10"]/M["m00"]),int(M["m01"]/M["m00"]))
        print(radius)
	if radius>5:
	    cv2.circle(img,(int(x),int(y)),int(radius),(0,255,255),2)
	    cv2.circle(img,center,5,(0,255,255),-1)
            cv2.imshow("i",img)
            cv2.imshow("m",mask)
    #th3 = cv2.adaptiveThreshold(GrayImage,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,  
                   # cv2.THRESH_BINARY,3,5)
    #erosion = cv2.erode(th3,kernel,iterations=1)
    #dilation = cv2.dilate(erosion,kernel,iterations=1)
    #xgrad=cv2.Sobel(dilation,cv2.CV_16SC1,1,0)
    #ygrad=cv2.Sobel(dilation,cv2.CV_16SC1,0,1)
    #ret,th1 = cv2.threshold(GrayImage,127,255,cv2.THRESH_BINARY)
    #th2 = cv2.adaptiveThreshold(GrayImage,255,cv2.ADAPTIVE_THRESH_MEAN_C,  
      #              cv2.THRESH_BINARY,3,5)  



    #kernel = np.ones((5,5),np.uint8)
        
img=cv2.imread("/home/gf/bebop_ws/src/bebop_autonomy/bebop_driver/scripts/1.jpg")
find_ball(img)

cv2.imshow("img",img)
key=cv2.waitKey(0)
if key==ord('q'):
    cv2.destroyAllWindows()
