#!/usr/bin/env python
import roslib

import sys
import rospy
import cv2
import PID
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
from geometry_msgs.msg import Twist
import numpy as np
num=0
P=0.002
I=0
D=0
P1=0.003
I1=0
D1=0.0001
P2=0.001
I2=0
D2=0
global focalLength 
focalLength=0
control=30
i=0
control_last=0
kernel = np.ones((9,9),np.uint8)
last_measurement=current_measurement=np.array((2,1),np.float32)
print(last_measurement)
last_prediction=current_prediction=np.array((2,1),np.float32)
kalman=cv2.KalmanFilter(4,2)
kalman.measurementMatrix=np.array([[1,0,0,0],[0,1,0,0]],np.float32)
kalman.transitionMatrix=np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]],np.float32)
kalman.processNoiseCov=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],np.float32)*0.03
kalman.measurementNoiseCov=np.array([[1,0],[0,1]],np.float32)

def KF(r,d):
    global last_measurement,current_measurement,last_prediction,current_prediction
    last_measurement=current_measurement
    last_prediction=current_prediction
    current_measurement=np.array([np.float32(r),np.float32(d)],np.float32)
    kalman.correct(current_measurement)
    current_prediction=kalman.predict()
    cr,cd=current_prediction[0],current_prediction[1]
    return cr,cd
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

    
        

        return x,y,radius,center
    else:
	return 0,0,0,0
def distance_to_camera(knownWidth, focalLength, perWidth):  
    # compute and return the distance from the maker to the camera
    return (knownWidth * focalLength) / perWidth            
KNOWN_DISTANCE = 65.0 
KNOWN_WIDTH = 20.0
KNOWN_HEIGHT = 20.0

kernel = np.ones((5,5),np.uint8)
lower=np.array([0,50,46])
upper=np.array([5,250,255])
class img_transform:
    def __init__(self):

	self.bridge=CvBridge()
	self.image_sub = rospy.Subscriber("/bebop/image_raw",Image,self.callback)
    def callback(self,data):
	global num
	global focalLength, control,i,control_last
	pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size = 1)
        pid1=PID.PID(P,I,D)
    	pid1.SetPoint=0.0
    	pid1.setSampleTime(0.01)
    	pid2=PID.PID(P1,I1,D1)
    	pid2.SetPoint=0.0
    	pid2.setSampleTime(0.01)
	pid3=PID.PID(P2,I2,D2)
    	pid3.SetPoint=0
    	pid3.setSampleTime(0.01)
       # print("get")
	try:
	    cv_image=self.bridge.imgmsg_to_cv2(data,"bgr8")
	    rec=400
	    #cv_image=cv_image[int(428-rec/2):int(428+rec/2),0:480,:]
                
	    x,y,marker,center=find_ball(cv_image)
            #print(marker)
	    if marker !=0:

		inches = distance_to_camera(KNOWN_WIDTH, focalLength, 2*marker)
                r,d=KF(marker,inches)
		cv2.putText(cv_image, "%.2fcm" % (d),
             (cv_image.shape[1] - 200, cv_image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,
	     2.0, (0, 255, 0), 3)
		if r>1:
	    	    cv2.circle(cv_image,(int(x),int(y)),int(r),(0,255,255),2)
		    cv2.circle(cv_image,(int(x),int(y)),int(marker),(0,0,255),2)
	    	    cv2.circle(cv_image,center,5,(0,255,255),-1)
	        pid1.update(center[0]-cv_image.shape[1]/2)
		#print(center[1]-cv_image.shape[0]/2)
		pid2.update(center[1]-cv_image.shape[0]/2)
		pid3.update(control-inches)
                output_y=pid1.output
                output_z=pid2.output
		output_x=pid3.output
		twist = Twist()
        	twist.linear.x = output_x; twist.linear.y = 0; twist.linear.z = output_z;
        	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = output_y
        	pub.publish(twist)
                if (inches<control and i==0) :
                    control_last=control
                    control=200
                    i=1
                if (inches>=200 and i==1) :
                     control=control_last-5
                     i=0

                
           # key=cv2.waitKey(1)
	    #if key==ord('a'):
		#num=num+1
		#filename="%s.jpg"%num
		#cv2.imwrite(filename,cv_image)

	except CvBridgeError as e :
	    print(e)
	cv2.imshow("Image window",cv_image)
	cv2.waitKey(1)

if __name__ == '__main__':
    IMAGE_PATHS = ["/home/gf/bebop_ws/src/1.jpg", "/home/gf/bebop_ws/src/2.jpg", "/home/gf/bebop_ws/src/3.jpg"]
    rospy.init_node('image_trans',anonymous=True)
    image = cv2.imread("/home/gf/bebop_ws/src/1.jpg")
    #cv2.imshow("1",image)
    cv2.waitKey(1) 
    x,y,marker,center = find_ball(image) 

            
    focalLength = (2*marker * KNOWN_DISTANCE) / KNOWN_WIDTH 
    print(focalLength)
    ic=img_transform()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("over!")
    cv2.destroyAllWindows()
    
