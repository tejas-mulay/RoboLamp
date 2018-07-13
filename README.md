# RoboLamp
A robo-lamp is a robotic arm with three degree of freedom whose end effector is LED array along with a webcam. The arm is intended to follow the object by performing the necessary movements so that the light is focused at that object.  
The video of the object was captured using a webcam and then it was processed with OpenCV. This processes include motion detection, image thresholding and finding the centroid of that particular object. So according to the coordinates acquired (centroid) the arm was supposed to change its position.
