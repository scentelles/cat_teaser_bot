#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32


# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

def camera():
    rospy.loginfo("Starting Camera Node")
    pub = rospy.Publisher('/sensor/camera/face', Int32, queue_size=10)
    rospy.init_node('camera', anonymous=True)

    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (320, 240)
    camera.framerate = 3
    rawCapture = PiRGBArray(camera, size=(320, 240))
 
    # allow the camera to warmup
    time.sleep(0.1)
 
 
    #Load a cascade file for detecting faces
    face_cascade = cv2.CascadeClassifier('/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml')
 


    # capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array
  
        #Convert to grayscale
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
	
        #Look for faces in the image using the loaded cascade file
        faces = face_cascade.detectMultiScale(gray, 1.1, 5)

 
 	nbFaces = len(faces)
	myLog = "Found "+str(nbFaces)+" face(s)"
        rospy.loginfo(myLog)
        pub.publish(nbFaces)

        #Draw a rectangle around every found face
        for (x,y,w,h) in faces:
            cv2.rectangle(image,(x,y),(x+w,y+h),(255,255,0),2)
 
 
	# show the frame
	#cv2.imshow("Frame", image)
	#key = cv2.waitKey(1) & 0xFF 
 
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
 
	# if the `q` key was pressed, break from the loop
	#if key == ord("q"):
	#	break


if __name__ == '__main__':
    try:
        camera()
    except rospy.ROSInterruptException:
        pass



