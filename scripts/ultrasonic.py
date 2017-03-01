#!/usr/bin/env python

import RPi.GPIO as GPIO                    #Import GPIO library
import time                                #Import time library
GPIO.setmode(GPIO.BCM)                     #Set GPIO pin numbering 


import rospy
from std_msgs.msg import Int32

TRIG = 21                                  #Associate pin 23 to TRIG
ECHO = 20                                  #Associate pin 24 to ECHO

rospy.loginfo("Distance measurement in progress")

GPIO.setup(TRIG,GPIO.OUT)                  #Set pin as GPIO out
GPIO.setup(ECHO,GPIO.IN)                   #Set pin as GPIO in



def ultrasonic():
    pub = rospy.Publisher('/sensor/distance', Int32, queue_size=1)
    rospy.init_node('ultrasonic', anonymous=True)
    rate = rospy.Rate(4)
    
    while not rospy.is_shutdown():

      GPIO.output(TRIG, False)                 #Set TRIG as LOW
      rate.sleep()                        

      GPIO.output(TRIG, True)                  #Set TRIG as HIGH
      time.sleep(0.00001)                      #Delay of 0.00001 seconds
      GPIO.output(TRIG, False)                 #Set TRIG as LOW

      while GPIO.input(ECHO)==0:               #Check whether the ECHO is LOW
        pulse_start = time.time()              #Saves the last known time of LOW pulse

      while GPIO.input(ECHO)==1:               #Check whether the ECHO is HIGH
        pulse_end = time.time()                #Saves the last known time of HIGH pulse 

      pulse_duration = pulse_end - pulse_start #Get pulse duration to a variable

      distance = pulse_duration * 17150        #Multiply pulse duration by 17150 to get distance
      distance = round(distance, 2)            #Round to two decimal points

      if distance > 2 and distance < 400:      #Check whether the distance is within range
        logString = "Distance : %i" % distance
	pub.publish(distance)
	rospy.loginfo(logString)  
    
      else:
        rospy.loginfo("Out Of Range")                   #display out of range
    
    
if __name__ == '__main__':
    try:
        ultrasonic()
    except rospy.ROSInterruptException:
        pass