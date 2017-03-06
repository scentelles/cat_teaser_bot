#!/usr/bin/env python
import RPi.GPIO as GPIO
from time import sleep
import time, math
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import atexit

import threading

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String


CONTROL_MODE_EXTERNAL = 1
CONTROL_MODE_INTERNAL = 2
STOPPED = 3
BKWD = 4

class robotCore():

	
    def setControlMode(mode):
        self.mode = mode
        

    def setSpeed(self, speed):
        self.pubMotorSetSpeed.publish(speed)


    def turnLeft(self, speed):
        self.pubMotorTurnLeft.publish(speed)

    def turnRight(self, speed):
        self.pubMotorTurnRight.publish(speed)

    def moveFwd(self, speed):
        self.pubMotorStart.publish(speed)

    def moveBkwd(self, speed):
        self.moveStatus = BKWD
	self.pubMotorBackward.publish(speed)
	
    def moveStop(self):
        self.moveStatus = STOPPED
        self.pubMotorStop.publish(1)


    #Callback definitions. Used for external control topics    
    #TODO : manage all types of moves in a single callback
    def startCallback(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I received command start')
        self.moveFwd(self.currentSpeed)
    
    def stopCallback(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I received command stops')
        self.moveStop()

    def leftCallback(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I received command left')
        self.turnLeft(self.currentSpeed)
    
    def rightCallback(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I received command right')
        self.turnRight(self.currentSpeed)

    def setSpeedCallback(self, data):
        value = data.data
        rospy.loginfo(rospy.get_caller_id() + 'Received setspeed command : ' + str(value))
        self.currentSpeed = value
        self.setSpeed(self.currentSpeed)
	
    def __init__ (self):
        self.mode = CONTROL_MODE_INTERNAL
        self.moveStatus = STOPPED
	self.currentSpeed = 0
        self.headingChangeInProgress = 0



    def rosNodeStart(self):
        rospy.init_node('robotCore', anonymous=False)


        rospy.Subscriber('/speech/command', String, self.vocalCommandCallback)
#TODO : unused?        rospy.Subscriber('/motor/command', Int32, self.callback)

        rospy.Subscriber('/camera/face', Int32, self.faceDetectedCallback)

        #ROS publisher for sensors
        rospy.Subscriber('/sensor/distance', Int32, self.distanceCallback)

        #ROS subriptions for external control topics
        rospy.Subscriber('/robot/stop', Int32, self.stopCallback)
        rospy.Subscriber('/robot/start', Int32, self.startCallback)
        rospy.Subscriber('/robot/left', Int32, self.leftCallback)
        rospy.Subscriber('/robot/right', Int32, self.rightCallback)
        rospy.Subscriber('/robot/setspeed', Int32, self.setSpeedCallback)


        #ROS publisher for audio rendering topics
        self.pubAudio = rospy.Publisher('/audio/command', String, queue_size=1)

        #ROS publishers for motor control topics
        self.pubMotorSetSpeed  = rospy.Publisher('/motor/set_speed', Int32, queue_size=1)
        self.pubMotorStart     = rospy.Publisher('/motor/start', Int32, queue_size=1)
        self.pubMotorStop      = rospy.Publisher('/motor/stop', Int32, queue_size=1)
        self.pubMotorBackward  = rospy.Publisher('/motor/backward', Int32, queue_size=1)
        self.pubMotorTurnLeft  = rospy.Publisher('/motor/turn_left', Int32, queue_size=1)
        self.pubMotorTurnRight = rospy.Publisher('/motor/turn_right', Int32, queue_size=1)


        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


    def faceDetectedCallback(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'Camera detected faces : %i', data.data)
        if(self.headingChangeInProgress == 0):
            if(data.data > 0):
	        self.changeHeading(2)
	        self.pubAudio.publish("EPIC")


    def worker(self, num):
        self.headingChangeInProgress = 1
        rospy.loginfo(rospy.get_caller_id() + ' Changing heading')    
        if(num == 1):
            self.pubMotorBackward.publish(self.currentSpeed)
            rospy.sleep(1)
            self.pubMotorTurnLeft.publish(self.currentSpeed)
            rospy.sleep(1.5)    
            self.pubMotorStart.publish(self.currentSpeed)
	    

 
        #Go away from face!!!!
        if(num == 2):
            self.pubMotorStop.publish()
            rospy.sleep(1)	
            self.pubMotorSetSpeed.publish(255)
            self.pubMotorBackward.publish(255)
            rospy.sleep(1) 

	
        self.headingChangeInProgress = 0 

    def changeHeading(self, behavior):
        t = threading.Thread(target=self.worker, args=(behavior,))
        t.start()
  

    def distanceCallback(self, data):
        distance = data.data
        if(self.headingChangeInProgress == 0):
            if (distance < 20):
	        self.changeHeading(1)


    def smoothDance(self):
        rospy.sleep(1.3)
        self.moveFwd(150)     
        rospy.sleep(0.5)
        self.moveBkwd(150)
        rospy.sleep(0.5)
  
        self.turnLeft(180)
        rospy.sleep(0.3) 
        self.turnRight(180)
        rospy.sleep(0.3) 
        self.turnLeft(210)
        rospy.sleep(1)     
        self.turnLeft(150)
        rospy.sleep(0.3) 
        self.moveFwd(150)
        rospy.sleep(0.3) 
        self.turnRight(180)
        rospy.sleep(1)
        self.moveFwd(150)     
        rospy.sleep(0.5)
        self.moveBkwd(150)
        rospy.sleep(0.5)
        self.moveFwd(150)     
        rospy.sleep(0.5)
        self.moveBkwd(150)
        rospy.sleep(0.5)

        self.turnLeft(180)
        rospy.sleep(0.3) 
        self.turnRight(180)
        rospy.sleep(0.3) 
        self.turnLeft(180)
        rospy.sleep(1)     
        self.turnLeft(150)
        rospy.sleep(0.3) 
        self.turnRight(150)
        rospy.sleep(0.3) 
        self.turnRight(180)
        rospy.sleep(1.5)
        self.moveFwd(150)     
        rospy.sleep(0.5)
        self.moveBkwd(150)
        rospy.sleep(1)
        self.moveFwd(150)     
        rospy.sleep(0.3)
        self.moveBkwd(150)
        rospy.sleep(0.5)

        self.turnLeft(180)
        rospy.sleep(0.3) 
        self.moveBkwd(180)
        rospy.sleep(0.3) 
        self.turnRight(210)
        rospy.sleep(1)     
        self.turnLeft(150)
        rospy.sleep(0.3) 
        self.turnRight(150)
        rospy.sleep(0.3) 
        self.turnLeft(200)
        rospy.sleep(1)
        self.moveFwd(150)     
        rospy.sleep(0.5)
        self.moveBkwd(150)
        rospy.sleep(0.5)
        self.moveFwd(150)     
        rospy.sleep(0.5)
        self.moveBkwd(150)
        rospy.sleep(0.5)
    
    
        self.turnLeft(180)
        rospy.sleep(0.3) 
        self.moveBkwd(180)
        rospy.sleep(0.3) 
        self.turnRight(210)
        rospy.sleep(1)     
        self.turnLeft(150)
        rospy.sleep(0.3) 
        self.turnRight(150)
        rospy.sleep(0.3) 
        self.turnLeft(200)
        rospy.sleep(1)
        self.moveFwd(150)     
        rospy.sleep(0.5)
        self.moveBkwd(150)
        rospy.sleep(0.5)
        self.moveFwd(150)     
        rospy.sleep(0.5)
        self.moveBkwd(150)
        rospy.sleep(0.5)

    
        self.moveStop()
	    
    def vocalCommandCallback(data):
        command = data.data
        rospy.loginfo(rospy.get_caller_id() + 'Received vocal command : ' + command)
        if(command == "start"):
        #TODO remove hardcoded speed
            self.moveFwd(80)
        if(command == "stop"):
            self.moveStop()
        if(command == "smooth"):
            self.smoothDance()

	

if __name__ == '__main__':
   myRobot = robotCore()
   myRobot.rosNodeStart()

