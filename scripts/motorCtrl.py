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




cm_per_sec_1 = 0
cm_per_sec_2 = 0
rpm_1 = 0
rpm_2 = 0
elapse_1 = 0
elapse_2 = 0
sensor_1 = 15
sensor_2 = 12
pulse_1 = 0
pulse_2 = 0
start_timer_1 = time.time()
start_timer_2 = time.time()
accumul_1 = 0
accumul_2 = 0 
NB_WHEEL_TICK = 40
NB_ACCUMUL = 10

nb_tour = 0

used_1 = 0
used_2 = 0



class MotorCtrl():
    def __init__ (self):
        self.init_GPIO()
        self.init_interrupt()
        self.mh = Adafruit_MotorHAT(addr=0x70) 
        self.myMotor2 = self.mh.getMotor(2)
        self.myMotor1 = self.mh.getMotor(1)    
        self.currentSpeed = 0
	
    def rosNodeStart(self):
        rospy.init_node('motorCtrl', anonymous=False)

        #Subscription to motor control topics	
        rospy.Subscriber('/motor/set_speed', Int32, self.setSpeedCallback)
        rospy.Subscriber('/motor/start', Int32, self.startCallback)
        rospy.Subscriber('/motor/stop', Int32, self.stopCallback)
        rospy.Subscriber('/motor/backward', Int32, self.backwardCallback)
        rospy.Subscriber('/motor/turn_left', Int32, self.leftCallback)
        rospy.Subscriber('/motor/turn_right', Int32, self.rightCallback)


        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()   
	
	
    def init_interrupt(self):
        GPIO.add_event_detect(sensor_1, GPIO.BOTH, callback = self.calculate_elapse_1, bouncetime = 5)
        GPIO.add_event_detect(sensor_2, GPIO.BOTH, callback = self.calculate_elapse_2, bouncetime = 5)

    # recommended for auto-disabling motors on shutdown!
    def turnOffMotors(self):
        mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
        mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)

    def init_GPIO(self):               # initialize GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(sensor_1,GPIO.IN,GPIO.PUD_UP)
        GPIO.setup(sensor_2,GPIO.IN,GPIO.PUD_UP)



    def setSpeed(self, speed):
        self.myMotor1.setSpeed(speed + 0)
        self.myMotor2.setSpeed(speed + 9)


    def turnLeft(self, speed):
        self.myMotor1.setSpeed(speed)
        self.myMotor2.setSpeed(speed)
        self.myMotor1.run(Adafruit_MotorHAT.BACKWARD)
        self.myMotor2.run(Adafruit_MotorHAT.FORWARD)
    
    def turnRight(self, speed):
        self.myMotor1.setSpeed(speed)
        self.myMotor2.setSpeed(speed)
        self.myMotor1.run(Adafruit_MotorHAT.FORWARD)
        self.myMotor2.run(Adafruit_MotorHAT.BACKWARD)

    def moveFwd(self, speed):
        self.myMotor1.setSpeed(speed)
        self.myMotor2.setSpeed(speed)
        self.myMotor1.run(Adafruit_MotorHAT.FORWARD)
        self.myMotor2.run(Adafruit_MotorHAT.FORWARD)

    def moveBkwd(self, speed):
        self.myMotor1.setSpeed(speed)
        self.myMotor2.setSpeed(speed)
        self.myMotor1.run(Adafruit_MotorHAT.BACKWARD)
        self.myMotor2.run(Adafruit_MotorHAT.BACKWARD)

    def moveStop(self):
        self.myMotor1.setSpeed(0)
        self.myMotor2.setSpeed(0)

    #TODO : warning : ambiguous use of speed. 
    #spread across motor control node and robotCore, passing the speed. 
    #To be fixed.
    def startCallback(self, data):
        self.moveFwd(self.currentSpeed)
        rospy.loginfo(rospy.get_caller_id() + ' received command start')
    
    def stopCallback(self, data):
        self.moveStop()
        rospy.loginfo(rospy.get_caller_id() + ' received command stops')

    def backwardCallback(self, data):
        self.moveBkwd(self.currentSpeed)
        rospy.loginfo(rospy.get_caller_id() + ' received command stops')


    def leftCallback(self, data):
        rospy.loginfo(rospy.get_caller_id() + ' received command left')
        self.turnLeft(self.currentSpeed)
    
    def rightCallback(self, data):
        rospy.loginfo(rospy.get_caller_id() + ' received command right')
        self.turnRight(self.currentSpeed)

    def setSpeedCallback(self, data):
        value = data.data
        rospy.loginfo(rospy.get_caller_id() + 'received setspeed command : ' + str(value))
        self.currentSpeed = value
        self.setSpeed(value)
   
    def calculate_elapse_1(self, channel):            # callback function
        global pulse_1, start_timer_1, elapse_1
        global accumul_1, NB_ACCUMUL
        global used_1
        accumul_1+=1
   
        if(accumul_1 <= NB_ACCUMUL):
            pulse_1+=1
        else:
            pulse_1+=1
            accumul_1 = 0
            elapse_1 = time.time() - start_timer_1      # elapse for every 1 wheel tick accumulated made!
            start_timer_1 = time.time()            # let current time equals to start_timer
            used_1 = 0



    def calculate_elapse_2(self, channel):            # callback function
        global pulse_2, start_timer_2, elapse_2
        global accumul_2, NB_ACCUMUL, nb_tour
        global used_2
        accumul_2+=1


   
        #print pulse_2
        if(accumul_2 < NB_ACCUMUL):
            pulse_2+=1
            #print ("skip")
        else:
            pulse_2+=1
            accumul_2 = 0
            elapse_2 = time.time() - start_timer_2      # elapse for every 1 wheel tick accumulated made!
            start_timer_2 = time.time()            # let current time equals to start_timer
            nb_tour +=1
            used_2 = 0
#       print "{0:.3f}".format(elapse_2)


    def calculate_speed(self, r_cm):
        global pulse_1,elapse_1,rpm_1,cm_per_sec_1, NB_WHEEL_TICK
        global pulse_2,elapse_2,rpm_2,cm_per_sec_2, NB_WHEEL_TICK
        global used_1, used_2
        circ_cm = (2*math.pi)*r_cm         # calculate wheel circumference in CM

        if (elapse_1 !=0):                     # to avoid DivisionByZero error
           rpm_1 = (((1/elapse_1) * 60) / NB_WHEEL_TICK) * NB_ACCUMUL  
           if used_1 == 0:
               cm_per_sec_1 = circ_cm / elapse_1      # calculate CM/sec
               used_1 = 1
           else:
               cm_per_sec_1 = cm_per_sec_1 / 2
	  
        if (elapse_2 !=0):                     # to avoid DivisionByZero error
           rpm_2 = ((1/elapse_2 * 60) / NB_WHEEL_TICK) * NB_ACCUMUL  
           if used_2 == 0:
               cm_per_sec_2 = circ_cm / elapse_2      # calculate CM/sec
               used_2 = 1
           else:
               cm_per_sec_2 = cm_per_sec_2/2
      #print (cm_per_sec_2)
 







atexit.register(MotorCtrl.turnOffMotors)



		


    

if __name__ == '__main__':


   myMotorCtrl = MotorCtrl()
   myMotorCtrl.rosNodeStart()

