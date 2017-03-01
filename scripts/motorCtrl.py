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

# create a default object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT(addr=0x70)

global myMotor1, myMotor2
global headingChangeInProgress
global currentSpeed
global pubAudio
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


def init_GPIO():               # initialize GPIO
   GPIO.setmode(GPIO.BCM)
   GPIO.setwarnings(False)
   GPIO.setup(sensor_1,GPIO.IN,GPIO.PUD_UP)
   GPIO.setup(sensor_2,GPIO.IN,GPIO.PUD_UP)

   
def calculate_elapse_1(channel):            # callback function
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



def calculate_elapse_2(channel):            # callback function
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


def calculate_speed(r_cm):
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
 

def init_interrupt():
   GPIO.add_event_detect(sensor_1, GPIO.BOTH, callback = calculate_elapse_1, bouncetime = 5)
   GPIO.add_event_detect(sensor_2, GPIO.BOTH, callback = calculate_elapse_2, bouncetime = 5)




# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
    mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)


atexit.register(turnOffMotors)

def setSpeed(speed):
    myMotor1.setSpeed(speed + 0)
    myMotor2.setSpeed(speed + 9)



def callback(data):
    global myMotor1, myMotor2, currentSpeed
    rospy.loginfo(rospy.get_caller_id() + 'I received command %s', data.data)
    setSpeed(data.data)
    currentSpeed = data.data
    
    


def faceDetectedCallback(data):
    global headingChangeInProgress
    rospy.loginfo(rospy.get_caller_id() + 'Camera detected faces : %i', data.data)
    if(headingChangeInProgress == 0):
        if(data.data > 0):
	    changeHeading(2)
	    pubAudio.publish("EPIC")


def worker(num):
    global headingChangeInProgress, currentSpeed
    headingChangeInProgress = 1
    
    if(num == 1):
        setSpeed(100)
        myMotor1.run(Adafruit_MotorHAT.BACKWARD)
        myMotor2.run(Adafruit_MotorHAT.BACKWARD)
        rospy.sleep(1)
        myMotor1.run(Adafruit_MotorHAT.FORWARD)
        rospy.sleep(1.5)    
        myMotor2.run(Adafruit_MotorHAT.FORWARD)
	setSpeed(currentSpeed)

 
    #Go away from face!!!!
    if(num == 2):
        myMotor1.setSpeed(0)
        myMotor2.setSpeed(0)
        rospy.sleep(1)	
        myMotor1.setSpeed(255)
        myMotor2.setSpeed(255)
        myMotor1.run(Adafruit_MotorHAT.BACKWARD)
        myMotor2.run(Adafruit_MotorHAT.BACKWARD)
        rospy.sleep(1) 
	setSpeed(100)
        myMotor1.run(Adafruit_MotorHAT.FORWARD)	
        rospy.sleep(0.5) 
        myMotor2.run(Adafruit_MotorHAT.FORWARD)
        setSpeed(currentSpeed)

	
    headingChangeInProgress = 0 

def changeHeading(behavior):
    t = threading.Thread(target=worker, args=(behavior,))
    t.start()
  

def distanceCallback(data):
    global headingChangeInProgress
    distance = data.data
    if(headingChangeInProgress == 0):
        if (distance < 20):
	    changeHeading(1)


def motorCtrl():
    global pubAudio
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    global headingChangeInProgress, currentSpeed
    headingChangeInProgress = 0
    currentSpeed = 0
    rospy.init_node('motorCtrl', anonymous=False)
    setSpeed(80)
    currentSpeed = 80

    rospy.Subscriber('/motor/command', Int32, callback)
    rospy.Subscriber('/camera/face', Int32, faceDetectedCallback)
    rospy.Subscriber('/sensor/distance', Int32, distanceCallback)

    pubAudio = rospy.Publisher('/audio/command', String, queue_size=1)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
   init_GPIO()
   init_interrupt()
   global myMotor1, myMotor2
   myMotor2 = mh.getMotor(2)

   myMotor1 = mh.getMotor(1)
   myMotor1.run(Adafruit_MotorHAT.FORWARD)
   myMotor2.run(Adafruit_MotorHAT.FORWARD)

   motorCtrl()
