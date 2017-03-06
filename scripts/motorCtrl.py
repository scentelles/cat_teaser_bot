#!/usr/bin/env python
"""Motor control ROS node"""

import time
import math
import atexit

import rospy
from std_msgs.msg import Int32

from Adafruit_MotorHAT import Adafruit_MotorHAT

import RPi.GPIO as GPIO


NB_WHEEL_TICK = 40
NB_ACCUMUL = 10



class MotorCtrl(object):
    """MotorCtrl class implementing motor control"""
    def __init__(self):
        self.cm_per_sec_1 = 0
        self.cm_per_sec_2 = 0
        self.rpm_1 = 0
        self.rpm_2 = 0
        self.elapse_1 = 0
        self.elapse_2 = 0
        self.sensor_1 = 15
        self.sensor_2 = 12
        self.pulse_1 = 0
        self.pulse_2 = 0
        self.start_timer_1 = time.time()
        self.start_timer_2 = time.time()
        self.accumul_1 = 0
        self.accumul_2 = 0


        self.nb_tour = 0

        self.used_1 = 0
        self.used_2 = 0

        self.init_gpio()
        self.init_interrupt()
        self.motor_handle = Adafruit_MotorHAT(addr=0x70)
        self.motor_2 = self.motor_handle.getMotor(2)
        self.motor_1 = self.motor_handle.getMotor(1)
        self.current_speed = 0



        atexit.register(self.turn_off_motors)



    def ros_node_start(self):
        """Init method, in charge of launching the ros node
	Also subscribes to relevant topics"""
        rospy.init_node('motorCtrl', anonymous=False)

        #Subscription to motor control topics
        rospy.Subscriber('/motor/set_speed', Int32, self.cb_set_speed)
        rospy.Subscriber('/motor/start', Int32, self.cb_start)
        rospy.Subscriber('/motor/stop', Int32, self.cb_stop)
        rospy.Subscriber('/motor/backward', Int32, self.cb_bkwd)
        rospy.Subscriber('/motor/turn_left', Int32, self.cb_left)
        rospy.Subscriber('/motor/turn_right', Int32, self.cb_right)


        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


    def init_interrupt(self):
        """registers interrupt for GPIO events used in speed monitoring"""
        GPIO.add_event_detect(self.sensor_1,
                              GPIO.BOTH,
                              callback=self.cb_calculate_elapse_1,
                              bouncetime=5)
        GPIO.add_event_detect(self.sensor_2,
                              GPIO.BOTH,
                              callback=self.cb_calculate_elapse_2,
                              bouncetime=5)

    # recommended for auto-disabling motors on shutdown!
    def turn_off_motors(self):
        """Turns off motors at electrical level"""
        self.motor_handle.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
        self.motor_handle.getMotor(2).run(Adafruit_MotorHAT.RELEASE)

    def init_gpio(self):               # initialize GPIO
        """Initialize board GPIO"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.sensor_1, GPIO.IN, GPIO.PUD_UP)
        GPIO.setup(self.sensor_2, GPIO.IN, GPIO.PUD_UP)



    def set_speed(self, speed):
        """Method to set motor speed,
	with offset to compensate different spee dat same value"""
        self.motor_1.setSpeed(speed + 0)
        self.motor_2.setSpeed(speed + 9)


    def turn_left(self, speed):
        """Turns left by reverting motor speeds"""
        self.motor_1.setSpeed(speed)
        self.motor_2.setSpeed(speed)
        self.motor_1.run(Adafruit_MotorHAT.BACKWARD)
        self.motor_2.run(Adafruit_MotorHAT.FORWARD)

    def turn_right(self, speed):
        """Turns right by reverting motor speeds"""
        self.motor_1.setSpeed(speed)
        self.motor_2.setSpeed(speed)
        self.motor_1.run(Adafruit_MotorHAT.FORWARD)
        self.motor_2.run(Adafruit_MotorHAT.BACKWARD)

    def move_fwd(self, speed):
        """Moves forward """
        self.motor_1.setSpeed(speed)
        self.motor_2.setSpeed(speed)
        self.motor_1.run(Adafruit_MotorHAT.FORWARD)
        self.motor_2.run(Adafruit_MotorHAT.FORWARD)

    def move_bkwd(self, speed):
        """Moves backward """
        self.motor_1.setSpeed(speed)
        self.motor_2.setSpeed(speed)
        self.motor_1.run(Adafruit_MotorHAT.BACKWARD)
        self.motor_2.run(Adafruit_MotorHAT.BACKWARD)

    def move_stop(self):
        """Stops the motors """
        self.motor_1.setSpeed(0)
        self.motor_2.setSpeed(0)
        self.turn_off_motors()

    #TODO : warning : ambiguous use of speed.
    #spread across motor control node and robotCore, passing the speed.
    #Check also usages in methods above, as the set_speed methods may be useless
    #It means currently there is no offset used to calibrate speed between the two motors
    #To be fixed.
    def cb_start(self, _):
        """Callback used upon ROS topic reception """
        self.move_fwd(self.current_speed)
        rospy.loginfo(rospy.get_caller_id() + ' received command start')

    def cb_stop(self, _):
        """Callback used upon ROS topic reception """
        self.move_stop()
        rospy.loginfo(rospy.get_caller_id() + ' received command stops')

    def cb_bkwd(self, _):
        """Callback used upon ROS topic reception """
        self.move_bkwd(self.current_speed)
        rospy.loginfo(rospy.get_caller_id() + ' received command stops')


    def cb_left(self, _):
        """Callback used upon ROS topic reception """
        rospy.loginfo(rospy.get_caller_id() + ' received command left')
        self.turn_left(self.current_speed)

    def cb_right(self, _):
        """Callback used upon ROS topic reception """
        rospy.loginfo(rospy.get_caller_id() + ' received command right')
        self.turn_right(self.current_speed)

    def cb_set_speed(self, data):
        """Callback used upon ROS topic reception """
        value = data.data
        rospy.loginfo(rospy.get_caller_id() + 'received set_speed command : ' + str(value))
        self.current_speed = value
        self.set_speed(value)


    # callback function
    def cb_calculate_elapse_1(self, channel):
        """callback function for speed sensor 1"""
        self.accumul_1 += 1

        if self.accumul_1 <= NB_ACCUMUL:
            self.pulse_1 += 1
        else:
            self.pulse_1 += 1
            self.accumul_1 = 0
            # elapse for every 1 wheel tick accumulated made!
            self.elapse_1 = time.time() - self.start_timer_1
            # let current time equals to start_timer
            self.start_timer_1 = time.time()
            self.used_1 = 0



    def cb_calculate_elapse_2(self, channel):
        """callback function for speed sensor 2"""
        self.accumul_2 += 1


        #print pulse_2
        if self.accumul_2 < NB_ACCUMUL:
            self.pulse_2 += 1
            #print ("skip")
        else:
            self.pulse_2 += 1
            self.accumul_2 = 0
            # elapse for every 1 wheel tick accumulated made!
            self.elapse_2 = time.time() - self.start_timer_2
            # let current time equals to start_timer
            self.start_timer_2 = time.time()
            self.nb_tour += 1
            self.used_2 = 0


    def calculate_speed(self, r_cm):
        """calculate speed based on sensor inputs """
	#TODO : separate this sensor calculation into a dedicated class
        # calculate wheel circumference in CM
        circ_cm = (2*math.pi)*r_cm

        # to avoid DivisionByZero error
        if self.elapse_1 != 0:
            self.rpm_1 = (((1/self.elapse_1) * 60) / NB_WHEEL_TICK) * NB_ACCUMUL
            if self.used_1 == 0:
                # calculate CM/sec
                self.cm_per_sec_1 = circ_cm / self.elapse_1
                self.used_1 = 1
            else:
                self.cm_per_sec_1 = self.cm_per_sec_1 / 2

        # to avoid DivisionByZero error
        if self.elapse_2 != 0:
            self.rpm_2 = ((1/self.elapse_2 * 60) / NB_WHEEL_TICK) * NB_ACCUMUL
            if self.used_2 == 0:
                # calculate CM/sec
                self.cm_per_sec_2 = circ_cm / self.elapse_2
                self.used_2 = 1
            else:
                self.cm_per_sec_2 = self.cm_per_sec_2/2






if __name__ == '__main__':
    my_motor_ctrl = MotorCtrl()
    my_motor_ctrl.ros_node_start()
