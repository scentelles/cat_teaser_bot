#!/usr/bin/env python
"""The Core of the robot.
This module is in charge of the decision of all robot moves and actions
based on sensors inputs or external commands"""
import threading
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String


CONTROL_MODE_EXTERNAL = 1
CONTROL_MODE_INTERNAL = 2
STOPPED = 3
BKWD = 4

class RobotCore(object):
    """RobotCore class implementing the brain of the robot"""

    def __init__(self):
        #TODO : manage control mode
        self.mode = CONTROL_MODE_INTERNAL
        self.move_status = STOPPED
        self.current_speed = 0
        self.heading_change_in_progress = 0

        self.ros_pub_motor_set_speed = None
        self.ros_pub_motor_start = None
        self.ros_pub_motor_stop = None
        self.ros_pub_motor_bkwd = None
        self.ros_pub_motor_left = None
        self.ros_pub_motor_right = None
        self.pub_audio = None


    def ros_node_start(self):
        """Init method, in charge of launching the ros node"""

        rospy.init_node('robotCore', anonymous=False)

        #ROS publisher for speech commands
        rospy.Subscriber('/speech/command', String, self.cb_vocal_command)

        #ROS publisher for sensors
        rospy.Subscriber('/sensor/distance/front', Int32, self.cb_distance)
        rospy.Subscriber('/sensor/camera/face', Int32, self.cb_face_detected)

        #ROS subriptions for external control topics
        rospy.Subscriber('/robot/stop', Int32, self.cb_stop)
        rospy.Subscriber('/robot/start', Int32, self.cb_start)
        rospy.Subscriber('/robot/left', Int32, self.cb_left)
        rospy.Subscriber('/robot/right', Int32, self.cb_right)
        rospy.Subscriber('/robot/set_speed', Int32, self.cb_set_speed)


        #ROS publisher for audio rendering topics
        self.pub_audio = rospy.Publisher('/audio/command', String, queue_size=1)

        #ROS publishers for motor control topics
        self.ros_pub_motor_set_speed = rospy.Publisher('/motor/set_speed', Int32, queue_size=1)
        self.ros_pub_motor_start = rospy.Publisher('/motor/start', Int32, queue_size=1)
        self.ros_pub_motor_stop = rospy.Publisher('/motor/stop', Int32, queue_size=1)
        self.ros_pub_motor_bkwd = rospy.Publisher('/motor/backward', Int32, queue_size=1)
        self.ros_pub_motor_left = rospy.Publisher('/motor/turn_left', Int32, queue_size=1)
        self.ros_pub_motor_right = rospy.Publisher('/motor/turn_right', Int32, queue_size=1)


        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def set_control_mode(self, mode):
        """Method to set control mode
	- internal : the robot will have autonomous behavior
	- external : the robot will be driven by external commands received
	             thru dedicated ROS topics"""
        self.mode = mode

    def set_speed(self, speed):
        """Sets the absolute speed of the robot
	and triggers corresponding actuators commands"""
        self.ros_pub_motor_set_speed.publish(speed)
        self.current_speed = speed

    def turn_left(self, speed):
        """Triggers actuators commands to turn left"""
        self.ros_pub_motor_left.publish(speed)

    def turn_right(self, speed):
        """Triggers actuators commands to turn right"""
        self.ros_pub_motor_right.publish(speed)

    def move_fwd(self, speed):
        """Triggers actuators commands to move forward"""
        self.ros_pub_motor_start.publish(speed)

    def move_bkwd(self, speed):
        """Triggers actuators commands to move backward"""
        self.move_status = BKWD
        self.ros_pub_motor_bkwd.publish(speed)

    def move_stop(self):
        """Triggers actuators commands to full stop the robot"""
        self.move_status = STOPPED
        self.ros_pub_motor_stop.publish(1)


    #Callback definitions. Used for external control topics
    #TODO : manage all types of moves in a single callback
    def cb_start(self, data):
        """Callback triggered upon ROS topic message received"""
        rospy.loginfo(rospy.get_caller_id() + 'I received command start')
        self.move_fwd(self.current_speed)

    def cb_stop(self, data):
        """Callback triggered upon ROS topic message received"""
        rospy.loginfo(rospy.get_caller_id() + 'I received command stops')
        self.move_stop()

    def cb_left(self, data):
        """Callback triggered upon ROS topic message received"""
        rospy.loginfo(rospy.get_caller_id() + 'I received command left')
        self.turn_left(self.current_speed)

    def cb_right(self, data):
        """Callback triggered upon ROS topic message received"""
        rospy.loginfo(rospy.get_caller_id() + 'I received command right')
        self.turn_right(self.current_speed)

    def cb_set_speed(self, data):
        """Callback triggered upon ROS topic message received"""
        value = data.data
        rospy.loginfo(rospy.get_caller_id() + 'Received set_speed command : ' + str(value))
        self.current_speed = value
        self.set_speed(self.current_speed)

    def cb_face_detected(self, data):
        """Callback triggered upon ROS topic message received"""
        rospy.loginfo(rospy.get_caller_id() + 'Camera detected faces : %i', data.data)
        if self.heading_change_in_progress == 0:
            if data.data > 0:
                #TODO : fix hardcoded change heading types
                self.change_heading(2)
                self.pub_audio.publish("EPIC")

    def cb_distance(self, data):
        """Callback triggered upon ROS topic message received"""
        distance = data.data
        if self.heading_change_in_progress == 0:
            if distance < 20:
                self.change_heading(1)

    def cb_vocal_command(self, data):
        """Callback triggered upon ROS topic message received"""
        command = data.data
        rospy.loginfo(rospy.get_caller_id() + 'Received vocal command : ' + command)
        if command == "start":
        #TODO remove hardcoded speed
            self.move_fwd(80)
        if command == "stop":
            self.move_stop()
        if command == "smooth":
            self.smooth_dance()

    def worker(self, num):
        """Body of the change_heading thread"""

        self.heading_change_in_progress = 1
        rospy.loginfo(rospy.get_caller_id() + ' Changing heading')
        if num == 1:
            self.ros_pub_motor_bkwd.publish(self.current_speed)
            rospy.sleep(1)
            self.ros_pub_motor_left.publish(self.current_speed)
            rospy.sleep(1.5)
            self.ros_pub_motor_start.publish(self.current_speed)

        #Go away from face!!!!
        if num == 2:
            self.ros_pub_motor_stop.publish()
            rospy.sleep(1)
            self.ros_pub_motor_set_speed.publish(255)
            self.ros_pub_motor_bkwd.publish(255)
            rospy.sleep(1)

        self.heading_change_in_progress = 0

    def change_heading(self, behavior):
        """method in charge of spawning a dedicated thread for heading change"""
        my_thread = threading.Thread(target=self.worker, args=(behavior,))
        my_thread.start()


    def smooth_dance(self):
        """Methods implementing the series of moves for a smooth danse !"""
        rospy.sleep(1.3)
        self.move_fwd(150)
        rospy.sleep(0.5)
        self.move_bkwd(150)
        rospy.sleep(0.5)

        self.turn_left(180)
        rospy.sleep(0.3)
        self.turn_right(180)
        rospy.sleep(0.3)
        self.turn_left(210)
        rospy.sleep(1)
        self.turn_left(150)
        rospy.sleep(0.3)
        self.move_fwd(150)
        rospy.sleep(0.3)
        self.turn_right(180)
        rospy.sleep(1)
        self.move_fwd(150)
        rospy.sleep(0.5)
        self.move_bkwd(150)
        rospy.sleep(0.5)
        self.move_fwd(150)
        rospy.sleep(0.5)
        self.move_bkwd(150)
        rospy.sleep(0.5)

        self.turn_left(180)
        rospy.sleep(0.3)
        self.turn_right(180)
        rospy.sleep(0.3)
        self.turn_left(180)
        rospy.sleep(1)
        self.turn_left(150)
        rospy.sleep(0.3)
        self.turn_right(150)
        rospy.sleep(0.3)
        self.turn_right(180)
        rospy.sleep(1.5)
        self.move_fwd(150)
        rospy.sleep(0.5)
        self.move_bkwd(150)
        rospy.sleep(1)
        self.move_fwd(150)
        rospy.sleep(0.3)
        self.move_bkwd(150)
        rospy.sleep(0.5)

        self.turn_left(180)
        rospy.sleep(0.3)
        self.move_bkwd(180)
        rospy.sleep(0.3)
        self.turn_right(210)
        rospy.sleep(1)
        self.turn_left(150)
        rospy.sleep(0.3)
        self.turn_right(150)
        rospy.sleep(0.3)
        self.turn_left(200)
        rospy.sleep(1)
        self.move_fwd(150)
        rospy.sleep(0.5)
        self.move_bkwd(150)
        rospy.sleep(0.5)
        self.move_fwd(150)
        rospy.sleep(0.5)
        self.move_bkwd(150)
        rospy.sleep(0.5)

        self.turn_left(180)
        rospy.sleep(0.3)
        self.move_bkwd(180)
        rospy.sleep(0.3)
        self.turn_right(210)
        rospy.sleep(1)
        self.turn_left(150)
        rospy.sleep(0.3)
        self.turn_right(150)
        rospy.sleep(0.3)
        self.turn_left(200)
        rospy.sleep(1)
        self.move_fwd(150)
        rospy.sleep(0.5)
        self.move_bkwd(150)
        rospy.sleep(0.5)
        self.move_fwd(150)
        rospy.sleep(0.5)
        self.move_bkwd(150)
        rospy.sleep(0.5)

        self.move_stop()


if __name__ == '__main__':
    my_robot = RobotCore()
    my_robot.ros_node_start()
