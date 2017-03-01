#!/usr/bin/env python
# Software License Agreement (BSD License)


import rospy
from std_msgs.msg import Int32

def talker():
    pub = rospy.Publisher('/motor/command', Int32, queue_size=10)
    rospy.init_node('motor_ramp', anonymous=True)
    rate = rospy.Rate(100) # 30hz
    command = 0
    while not rospy.is_shutdown():
         
        hello_str = "ramp %i" % command
        rospy.loginfo(hello_str)
        pub.publish(command)
	if(command > 300):
	    command = 0
	else:
	    command += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
