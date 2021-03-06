#!/usr/bin/env python
# Software License Agreement (BSD License)

import paho.mqtt.client as mqtt
import rospy
from std_msgs.msg import Int32

global pubStart, pubStop, pubLeft, pubRight, pubSetSpeed
global client

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc) + "\n")


def on_message(client, userdata, msg):
    global pubStart, pubStop, pubLeft, pubRight, pubSetSpeed, pubSetSpeed
    tmpString = msg.topic+" "+str(msg.payload)+"\n"
    rospy.loginfo(tmpString)
    if msg.topic == "robot/start":
        rospy.loginfo(msg.topic)
	#if msg.payload == "2":
	rospy.loginfo("trigger robot  start")
        pubStart.publish(1)

    if msg.topic == "robot/stop":
        rospy.loginfo(msg.topic)
	#if msg.payload == "2":
	rospy.loginfo("trigger robot  stop")
        pubStop.publish(1)

    if msg.topic == "robot/left":
        rospy.loginfo(msg.topic)
	rospy.loginfo("trigger robot  left")
        pubLeft.publish(1)

    if msg.topic == "robot/right":
        rospy.loginfo(msg.topic)
	rospy.loginfo("trigger robot  right")
        pubRight.publish(1)

    if msg.topic == "robot/setspeed":
        newSpeed = int(msg.payload)
	rospy.loginfo(msg.topic)
	rospy.loginfo("trigger set speed :")
        rospy.loginfo(newSpeed)
	pubSetSpeed.publish(newSpeed)


def updateDistanceCallback(data):
    global client
    client.publish("sensor/distance", payload=int(data.data), qos=0, retain=False)

def mqttBridge():
    global pubStart, pubStop, pubLeft, pubRight, pubSetSpeed
    global client
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message


    client.connect("localhost")

    client.subscribe("robot/start")
    client.subscribe("robot/stop")
    client.subscribe("robot/left")
    client.subscribe("robot/right")
    client.subscribe("robot/setspeed")

    pubStart = rospy.Publisher('/robot/start', Int32, queue_size=10)
    pubStop  = rospy.Publisher('/robot/stop', Int32, queue_size=10)
    pubLeft  = rospy.Publisher('/robot/left', Int32, queue_size=10)
    pubRight = rospy.Publisher('/robot/right', Int32, queue_size=10)
    pubSetSpeed = rospy.Publisher('/robot/setspeed', Int32, queue_size=10)

    rospy.init_node('mqtt_bridge', anonymous=False)

    rospy.Subscriber('/sensor/distance', Int32, updateDistanceCallback)


    client.loop_start()

    rospy.spin()

if __name__ == '__main__':
    try:
        mqttBridge()
    except rospy.ROSInterruptException:
        pass

