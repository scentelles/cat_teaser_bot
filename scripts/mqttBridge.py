import paho.mqtt.client as mqtt
import rospy
from std_msgs.msg import Int32

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc) + "\n")

def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload) + "\n")
    

def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload)+"\n")
    if msg.topic == "robot/start":
        rospy.loginfo(msg.topic)
	#if msg.payload == "2":
	print "trigger robot  start"
        pub.publish(command)



def mqttBridge():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message


    client.connect("localhost")

    client.subscribe("robot/start")
    client.subscribe("robot/stop")

    pubStart = rospy.Publisher('/robot/start', Int32, queue_size=10)
    pubStop  = rospy.Publisher('/robot/stop', Int32, queue_size=10)

    rospy.init_node('mqtt_bridge', anonymous=False)

    client.loop_start()


if __name__ == '__main__':
    try:
        mqttBridge()
    except rospy.ROSInterruptException:
        pass

