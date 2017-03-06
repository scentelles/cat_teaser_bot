import socket
import time
import BaseHTTPServer

import paho.mqtt.client as mqtt

from threading import Thread

import re
HOST_NAME = 'localhost' # !!!REMEMBER TO CHANGE THIS!!!
PORT_NUMBER = 12345 # Maybe set this to 9000.

MQTT_ADDRESS = "192.168.4.2"

client = 0

global currentDistance
class MyHandler(BaseHTTPServer.BaseHTTPRequestHandler):
    def do_HEAD(s):
        s.send_response(200)
        s.send_header("Content-type", "text/html")
        s.end_headers()

		
    def do_GET(s):
        """Respond to a GET request."""
        global client
        global robotIsConnected
        print "Get request received"
        s.send_response(200)
        s.send_header("Content-type", "text/html")
        s.end_headers()
        
        if (s.path == "/poll"):
            if(robotIsConnected):
                s.wfile.write("distance " + str(currentDistance) + "\n")
            else:
                s.wfile.write("_problem The robot is not connected\n")
            
        if (s.path == "/start"):
            client.publish("robot/start", payload='1', qos=0, retain=False)

        if (s.path == "/stop"):
            client.publish("robot/stop", payload='1', qos=0, retain=False)

        if (s.path == "/left"):
            client.publish("robot/left", payload='1', qos=0, retain=False)

        if (s.path == "/right"):
            client.publish("robot/right", payload='1', qos=0, retain=False)		
        
        match = re.search(r'setspeed/(\d+)', s.path)
        if match:
            
            client.publish("robot/setspeed", payload=int(match.group(1)), qos=0, retain=False)

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc) + "\n")

def on_message(client, userdata, msg):
    global currentDistance
    print(msg.topic+" "+str(msg.payload) + "\n")
    
    if msg.topic == "sensor/distance":
        currentDistance = int(msg.payload)

def on_disconnect(client, userdata, rc):
    global robotIsConnected
    print "Disconnected from MQTT server with code: %s" % rc
    robotIsConnected = False
    while rc != 0:
        time.sleep(5)
        print "Reconnecting..."
        try:
            rc = client.reconnect()
            robotIsConnected = True
        except socket.error:
            "Could not connect to Mqtt server\n"
def mqttClientBody(mqttAddress):
    global client, robotIsConnected
    robotIsConnected = False
    client = mqtt.Client(client_id="ScratchMqtt", clean_session=False)
    client.on_connect    = on_connect
    client.on_message    = on_message
    client.on_disconnect = on_disconnect
    
    while(robotIsConnected == False):
        try:
            client.connect(MQTT_ADDRESS)
            robotIsConnected = True
        except socket.error:
            print "Connection to MQTT on robot side failed. Please check the MQTT the robot address is correct, and the bridge is running.\n"
            robotIsConnected = False
    
    client.subscribe("sensor/distance")    

    client.loop_start()

    
if __name__ == '__main__':
    print "toto\n"
    server_class = BaseHTTPServer.HTTPServer
    httpd = server_class((HOST_NAME, PORT_NUMBER), MyHandler)
    print time.asctime(), "Server Starts - %s:%s" % (HOST_NAME, PORT_NUMBER)
	
    global client
    global currentDistance
    currentDistance = 0

    # start MQTT client in separate thread to manage reconnection loop 
    mqttClientThread = Thread(target=mqttClientBody, args=(MQTT_ADDRESS,))
    mqttClientThread.start()

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    httpd.server_close()
    print time.asctime(), "Server Stops - %s:%s" % (HOST_NAME, PORT_NUMBER)