import time
import BaseHTTPServer

import paho.mqtt.client as mqtt


import re
HOST_NAME = 'localhost' # !!!REMEMBER TO CHANGE THIS!!!
PORT_NUMBER = 12345 # Maybe set this to 9000.

MQTT_ADDRESS = "192.168.4.2"

value= 0
client = 0

global currentDistance
class MyHandler(BaseHTTPServer.BaseHTTPRequestHandler):
    def do_HEAD(s):
        s.send_response(200)
        s.send_header("Content-type", "text/html")
        s.end_headers()

		
    def do_GET(s):
        """Respond to a GET request."""
        global value
        global client
        print "Get request received"
        s.send_response(200)
        s.send_header("Content-type", "text/html")
        s.end_headers()
        
        if (s.path == "/poll"):
            value += 1
            s.wfile.write("test " + str(value) + "\n")
            s.wfile.write("distance " + str(currentDistance) + "\n")
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
    
  
if __name__ == '__main__':
    print "toto\n"
    server_class = BaseHTTPServer.HTTPServer
    httpd = server_class((HOST_NAME, PORT_NUMBER), MyHandler)
    print time.asctime(), "Server Starts - %s:%s" % (HOST_NAME, PORT_NUMBER)
	
    global client
    global currentDistance
    currentDistance = 0
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(MQTT_ADDRESS)

    client.subscribe("sensor/distance")    

    client.loop_start()

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    httpd.server_close()
    print time.asctime(), "Server Stops - %s:%s" % (HOST_NAME, PORT_NUMBER)