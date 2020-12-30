# DataClient2.py
#!/usr/bin/env python
import rospy
from tcpcom import TCPClient
import time
from std_msgs.msg import String,Float32

IP_ADDRESS = "192.168.0.159"
IP_PORT = 22000

dir_string=""

def callback2(msg):
	global dir_string
	global call
	dir_string = msg.data;
	print "Values sent"
	client.sendMessage(dir_string)

def onStateChanged(state, message):
    global isConnected
    if state == "CONNECTING":
       print "Client:-- Waiting for connection..."
    elif state == "CONNECTED":
       print "Client:-- Connection estabished."
    elif state == "DISCONNECTED":
       print "Client:-- Connection lost."
       isConnected = False
    elif state == "MESSAGE":
       print "Client:-- Received data:", message
       pub1.publish(message)

rospy.init_node('client', anonymous=True)
rospy.Subscriber('robo_command', String, callback2)
pub1 = rospy.Publisher('take_pic',String, queue_size=10)
client = TCPClient(IP_ADDRESS, IP_PORT, stateChanged = onStateChanged)
rc = client.connect()
if rc:
    isConnected = True
    while isConnected:
			#dir_string = raw_input("Robot params ")

		if dir_string == "q":
			client.disconnect()
			break

           
else:
    print "Client:-- Connection failed"  