#!/usr/bin/env python
import paho.mqtt.client as mqtt
from time import sleep
import rospy
import json
from std_msgs.msg import String

cam1_pub = rospy.Publisher("camera_right", String, queue_size = 2)
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    # client.subscribe("camera1_data")
    client.subscribe("right_line") 

def on_message(client, userdata, msg): 
    data = msg.payload.decode()
    try:
    	# print data
    	# client.publish("feedback", "got message")
    	if msg.topic == "right_line":
    		info = json.loads(data)
    		# print (info["line"])
    		msg = String()
    		msg.data = data
    		cam1_pub.publish(msg)
    except Exception as e :
    	client.publish("feedback", "broken message")
    	print e


def cam_callback(data):
	info = json.loads(data.data)
	#print info["line"][0]


if __name__ == "__main__":
	rospy.init_node('camera_1_data')
	client = mqtt.Client()
	client.connect("192.168.44.112",1883,60) 
	client.on_connect = on_connect
	client.on_message = on_message
	client.loop_start()
	rospy.Subscriber("/camera_right", String, cam_callback)
	while not rospy.is_shutdown():
		try:
			# client.publish("feedback", "got message")
			sleep(1)
		except KeyboardInterrupt:
			print("bie")
			client.disconnect()
			break

