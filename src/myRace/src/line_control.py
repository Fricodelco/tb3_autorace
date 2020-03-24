#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from time import sleep
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Vector3
pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
integral = 0
move_flag = True
first_time_flag = True
choose_line = [0,0,1]
def cbError(error_msg):
	global first_time_flag
	global integral, move_flag
	error = error_msg.x
	dif_error = error_msg.y
	if(move_flag == True):
		velocity = Twist()
		proportional = 0.01*error #0.07
		differential = 0.023*dif_error #0.0015
		integral = integral + 0.0001*error #0.0001
		up = proportional+differential+integral
		velocity.angular.z = up
		velocity.linear.x = 0.22 - 0.07*abs(up)#0.22 - 0.05*abs(up) #0.15
		print("proportional:"+str(float('{:.2f}'.format(proportional)))+" differential:"+str(float('{:.2f}'.format(differential)))+" integral:"+str(float('{:.2f}'.format(integral))))
		pub_vel.publish(velocity)
	elif(first_time_flag == True):
		# velocity = Twist()
		# velocity.angular.z = 0
		# velocity.linear.x = 0
		# for i in range(0,2,1):
			# pub_vel.publish(velocity)
			# rospy.sleep(0.01)
		first_time_flag = False

def cb_flag(data):
	global move_flag, first_time_flag
	move_flag = data.data
	first_time_flag = True


if __name__ == '__main__':
	rospy.init_node('line_control')
	sub_line = rospy.Subscriber('error_line', Vector3, cbError, queue_size=1)
	sub_move_flag = rospy.Subscriber('line_move_flag', Bool, cb_flag, queue_size=1)
	while not rospy.is_shutdown():
		try:
			rospy.sleep(0.1)
		except KeyboardInterrupt:
			velocity = Twist()
			velocity.angular.z = 0
			velocity.linear.x = 0
			pub_vel.publish(velocity)
			break
			
