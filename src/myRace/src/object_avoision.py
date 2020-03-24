#!/usr/bin/env python

import rospy
from time import sleep
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import LaserScan, Imu
import json

class Avoider:
	def __init__(self):
		self.sub_err = rospy.Subscriber('error_line', Vector3, self.cb_line, queue_size=1)
		self.sub_scan = rospy.Subscriber('scan', LaserScan, self.cbscan, queue_size=1)
		self.sub_scan = rospy.Subscriber('avoider', Bool, self.cbavoid, queue_size=1)    
		self.stop_line_pub = rospy.Publisher("line_move_flag", Bool, queue_size = 2)
		self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 2)
		self.choose_line_pub = rospy.Publisher("choosen_line", Vector3, queue_size = 2)
		self.line_flag = True
		self.integral = 0
		self.collision_from = "no collision" 
		self.stop_avoision = False
		self.avoision = False
		self.stop_flag = False
		self.iterations = 0
		self.once = True
		self.choose_line(0,0,1)
	def cbavoid(self, data):
		self.avoision = data.data

	def cb_line(self,error_msg):
		if self.avoision == True:
			error = error_msg.x
			dif_error = error_msg.y
			if(self.line_flag == True):
				print "here"
				velocity = Twist()
				proportional = 0.04*error #0.07
				differential = 0.013*dif_error #0.0015
				self.integral = self.integral + 0.0001*error #0.0001
				up = proportional+differential+self.integral
				wz = up
				vx = 0.22 - 0.005*abs(up)#0.22 - 0.05*abs(up) #0.15
				# print("proportional:"+str(float('{:.2f}'.sformat(proportional)))+" differential:"+str(float('{:.2f}'.format(differential)))+" integral:"+str(float('{:.2f}'.format(self.integral))))
				self.cmd_pub(vx,wz)

	def cbscan(self, data):
		if self.avoision == True:
			ranges = data.ranges
			count_of_min_left = 0
			count_of_min_right = 0

			for i in range(0,40,1):
				if ranges[i] < 0.4 and ranges[i] > 0:
					count_of_min_left+=1

			for i in range(len(ranges)-1,320,-1):
				if ranges[i] < 0.4 and ranges[i] > 0:
					count_of_min_right+=1
					

	        
			if count_of_min_right > count_of_min_left and count_of_min_right > 10:
				self.collision_from="right"
			elif count_of_min_left > 10 and count_of_min_left > count_of_min_right:
				self.collision_from = "left"
			
			j = 0				
			for i in range(0,len(ranges)-1,2):
				if ranges[i] > 0.5 or ranges[i] == 0:
					j+=1
			# print j
			if j > 190:
				self.avoision = False
				self.stop_flag = True
				# self.collision_from = "no collision"	

	        #move on left or right line to avoid collision
			if self.collision_from=="left":
				if self.iterations >24:
					self.choose_line(0,1,0)
					print "choose right line"
					self.line_flag = True
			elif self.collision_from=="right":
				# print "choose left line"
				if self.once ==True:
					print self.iterations
					if self.iterations < 4:
						self.left_turn()
						self.iterations+=1
					elif self.iterations < 14:
						self.cmd_pub(0.22,-1.3)
						self.iterations+=1
					elif self.iterations < 22:
						self.cmd_pub(0.22,-0.4)
						self.iterations+=1
					else:
						self.once = False
						self.choose_line(1,0,0)
						self.line_flag=True
			#check for stop avoiding
			
			# else:
				# print "choose both line"
				# self.choose_line(0,0,1)
				# self.line_flag=True """

	def left_turn(self):
		self.line_flag = False
		self.cmd_pub(0.15,2.1)
		
	def get_stop_flag(self):
		return self.stop_flag
	
	def choose_line(self,l,r,f):
		msg = Vector3()
		msg.x = l
		msg.y = r
		msg.z = f
		self.choose_line_pub.publish(msg)

	def cmd_pub(self,vx,wz):
		msg = Twist()
		msg.linear.x = vx
		msg.angular.z = wz
		self.cmd_vel_pub.publish(msg)
                    
    


if __name__ == '__main__':
    rospy.init_node('Avoider')
    avoider = Avoider()
    while not rospy.is_shutdown():
        try:
			stop_flag = avoider.get_stop_flag()
			if(stop_flag == True):
				avoider.cmd_pub(0,0)
				break
			rospy.sleep(0.05)
        except KeyboardInterrupt:
            avoider.cmd_pub(0,0)
            break
            print("Shutting down")

        
