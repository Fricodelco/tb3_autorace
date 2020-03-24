#!/usr/bin/env python

import rospy
from time import sleep
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Vector3, Twist
from math import pi

class Logic:
    def __init__(self):
        self.sub_sign = rospy.Subscriber('sign', String, self.cbsign, queue_size=1)
        # self.sub_traffic = rospy.Subscriber('traffic', Bool, self.cbtraffic, queue_size=1)
        # self.sub_right = rospy.Subscriber('camera_right', String, self.cbright, queue_size=1)
        self.turn_pub = rospy.Publisher("turn", String, queue_size = 2)
        self.parking_pub = rospy.Publisher("parking", Bool, queue_size = 2)
        self.tunnel_pub = rospy.Publisher("in_tunnel", Bool, queue_size = 2)
        self.line_move_pub = rospy.Publisher("traffic", Bool, queue_size = 2)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 2)
        self.avoider_pub = rospy.Publisher("avoider", Bool, queue_size = 2)
        self.stop_line_follow = rospy.Publisher("line_move_flag", Bool, queue_size = 2)
        self.middle = []
        # self.middle.append("park")
        for i in range(0,6,1):
            self.middle.append("None")
        self.options = ("None", "parking", "right", "left", "tunnel", "road_work", "bar_up", "bar_down", "red_traffic")
        self.current_sign = "None"
        self.previos_sign = "None"
        # self.move_right("right")
        # print self.options[1]
    def cbsign(self,data):
        self.move_right(data.data)
        # self.middle[0] = data.data
        for option in self.options:
            # print self.middle
            count = self.check_count_of_option_in_list(option)
            # print self.middle
            # if self.previos_sign == "None": #if previous "None" we need six detection to determine sign
            if count >= 4:
                self.current_sign = option
                break
            else:
                self.current_sign = "None"
            """else: #if previous sign we need six detection to determine "None"
            	# print "ll"
                if count == 6:
                    self.current_sign = "None"
                    break
            	else:
            		self.current_sign = self.previos_sign"""
                    

        if self.current_sign == "parking":
        	msg = Bool()
        	msg.data = True
        	self.multuply_pub(self.parking_pub, msg)
        elif self.current_sign == "right":
            msg = String()
            msg.data = self.current_sign
            self.turn_pub.publish(msg)
            
            # self.choose_both_line()

        elif self.current_sign == "left":
            msg = String()
            msg.data = self.current_sign
            self.turn_pub.publish(msg)
            
            # self.choose_both_line()

        elif self.current_sign == "road_work":
        	msg = Bool()
        	msg.data = True
        	self.multuply_pub(self.avoider_pub,msg)

        elif self.current_sign == "red_traffic":
        	self.stop_robot()
        elif self.current_sign == "bar_down":
        	self.stop_robot()
        elif self.current_sign == "bar_up":
        	self.start_robot()
        elif self.current_sign == "tunnel":
        	msg = Bool()
        	msg.data = True
        	self.multuply_pub(self.tunnel_pub, msg)
        elif self.current_sign == "None" and self.previos_sign == "red_traffic" or self.previos_sign == "bar_down":
		rospy.sleep(1)
        	self.start_robot()
        self.previos_sign = self.current_sign
        print self.current_sign

    def check_count_of_option_in_list(self,option):
        i = 0
        for sign in self.middle:
            if sign == option:
                i+=1
        return i

    def move_right(self, data):
        # print self.middle
        self.middle.pop(len(self.middle)-1)
        tet = []
        tet.append(data)
        tet.extend(self.middle)
        self.middle = tet
        # print self.middle
    def start_robot(self):
        msg = Bool()
        msg.data = True
        self.multuply_pub(self.stop_line_follow, msg)

    def stop_robot(self):
        msg = Bool()
        msg.data = False
        self.multuply_pub(self.stop_line_follow, msg)
        msg_cmd = Twist()
        self.multuply_pub(self.cmd_pub, msg_cmd)

    """
    def choose_left_line(self):
        msg = Vector3()
        msg.x = 1
        msg.y = 0
        msg.z = 0
        self.multuply_pub(self.choose_line_pub, msg)

    def choose_both_line(self):
        msg = Vector3()
        msg.x = 0
        msg.y = 0
        msg.z = 1
        self.multuply_pub(self.choose_line_pub, msg)


    def choose_right_line(self):
        msg = Vector3()
        msg.x = 0
        msg.y = 1
        msg.z = 0
        self.multuply_pub(self.choose_line_pub, msg)"""

    def multuply_pub(self,publisher,msg):
        for i in range(0,5,1):
        	publisher.publish(msg)
        	rospy.sleep(0.02)

    

    



if __name__ == '__main__':
    rospy.init_node('logic_py')
    logic = Logic()
    while not rospy.is_shutdown():
        try:
            # error.calculate_error()
            rospy.sleep(0.05)
        except KeyboardInterrupt:
            break
            print("Shutting down")

        
