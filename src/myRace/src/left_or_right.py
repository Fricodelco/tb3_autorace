#!/usr/bin/env python

import rospy
from time import sleep
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Vector3, Twist
from math import pi
import time

class Choose:
    def __init__(self):
        self.sub_turn = rospy.Subscriber('turn', String, self.cbturn, queue_size=1)
        # self.sub_traffic = rospy.Subscriber('traffic', Bool, self.cbtraffic, queue_size=1)
        # self.sub_right = rospy.Subscriber('camera_right', String, self.cbright, queue_size=1)
        self.choose_line_pub = rospy.Publisher("choosen_line", Vector3, queue_size = 2)
        self.once = False
        self.last_time = time.time()
        # print self.last_time
    def cbturn(self,data):
        if self.once == False:
            if data.data == "left":
                self.choose_left_line()
            else:
                self.choose_right_line()
            rospy.sleep(15)
            self.choose_both_line()
            self.last_time = time.time()
            self.once = True
        elif time.time() - self.last_time > 1:
            self.once = False
            # print time.time() - self.last_time
        else:
            # print time.time() - self.last_time 
            self.once = True
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
        self.multuply_pub(self.choose_line_pub, msg)

    def multuply_pub(self,publisher,msg):
        for i in range(0,5,1):
        	publisher.publish(msg)
        	rospy.sleep(0.02)

    

    



if __name__ == '__main__':
    rospy.init_node('left_or_right_py')
    choose = Choose()
    while not rospy.is_shutdown():
        try:
            # error.calculate_error()
            rospy.sleep(0.05)
        except KeyboardInterrupt:
            break
            print("Shutting down")

        
