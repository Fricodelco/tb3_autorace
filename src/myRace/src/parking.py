#!/usr/bin/env python

import rospy
from time import sleep
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import LaserScan
from srv_msg.srv import DistCmd
from math import pi
import json

class Parking:
    def __init__(self):
        self.sub_left = rospy.Subscriber('camera_left', String, self.cbleft, queue_size=1)
        # self.sub_right = rospy.Subscriber('camera_right', String, self.cbright, queue_size=1)
        self.sub_scan = rospy.Subscriber('scan', LaserScan, self.cbscan, queue_size=2)
        self.sub_parking = rospy.Subscriber('parking', Bool, self.cbpark, queue_size=2)
        # self.error_pub = rospy.Publisher("error_line", Vector3, queue_size = 2)
        self.choose_line_pub = rospy.Publisher("choosen_line", Vector3, queue_size = 2)
        self.stop_line_follow = rospy.Publisher("line_move_flag", Bool, queue_size = 2)
        self.park_flag = False
        self.left_line = []
        self.choose_park_side = 'none'
    def cbpark(self,data):
	rospy.sleep(3)
        self.park_flag = data.data
    def cbscan(self,data):
        if self.park_flag == True:
            rng = data.ranges
            for i in range(45,135,2):
                if rng[i] < 0.5 and rng[i] > 0:
                    self.choose_park_side = 'right'
                    break
            for i in range(225, 315, 2):
                if rng[i] < 0.5 and rng[i] > 0:
                    self.choose_park_side = 'left'
                    break
            print self.choose_park_side
    def cbleft(self, data):
        if self.park_flag == True:
            self.choose_line(1,0,0)
            if self.left_line is not None:
                del self.left_line[:]
            self.error_left = 0
            self.dif_left_err = 0
            info = json.loads(data.data)
            self.left_line = info["line"]
            if self.left_line is not None:
                differential = []
                for i in range(1, len(self.left_line)-1, 1):
                    dif = self.left_line[i][0]-self.left_line[i-1][0]
                    differential.append(dif)
                for dif in differential:
                    if dif < - 80:
                        print "detect parking"
                        msg = Bool()
                        msg.data = False
                        self.stop_line_follow.publish(msg) 
                        self.park()
                        msg.data = True
                        self.stop_line_follow.publish(msg)
                        rospy.sleep(2)
                        self.choose_line(0,0,1)

    def choose_line(self,x,y,z):
        msg = Vector3()
        msg.x = x
        msg.y = y
        msg.z = z
        self.choose_line_pub.publish(msg)

    def park(self):
        linCmd = rospy.ServiceProxy('cmd_lin_dist', DistCmd)
        angCmd = rospy.ServiceProxy('cmd_ang_dist', DistCmd)
        respl = linCmd(0.2)
        self.park_flag = False
        if self.choose_park_side == 'left':
            respa = angCmd(pi/2.2)
        else:
            respa = angCmd(-pi/2.2)
        respl = linCmd(0.25)
        respa = angCmd(pi/2.2)
        rospy.sleep(0.1)
        respa = angCmd(pi/2.2)
        respl = linCmd(0.25)
        if self.choose_park_side == 'left':
            respa = angCmd(-pi/2.2)
        else:
            respa = angCmd(pi/2.2)

            



if __name__ == '__main__':
    rospy.init_node('parking_py')
    parking = Parking()
    while not rospy.is_shutdown():
        try:
            # error.calculate_error()
            rospy.sleep(0.05)
        except KeyboardInterrupt:
            break
            print("Shutting down")

        
