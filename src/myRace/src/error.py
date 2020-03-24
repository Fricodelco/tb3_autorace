#!/usr/bin/env python

import rospy
from time import sleep
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
import json

class Line_error:
    def __init__(self):
        self.sub_left = rospy.Subscriber('camera_left', String, self.cbleft, queue_size=1)
        self.sub_right = rospy.Subscriber('camera_right', String, self.cbright, queue_size=1)
        self.error_pub = rospy.Publisher("error_line", Vector3, queue_size = 2)
        self.choose_line_sub = rospy.Subscriber("choosen_line", Vector3, self.cb_choose, queue_size = 2)
        self.left_line = []
        self.right_line = []
        self.error_left = 0
        self.error_right = 0
        self.ideal_left = 170
        self.ideal_right = 170
        self.choosen_line = [0,0,1]
        self.dif_left_err = 0
        self.dif_right_err = 0

    def cbleft(self, data):
        if self.left_line is not None:
            del self.left_line[:]
        self.error_left = 0
        self.dif_left_err = 0
        info = json.loads(data.data)
        self.left_line = info["line"]
        if self.left_line is not None:
            for point in self.left_line:
                weight = point[1]*(-0.006)+0.94
                self.error_left = weight*(self.ideal_left - point[0])+self.error_left 
            self.error_left = self.error_left/len(self.left_line)
            if len(self.left_line) > 7:
                for i in range(7, len(self.left_line), 1):
                    self.dif_left_err = self.ideal_left - self.left_line[i][0]
                # print self.dif_left_err

    def cbright(self, data):
        if self.right_line is not None:
            del self.right_line[:]
        self.error_right = 0
        self.dif_right_err = 0
        info = json.loads(data.data)
        self.right_line = info["line"]
        
        if self.right_line is not None:
            for point in self.right_line:
                weight = point[1]*(-0.006)+0.94
                self.error_right = weight*(self.ideal_right - point[0])+self.error_right                 
            self.error_right = self.error_right/len(self.right_line)
            if len(self.right_line) > 7:
                for i in range(7, len(self.right_line), 1):
                    self.dif_right_err = self.ideal_right - self.right_line[i][0]
                # print self.dif_right_err

    def cb_choose(self,data):
        self.choosen_line[0] = data.x
        self.choosen_line[1] = data.y
        self.choosen_line[2] = data.z

    def calculate_error(self):
        print("error right:"+str(self.error_right)+" error_left:"+str(self.error_left))
        if self.choosen_line[2] == 1:
            error = (self.error_right+self.error_left)/2
            dif_error = (self.dif_right_err+self.dif_left_err)/2
        elif self.choosen_line[1] == 1:
            error = self.error_right
            dif_error = self.dif_right_err
        else:
            error = self.error_left
            dif_error = self.dif_left_err
        msg = Vector3()
        msg.x = error
        msg.y = dif_error
        self.error_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('error_line_py')
    error = Line_error()
    while not rospy.is_shutdown():
        try:
            error.calculate_error()
            rospy.sleep(0.05)
        except KeyboardInterrupt:
            break
            print("Shutting down")

        
