#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from srv_msg.srv import DistCmd
from math import pi
def cmd_client(x):
    try:
        linCmd = rospy.ServiceProxy('cmd_lin_dist', DistCmd)
        angCmd = rospy.ServiceProxy('cmd_ang_dist', DistCmd)
        respl = linCmd(0.2)
        choose_park_side = 'left'
        if choose_park_side == 'left':
            respa = angCmd(pi/2.2)
        else:
            respa = angCmd(-pi/2.2)
        respl = linCmd(0.25)
        respa = angCmd(pi/2.2)
        rospy.sleep(0.1)
        respa = angCmd(pi/2.2)
        respl = linCmd(0.25)
        if choose_park_side == 'left':
            respa = angCmd(-pi/2.2)
        else:
            respa = angCmd(pi/2.2)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    cmd_client(10)
