#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from srv_msg.srv import DistCmd, DistCmdResponse
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from math import fabs
import time

R = 0.033
D = 0.16
maxLinVel = 0.22
maxAngVel = 2.84 * 0.6

cmdVelPub = rospy.Publisher('cmd_vel',Twist,queue_size=10)

curVel = Twist()
cmdVel = Twist()
def StartInitVar():
    global cmdVel, curVel
    curVel.angular.x = 0
    curVel.angular.y = 0
    curVel.angular.z = 0
    curVel.linear.x = 0
    curVel.linear.y = 0
    curVel.angular.z = 0

    cmdVel.angular.x = 0
    cmdVel.angular.y = 0
    cmdVel.angular.z = 0
    cmdVel.linear.x = 0
    cmdVel.linear.y = 0
    cmdVel.angular.z = 0

def Sign(num):
    if (num > 0):
        return 1
    else:
        return -1

def UpdateCurVel(cur):
    global curVel
    curVel.linear.x = (cur.velocity[0]+cur.velocity[1])*R/2
    curVel.angular.z = (cur.velocity[1]-cur.velocity[0])*R/D
    print curVel.angular.z


def LinDistControl(req):
    global curVel,cmdVel
    sign = Sign(req.cmd)
    curDist = 0
    lastTimeStep = time.time()
    rate = rospy.Rate(20)
    while (fabs(curDist)<fabs(req.cmd)):
        if(fabs(curDist) < 0.25*fabs(req.cmd) or fabs(curDist) > 0.75*fabs(req.cmd)):
            cmdVel.linear.x = 0.5*sign*maxLinVel
        else:
            cmdVel.linear.x = sign*maxLinVel
        newTimeStep = time.time()
        curDist += (newTimeStep - lastTimeStep)*curVel.linear.x
        lastTimeStep = newTimeStep
        cmdVelPub.publish(cmdVel)
        rate.sleep()
    cmdVel.linear.x = 0
    cmdVelPub.publish(cmdVel)
    StartInitVar()
    return DistCmdResponse(True)

def AngDistControl(req):
    global curVel,cmdVel
    sign = Sign(req.cmd)
    curDist = 0
    lastTimeStep = time.time()
    rate = rospy.Rate(20)
    while (fabs(curDist)<fabs(req.cmd)):
        if(fabs(curDist) < 0.25*fabs(req.cmd) or fabs(curDist) > 0.75*fabs(req.cmd)):
            cmdVel.angular.z = 0.7*sign*maxAngVel
        else:
            cmdVel.angular.z = sign*maxAngVel
        newTimeStep = time.time()
        curDist += (newTimeStep - lastTimeStep)*curVel.angular.z
        lastTimeStep = newTimeStep
        cmdVelPub.publish(cmdVel)
        rate.sleep()
    cmdVel.angular.z = 0
    cmdVelPub.publish(cmdVel)
    StartInitVar()
    return DistCmdResponse(True)

def CmdDistServer():
    rospy.init_node('cmd_server')
    sl = rospy.Service('cmd_lin_dist', DistCmd, LinDistControl)
    sa = rospy.Service('cmd_ang_dist', DistCmd, AngDistControl)

if __name__ == "__main__":
    StartInitVar()
    CmdDistServer()
    rospy.Subscriber('joint_states', JointState,UpdateCurVel)
    rospy.spin()
