#!/usr/bin/env python
import rospy
import roslaunch
from std_msgs.msg import Bool, Empty
from geometry_msgs.msg import Point, Twist
in_tunnel = False
def in_tunnel_cb(data):
    global in_tunnel
    in_tunnel = data.data

def go_to_tunnel():
    line_control_pub = rospy.Publisher("line_move_flag", Bool, queue_size = 5)
    cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 5)
    
    line_control_pub.publish(msg_line)
    
    msg_cmd = Twist() #stop the robot
    msg.linear.x = 0 
    msg.angular.z = 0
    cmd_pub.publish(msg_cmd)

def go_from_tunnel():
    line_control_pub = rospy.Publisher("line_move_flag", Bool, queue_size = 5)
    msg_line = Bool()
    msg_line.data = True
    line_control_pub.publish(msg_line)

if __name__ == '__main__':
    rospy.init_node('amcl_launch')
    point_pub = rospy.Publisher("target_pose", Point, queue_size = 5)
    reset_pub = rospy.Publisher("reset", Empty, queue_size = 5)

    line_control_pub = rospy.Publisher("line_move_flag", Bool, queue_size = 5)
    cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 5)
    

    in_tunnel_sub = rospy.Subscriber("in_tunnel", Bool, in_tunnel_cb)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/nuc/tb_ws/src/turtlebot3/turtlebot3_navigation/launch/turtlebot3_navigation.launch"])
    while not rospy.is_shutdown():
        try:
            if(in_tunnel == True):
				# go_to_tunnel()
				rospy.sleep(1.5)
				print "here"
				turn = Bool()
				turn.data = False
				line_control_pub.publish(turn)
				line_control_pub.publish(turn)
				msg_cmd = Twist() #stop the robot
				msg_cmd.linear.x = 0.22 
				msg_cmd.angular.z = 0
				cmd_pub.publish(msg_cmd)
				rospy.sleep(1.5)
				msg_cmd.linear.x = 0 
				cmd_pub.publish(msg_cmd)
				cmd_pub.publish(msg_cmd)				
				print("IN tunnel")
				rospy.sleep(1)
				msg = Empty()
				for i in range(0, 3, 1):
					reset_pub.publish(msg)
					cmd_pub.publish(msg_cmd)
					cmd_pub.publish(msg_cmd)
					rospy.sleep(0.02)
				launch.start()
				rospy.sleep(3)
				target = Point()
				target.x = 1.713
				target.y = 1.923
				target.z = 90
				point_pub.publish(target)

				while in_tunnel == True:
					rospy.sleep(0.1)
				go_from_tunnel()
				launch.shutdown()
				break
            rospy.sleep(0.1)
        except KeyboardInterrupt:
            break
            print("Shutting down")
