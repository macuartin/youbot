#!/usr/bin/env python
import rospy
from youbot_msgs.msg import Point
from rospy.numpy_msg import numpy_msg



def planner_listener():
    rospy.init_node('inverse_kinematic_node')
    sub = rospy.Subscriber('traj_planner_topic', numpy_msg(Point), printer)
    rospy.spin()


def printer(data):
    rospy.loginfo(data.x)


if __name__ == '__main__':
    planner_listener()
