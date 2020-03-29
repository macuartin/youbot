#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def planner_listener():
    rospy.init_node('inverse_kinematic')
    subscriber_topic = 'traj_planner_topic'
    sub = rospy.Subscriber(subscriber_topic, String, printer)
    rospy.spin()


def printer(req):
    print req


if __name__ == '__main__':
    planner_listener()