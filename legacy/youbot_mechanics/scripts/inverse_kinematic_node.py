#!/usr/bin/env python
import rospy
from youbot_msgs.msg import Point
from rospy.numpy_msg import numpy_msg
from youbot_mechanics.inverse_kinematic import inverse_kinematic
from youbot_mechanics.robot import Robot


if __name__ == '__main__':

    try:
        # dh = rospy.get_param('/inverse_kinematic_node/dh')
        dh = {'j1': ['q', 0.147, 0.033, 1.57079],
              'j2': ['q', 0, 0.155, 0],
              'j3': ['q', 0, 0.135, 0],
              'j4': ['q', 0, 0, 1.57079],
              'j5': ['q', 0.2175, 0, 0]}

        q0 = [0, 0, 0, 0, 0]

        youbot = Robot(dh, q0)
        rospy.init_node('inverse_kinematic_node')
        sub = rospy.Subscriber('traj_planner_topic',
                               numpy_msg(Point),
                               trajectory_handler)
        rospy.spin()
    except rospy.ROSException as err:
        rospy.logerr(err)


def trajectory_handler(data):

    rospy.loginfo(data)
