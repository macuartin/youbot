#!/usr/bin/env python

from __future__ import print_function
from youbot_msgs.srv import InverseKinematic
import rospy
from youbot_mechanics.robot import Robot


def handle_trajectory(req):
    rospy.loginfo('handling \n{}'.format(req.p))
    return True


if __name__ == "__main__":

    try:
        # dh = rospy.get_param('/inverse_kinematic_node/dh')
        dh = {'j1': ['q', 0.147, 0.033, 1.57079],
              'j2': ['q', 0, 0.155, 0],
              'j3': ['q', 0, 0.135, 0],
              'j4': ['q', 0, 0, 1.57079],
              'j5': ['q', 0.2175, 0, 0]}

        q0 = [0, 0, 0, 0, 0]

        rospy.init_node('inverse_kinematic_server')

        youbot = Robot(dh, q0)

        s = rospy.Service('inverse_kinematic',
                          InverseKinematic,
                          handle_trajectory)

        rospy.loginfo('Inverse Kinematic Server')
        rospy.spin()

    except rospy.ROSException as err:
        rospy.logerr(err)
