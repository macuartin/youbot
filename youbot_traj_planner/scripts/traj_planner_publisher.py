#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from youbot_traj_planner.traj_planner_publisher import cubic_splines_planner
from numpy import pi


def planner_talker(q, qd, qdd, path_points):
    rospy.init_node('traj_planner')
    publisher_topic = 'traj_planner_topic'
    pub = rospy.Publisher(publisher_topic, String, queue_size=10)
    rate = rospy.Rate(10)

    for point in range(path_points):
        pub.publish('Hi')
        rospy.loginfo('Logging..')
        rate.sleep()

        if rospy.is_shutdown():
            break


if __name__ == '__main__':

    try:
        checkpoints = [[0, 0, 0, pi/3, pi, pi/2], [5, 7, 1, pi/2, pi/3, pi/3]]
        checkpoints_timing = [1, 4]
        initial_velocity = [2, 3, 6]
        final_velocity = [4, 2, 1]
        sampling_time = 0.05

        cubic_splines_planner(checkpoints, checkpoints_timing,
                              initial_velocity, final_velocity, sampling_time)

        planner_talker(1, 1, 1, 100)

    except rospy.ROSInterruptException:
        pass
