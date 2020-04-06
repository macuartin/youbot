#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from youbot_traj_planner.cartesian_traj_generator import cubic_splines_planner
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
        checkpoints = rospy.get_param('/traj_planner_publisher/checkpoints')
        checkpoints_timing = rospy.get_param('/traj_planner_publisher/checkpoints_timing')
        initial_velocity = rospy.get_param('/traj_planner_publisher/initial_velocity')
        final_velocity = rospy.get_param('/traj_planner_publisher/final_velocity')
        sampling_time = rospy.get_param('/traj_planner_publisher/sampling_time')

        cubic_splines_planner(checkpoints, checkpoints_timing,
                              initial_velocity, final_velocity, sampling_time)

        planner_talker(1, 1, 1, 100)

    except rospy.ROSInterruptException:
        pass
