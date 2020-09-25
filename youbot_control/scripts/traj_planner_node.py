#!/usr/bin/env python
import rospy
from youbot_msgs.msg import Point
from rospy.numpy_msg import numpy_msg
from youbot_control.cartesian_traj_generator import cubic_splines_planner
from youbot_control.trajectory import TrajectoryController
from numpy import pi


if __name__ == '__main__':

    try:
        checkpoints = rospy.get_param('/traj_planner_node/checkpoints')
        checkpoints_timing = rospy.get_param('/traj_planner_node/checkpoints_timing')
        initial_velocity = rospy.get_param('/traj_planner_node/initial_velocity')
        final_velocity = rospy.get_param('/traj_planner_node/final_velocity')
        sampling_time = rospy.get_param('/traj_planner_node/sampling_time')

        controller = TrajectoryController(checkpoints,
                                          checkpoints_timing,
                                          initial_velocity,
                                          final_velocity,
                                          sampling_time,
                                          'cubic_splines')

        position = controller.get_position_trajectory()

        rospy.init_node('traj_planner_node')
        pub = rospy.Publisher('traj_planner_topic', numpy_msg(Point), queue_size=10)
        rate = rospy.Rate(10)
        msg = Point()

        for point in position:
            msg.x = point[0]
            msg.y = point[1]
            msg.z = point[2]

            pub.publish(msg)
            rospy.loginfo(msg)
            rate.sleep()

            if rospy.is_shutdown():
                break

    except rospy.ROSException as err:
        rospy.logerr(err)
