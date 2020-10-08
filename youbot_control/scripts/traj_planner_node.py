#!/usr/bin/env python
import rospy
from youbot_msgs.msg import Point
from youbot_msgs.srv import InverseKinematic
from rospy.numpy_msg import numpy_msg
from youbot_control.trajectory import TrajectoryController
from numpy import pi


if __name__ == '__main__':

    try:
        rospy.init_node('traj_planner_node')
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
        velocity = controller.get_velocity_trajectory()

        # pub = rospy.Publisher('traj_planner_topic', numpy_msg(Point), queue_size=10)
        rate = rospy.Rate(10)
        msg = Point()

        rospy.loginfo('wating for inverse_kinematic service')
        rospy.wait_for_service('inverse_kinematic')

        for point in position:
            msg.x = point[0]
            msg.y = point[1]
            msg.z = point[2]

            get_joints = rospy.ServiceProxy('inverse_kinematic',
                                            InverseKinematic)

            joints = get_joints(msg)

            # pub.publish(msg)
            rospy.loginfo(joints.success)
            rate.sleep()

            if rospy.is_shutdown():
                break

    except rospy.ROSException as err:
        rospy.logerr(err)
