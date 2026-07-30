#!/usr/bin/env python
import rospy
from youbot_msgs.msg import Point
from youbot_msgs.srv import InverseKinematic
from rospy.numpy_msg import numpy_msg
from youbot_control.trajectory import TrajectoryController
from numpy import pi


if __name__ == '__main__':

    try:

        rospy.loginfo('wating for inverse_kinematic service')
        rospy.wait_for_service('inverse_kinematic')

        rospy.init_node('controller_node')

        checkpoints = rospy.get_param('/controller_node/checkpoints')
        checkpoints_timing = rospy.get_param('/controller_node/checkpoints_timing')
        initial_velocity = rospy.get_param('/controller_node/initial_velocity')
        final_velocity = rospy.get_param('/controller_node/final_velocity')
        sampling_time = rospy.get_param('/controller_node/sampling_time')

        controller = TrajectoryController(checkpoints,
                                          checkpoints_timing,
                                          initial_velocity,
                                          final_velocity,
                                          sampling_time,
                                          'cubic_splines')

        position = controller.get_position_trajectory()
        velocity = controller.get_velocity_trajectory()
        path_points = len(position)

        pub = rospy.Publisher('kinematic_control_topic', numpy_msg(Point), queue_size=10)
        rate = rospy.Rate(10)
        pos = Point()
        vel = Point()

        for point in range(path_points):
            pos.x = position[point][0]
            pos.y = position[point][1]
            pos.z = position[point][2]

            vel.x = velocity[point][0]
            vel.y = velocity[point][1]
            vel.z = velocity[point][2]

            get_joints = rospy.ServiceProxy('inverse_kinematic',
                                            InverseKinematic)

            joints = get_joints(pos, vel)

            pub.publish(pos)
            rospy.loginfo(joints.success)
            rate.sleep()

            if rospy.is_shutdown():
                break

    except rospy.ROSException as err:
        rospy.logerr(err)
