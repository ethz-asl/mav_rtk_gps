#!/usr/bin/env python

#
#  Title:        add_orientation_offset.py
#  Description:  ROS module to add a constant offset to incoming IMU messages.
#                WARNING, use this module ONLY to initialize Rovio with NPOSE=0, as ONLY IMU orientation is processed,
#                accelerations and gyros are not affected!!!
#  Convention:   q_out =  q_in * q_offset

import rospy
from sensor_msgs.msg import Imu
import tf.transformations as tf
import math


class AddOrientationOffset:
    def __init__(self):

        if rospy.has_param('~orientation_offset'):
            # Orientation offset as quaterion q = [x,y,z,w].
            self.orientation_offset = rospy.get_param('~orientation_offset')
        else:
            yaw_offset_deg = rospy.get_param('~yaw_offset_deg', 0.0)
            self.orientation_offset = tf.quaternion_from_euler(0.0, 0.0, math.radians(yaw_offset_deg))

        rospy.Subscriber(rospy.get_name() + "/imu_in", Imu, self.imu_callback)

        self.pub_imu_out = rospy.Publisher(rospy.get_name() + '/imu_out',
                                           Imu, queue_size=10)

        rospy.spin()

    def imu_callback(self, msg):

        q_in = [msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w]

        q_out = tf.quaternion_multiply(q_in, self.orientation_offset)

        imu_out = msg
        imu_out.orientation.x = q_out[0]
        imu_out.orientation.y = q_out[1]
        imu_out.orientation.z = q_out[2]
        imu_out.orientation.w = q_out[3]

        self.pub_imu_out.publish(imu_out)


if __name__ == '__main__':
    rospy.init_node('add_orientation_offset')
    rospy.loginfo(rospy.get_name() + ' start')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        add_orientation_offset = AddOrientationOffset()
    except rospy.ROSInterruptException:
        pass
