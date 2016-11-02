#!/usr/bin/env python

#
#  Title:        constant_yaw_to_imu.py
#  Description:  ROS module to publish imu message built from a constant yaw angle.
#

import rospy
import tf.transformations as tf
from sensor_msgs.msg import Imu
import math

def yaw_to_imu(yaw):

    # WARNING: we assume zero roll and zero pitch!
    q_avg = tf.quaternion_from_euler(0.0, 0.0, yaw);
    imu_msg = Imu()
    imu_msg.orientation.w = q_avg[3]
    imu_msg.orientation.x = q_avg[0]
    imu_msg.orientation.y = q_avg[1]
    imu_msg.orientation.z = q_avg[2]

    pub_imu.publish(imu_msg)


if __name__ == '__main__':

    rospy.init_node('constant_yaw_to_imu')
    rospy.loginfo(rospy.get_name() + " start")
    
    # Read Settings
    if not rospy.has_param('~publishing_frequency'):
        publishing_frequency = 10.0
    else:
        publishing_frequency = rospy.get_param('~publishing_frequency')

    if not rospy.has_param('~constant_yaw_deg'):
        constant_yaw = 0.0
    else:
        constant_yaw = math.radians(rospy.get_param('~constant_yaw_deg'))

    pub_imu = rospy.Publisher(rospy.get_name() + '/imu',
                              Imu, queue_size = 10)

    rate = rospy.Rate(publishing_frequency)
    while not rospy.is_shutdown():
        yaw_to_imu(constant_yaw)
        rate.sleep()



