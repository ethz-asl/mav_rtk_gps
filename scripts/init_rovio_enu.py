#!/usr/bin/env python

#
#  Title:        init_rovio_state.py
#  Description:  ROS module to initialize the ROVIO state and align it with local ENU frame.
#

import rospy
import tf.transformations as tf
from std_msgs.msg import *
from geometry_msgs.msg import TransformStamped, Pose
from sensor_msgs.msg import Imu
from rovio.srv import SrvResetToPose
import math

def mag_imu_callback(imuMsg):

    global num_external_pose_read
    global resent_to_send
    global samples_before_reset
    num_external_pose_read += 1

    if resent_to_send and num_external_pose_read > samples_before_reset:
        # Send reset to ROVIO
        rospy.wait_for_service('rovio/reset_to_pose')
        try:
            rovio_reset_srv = rospy.ServiceProxy('rovio/reset_to_pose', SrvResetToPose)
            # Convert GPS TransformStamped to Pose, as requested from the service
            pose_msg = Pose()

            pose_msg.position.x = 0
            pose_msg.position.y = 0
            pose_msg.position.z = 0

            # orientation of the IMU frame of the MAV (body frame, or I frame according to MSF)
            qEnuI = [imuMsg.orientation.x, 
                     imuMsg.orientation.y, 
                     imuMsg.orientation.z, 
                     imuMsg.orientation.w]

            # compute pose from local ENU (East-North-Up frame) to 
            # IMU frame of the MAV (== body frame or C frame, according to MSF)
            qEnuC = tf.quaternion_multiply(qEnuI, qIC)
            pose_msg.orientation.w = qEnuC[3]
            pose_msg.orientation.x = qEnuC[0]
            pose_msg.orientation.y = qEnuC[1]
            pose_msg.orientation.z = qEnuC[2]

            resp = rovio_reset_srv(pose_msg)
            resent_to_send = False
            rospy.loginfo(rospy.get_name() + ": sent reset to ROVIO")

            qEnuI_euler = tf.euler_from_quaternion(qEnuI)
            # print orientation to initialize rovio, this information
            # should always be double checked with manual measurements
            rospy.logwarn(rospy.get_name() + ": body frame of MAV assumed with " +
                          str(math.degrees(qEnuI_euler[0])) + " (deg) roll, " +
                          str(math.degrees(qEnuI_euler[1])) + " (deg) pitch, " +
                          str(math.degrees(qEnuI_euler[2])) + " (deg) yaw from local ENU")

        except rospy.ServiceException, e:
            print "Service call to reset rovio internal state failed: %s"%e

if __name__ == '__main__':

    rospy.init_node('init_rovio_enu')
    rospy.loginfo(rospy.get_name() + " start")
    resent_to_send = True
    num_external_pose_read = 0

    # Read Settings
    if not rospy.has_param('~samples_before_reset'):
        samples_before_reset = 50
    else:
        samples_before_reset = rospy.get_param('~samples_before_reset')

    if not rospy.has_param('~pose_sensor/init/q_ic/x') \
       and not rospy.has_param('~pose_sensor/init/q_ic/y') \
       and not rospy.has_param('~pose_sensor/init/q_ic/z') \
       and not rospy.has_param('~pose_sensor/init/q_ic/w'):
        qIC_w = 1.0
        qIC_x = 0.0
        qIC_y = 0.0
        qIC_z = 0.0
        # warn user about missing transformation between vi-sensor IMU frame (C) and MAV IMU frame (I)
        rospy.logwarn(rospy.get_name() + 
            ": missing transformation from vi-sensor IMU and MAV IMU. Using identity quaternion.")
    else:
        # this is the orientation of the vi-sensor IMU frame (C) with respect to MAV IMU frame (I)
        qIC_w = rospy.get_param('~pose_sensor/init/q_ic/w')
        qIC_x = rospy.get_param('~pose_sensor/init/q_ic/x')
        qIC_y = rospy.get_param('~pose_sensor/init/q_ic/y')
        qIC_z = rospy.get_param('~pose_sensor/init/q_ic/z')
    # intrinsic quaternion from IMU of the vi-sensor (C frame) to IMU of the MAV (I frame)
    qIC = [qIC_x, qIC_y, qIC_z, qIC_w]

     # Subscribe to Imu topic which contains the yaw orientation
    rospy.Subscriber("mag_imu", Imu, mag_imu_callback)

    # Spin
    rospy.spin()



