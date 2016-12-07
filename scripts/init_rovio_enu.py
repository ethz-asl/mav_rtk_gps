#!/usr/bin/env python

#
#  Title:        init_rovio_state.py
#  Description:  ROS module to initialize ROVIO world frame and align it with local ENU frame.
#

import rospy
import tf.transformations as tf
from std_msgs.msg import *
import std_srvs.srv
from geometry_msgs.msg import TransformStamped, Pose
from sensor_msgs.msg import Imu
from rovio.srv import SrvResetToPose
import math

class InitRovioEnu:

    def __init__(self):

        # Read Settings
        self._samples_before_reset = rospy.get_param('~samples_before_reset', 50)
        self._send_reset_automatically = rospy.get_param('~send_reset_automatically', False)
        self._verbose = rospy.get_param('~verbose', True) # this is a crucial node, be verbose per default

        # intrinsic quaternion from IMU of the vi-sensor (C frame) to IMU of the MAV (I frame)
        qIC_w = rospy.get_param('~pose_sensor/init/q_ic/w', 1.0)
        qIC_x = rospy.get_param('~pose_sensor/init/q_ic/x', 0.0)
        qIC_y = rospy.get_param('~pose_sensor/init/q_ic/y', 0.0)
        qIC_z = rospy.get_param('~pose_sensor/init/q_ic/z', 0.0)
        self._qIC = [qIC_x, qIC_y, qIC_z, qIC_w]

        if self._verbose:
            rospy.loginfo(rospy.get_name() +
                          ": transformation from vi-sensor IMU and MAV IMU [x, y, z, w]: " + str(self._qIC))
            if self._send_reset_automatically:
                rospy.loginfo(rospy.get_name() +
                          ": reset will be sent after " + str(self._samples_before_reset) + " of IMU messages")
            else:
                rospy.loginfo(rospy.get_name() +
                          ": reset has to be sent by calling service 'send_reset_to_rovio' ")

        # Init other variables
        self._num_external_pose_read = 0
        self._resent_to_send = True
        self._qEnuI = [0.0, 0.0, 0.0, 1.0]

        # If init is not automatically, advertise service
        if not self._send_reset_automatically:
            self._reset_rovio_srv_server = rospy.Service(rospy.get_name() + '/send_reset_to_rovio', 
                                                         std_srvs.srv.Empty, 
                                                         self.handle_send_reset_to_rovio_service)

        # Subscribe to Imu topic which contains the yaw orientation
        rospy.Subscriber("mag_imu", Imu, self.mag_imu_callback)

        rospy.spin()

    def mag_imu_callback(self, imu_msg):

        self._num_external_pose_read += 1

        # Compute new pose to use when resetting ROVIO
        # orientation of the IMU frame of the MAV (body frame, or I frame according to MSF)
        self._qEnuI = [imu_msg.orientation.x,
                       imu_msg.orientation.y,
                       imu_msg.orientation.z,
                       imu_msg.orientation.w]

        # If reset has to be sent automatically after a ceratin amount of IMU with magnetometer samples
        if  self._send_reset_automatically and self._resent_to_send and \
            self._num_external_pose_read > self._samples_before_reset:

            self.send_reset_to_rovio()
            # TODO (marco-tranzatto) check return of service call before setting this to false
            self._resent_to_send = False 

    def handle_send_reset_to_rovio_service(self, request):
        self.send_reset_to_rovio()
        return std_srvs.srv.EmptyResponse()
        #response = Empty()
        #return response

    def send_reset_to_rovio(self):
        rospy.wait_for_service('rovio/reset_to_pose')
        try:
            rovio_reset_srv = rospy.ServiceProxy('rovio/reset_to_pose', SrvResetToPose)

            # compute pose from local ENU (East-North-Up frame) to
            # IMU frame of the MAV (== body frame or C frame, according to MSF)
            qEnuC = tf.quaternion_multiply(self._qEnuI, self._qIC)
            pose_world_imu_msg = Pose()
            # For now assume world frame has same origin as IMU frame
            pose_world_imu_msg.position.x = 0.0
            pose_world_imu_msg.position.y = 0.0
            pose_world_imu_msg.position.z = 0.0
            pose_world_imu_msg.orientation.w = qEnuC[3]
            pose_world_imu_msg.orientation.x = qEnuC[0]
            pose_world_imu_msg.orientation.y = qEnuC[1]
            pose_world_imu_msg.orientation.z = qEnuC[2]
            
            response = rovio_reset_srv(pose_world_imu_msg)
            rospy.loginfo(rospy.get_name() + ": sent reset to ROVIO")

            # print orientation to initialize rovio, this information
            # should always be double checked with manual measurements
            qEnuI_euler = tf.euler_from_quaternion(self._qEnuI, 'rzyx')
            rospy.loginfo(rospy.get_name() + ": body frame of MAV assumed with " +
                          str(math.degrees(qEnuI_euler[0])) + " (deg) roll, " +
                          str(math.degrees(qEnuI_euler[1])) + " (deg) pitch, " +
                          str(math.degrees(qEnuI_euler[2])) + " (deg) yaw from local ENU (local axis, ZYX)")

            qEnuC_euler = tf.euler_from_quaternion(qEnuC, 'rzyx')
            rospy.loginfo(rospy.get_name() + ": camera IMU assumed " +
                          str(math.degrees(qEnuC_euler[0])) + " (deg) roll, " +
                          str(math.degrees(qEnuC_euler[1])) + " (deg) pitch, " +
                          str(math.degrees(qEnuC_euler[2])) + " (deg) yaw from local ENU (local axis, ZYX)")

        except rospy.ServiceException, e:
            print "Service call to reset rovio internal state failed: %s"%e

if __name__ == '__main__':

    rospy.init_node('init_rovio_enu')
    rospy.loginfo(rospy.get_name() + " start")

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        init_rovio_enu = InitRovioEnu()
    except rospy.ROSInterruptException: pass
