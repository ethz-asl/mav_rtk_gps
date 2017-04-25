#!/usr/bin/env python

#
#  Title:        init_rovio_state.py
#  Description:  ROS module to initialize Rovio world frame and align it with local ENU frame.
#  Convention:   Quaternion q_A_B: r_A = rotation_matrix(q_A_B) * r_B
#                Vector A_p_B_C: vector expressed in frame A, from point B to point C
#  Frames:       I = MAV IMU; C = Sensor IMU; Enu = East-North-Up
#

import rospy
import tf.transformations as tf
from std_msgs.msg import *
import std_srvs.srv
from geometry_msgs.msg import TransformStamped, Pose, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from rovio.srv import SrvResetToPose
import math
import os
import numpy as np

class InitRovioEnu:

    def __init__(self):
        # read settings
        self._samples_before_reset = rospy.get_param('~samples_before_reset', 50)
        self._send_reset_automatically = rospy.get_param('~send_reset_automatically', False)
        # this is a crucial node, be verbose per default
        self._verbose = rospy.get_param('~verbose', True)

        # quaternion from IMU of the camera-sensor (C frame) to IMU of the MAV (I frame)
        q_I_C_w = rospy.get_param('~pose_sensor/init/q_ic/w', 1.0)
        q_I_C_x = rospy.get_param('~pose_sensor/init/q_ic/x', 0.0)
        q_I_C_y = rospy.get_param('~pose_sensor/init/q_ic/y', 0.0)
        q_I_C_z = rospy.get_param('~pose_sensor/init/q_ic/z', 0.0)
        q_I_C = [q_I_C_x, q_I_C_y, q_I_C_z, q_I_C_w]

        # position of IMU of the camera-sensor (C frame) from IMU of the MAV (I frame)
        # yes, it's the other way around respect the quaternion ...
        I_p_I_C_x = rospy.get_param('~pose_sensor/init/p_ic/x', 0.0)
        I_p_I_C_y = rospy.get_param('~pose_sensor/init/p_ic/y', 0.0)
        I_p_I_C_z = rospy.get_param('~pose_sensor/init/p_ic/z', 0.0)
        I_p_I_C = [I_p_I_C_x, I_p_I_C_y, I_p_I_C_z]

        # position of GPS antenna (V frame, accordin to Rovio)
        # with respect to MAV IMU (I frame, accordin to MSF)
        self._I_p_I_V = rospy.get_param('~mavimu_p_mavimu_gps', [0.0, 0.0, 0.0])

        # full transformation from MAV IMU (I) to sensor IMU (C)
        # do first translation and then rotation!
        self._T_I_C = tf.concatenate_matrices(tf.translation_matrix(I_p_I_C),
                                              tf.quaternion_matrix(q_I_C))

        if self._verbose:
            rospy.loginfo(rospy.get_name() +
                          ": transformation from vi-sensor IMU and MAV IMU [x, y, z, w]: " +
                          str(q_I_C))

            if self._send_reset_automatically:
                rospy.loginfo(rospy.get_name() +
                          ": reset will be sent after " +
                          str(self._samples_before_reset) + " of IMU messages")
            else:
                rospy.loginfo(rospy.get_name() +
                          ": reset has to be sent by calling service 'send_reset_to_rovio' ")

        # init other variables
        self._num_imu_msgs_read = 0
        self._num_gps_transform_msgs_read = 0
        self._Enu_p_Enu_V = [0.0, 0.0, 0.0]
        self._automatic_rovio_reset_sent_once = False
        self._pose_world_imu_msg = Pose()
        self._T_Enu_I = tf.identity_matrix()

        # allow testing in Vicon
        testing_in_vicon =  rospy.get_param('~testing_in_vicon', False)
        if testing_in_vicon:
          # manually set this paramter if testing in Vicon
          # it will be overwritten in any case after the first external GPS measurement
          self._Enu_p_Enu_V = rospy.get_param('~vicon_p_vicon_gps', [0.0, 0.0, 0.0])
          self._num_gps_transform_msgs_read = 1

        # advertise service
        self._reset_rovio_srv_server = rospy.Service(rospy.get_name() +
                                                     '/send_reset_to_rovio',
                                                     std_srvs.srv.Trigger,
                                                     self.send_reset_to_rovio_service_callback)

        # subscribe to Imu topic which contains the yaw orientation
        rospy.Subscriber(rospy.get_name() + "/mag_imu", Imu, self.mag_imu_callback)
        # use either gps_transform or gps_pose
        rospy.Subscriber(rospy.get_name() + "/gps_transform", TransformStamped, 
                         self.gps_transform_callback)
        rospy.Subscriber(rospy.get_name() + "/gps_pose", PoseWithCovarianceStamped,
                         self.gps_pose_callback)

        rospy.spin()

    def gps_pose_callback(self, pose_msg):
        self._num_gps_transform_msgs_read += 1
        self._Enu_p_Enu_V = [ pose_msg.pose.pose.position.x,
                              pose_msg.pose.pose.position.y,
                              pose_msg.pose.pose.position.z]

    def gps_transform_callback(self, gps_msg):
        self._num_gps_transform_msgs_read += 1
        self._Enu_p_Enu_V = [ gps_msg.transform.translation.x,
                              gps_msg.transform.translation.y,
                              gps_msg.transform.translation.z]

    def mag_imu_callback(self, imu_msg):
        self._num_imu_msgs_read += 1

        # compute new pose to use when resetting ROVIO
        # orientation of the IMU frame of the MAV (body frame, or I frame according to MSF)
        q_Enu_I = [imu_msg.orientation.x,
                   imu_msg.orientation.y,
                   imu_msg.orientation.z,
                   imu_msg.orientation.w]
        R_Enu_I = tf.quaternion_matrix(q_Enu_I)

        # use latest position received from GPS, but first compute position of MAV IMU from GPS
        # by subtracting offset between MAV IMU and GPS antenna
        I_p_I_V = np.array(self._I_p_I_V)
        Enu_p_I_V = np.dot(R_Enu_I[0:3, 0:3], I_p_I_V)
        Enu_p_Enu_I = self._Enu_p_Enu_V - Enu_p_I_V

        # full transformation from MAV IMU (I) to local ENU (East-North-Up) frame
        # do first translation and then rotation!
        self._T_Enu_I = tf.concatenate_matrices(tf.translation_matrix(Enu_p_Enu_I),
                                                R_Enu_I)

        if  self._send_reset_automatically and not self._automatic_rovio_reset_sent_once and \
            self._num_imu_msgs_read > self._samples_before_reset and self._num_gps_transform_msgs_read > 0:
            (success, message) = self.send_reset_to_rovio()

            if success:
                self._automatic_rovio_reset_sent_once = True

            rospy.loginfo(rospy.get_name() + " " + message)

    def send_reset_to_rovio_service_callback(self, request):
        response = std_srvs.srv.TriggerResponse()

        if self._automatic_rovio_reset_sent_once or self._send_reset_automatically:
            message = "Reset sent automatically after %d IMU messages, rosservice call refused." % \
                      (self._samples_before_reset)

            rospy.logwarn(rospy.get_name() + " " + message)
            response.success = False
            response.message = message
        elif self._num_imu_msgs_read <= 0:
            response.success = False
            response.message = "No external IMU message received, at least one single orientation is needed."
        elif self._num_gps_transform_msgs_read <= 0:
            response.success = False
            response.message = "No external GPS transform message received, at least one single position is needed."
        else: # everything's fine, send reset
            (success, message) = self.send_reset_to_rovio()
            response.success = success
            response.message = message

        return response

    def send_reset_to_rovio(self):
        rospy.wait_for_service('rovio/reset_to_pose')

        try:
            rovio_reset_srv = rospy.ServiceProxy('rovio/reset_to_pose', SrvResetToPose)

            # compute pose from local ENU (East-North-Up frame) to
            # IMU frame of the ViSensor (== C frame, according to MSF)
            T_Enu_C = tf.concatenate_matrices(self._T_Enu_I, self._T_I_C)
            q_Enu_C = tf.quaternion_from_matrix(T_Enu_C)

            # set new Sensor IMU position and orientation respect to World frame
            # (which is now aligned to local ENU)
            self._pose_world_imu_msg.position.x = T_Enu_C[0, 3]
            self._pose_world_imu_msg.position.y = T_Enu_C[1, 3]
            self._pose_world_imu_msg.position.z = T_Enu_C[2, 3]
            self._pose_world_imu_msg.orientation.w = q_Enu_C[3]
            self._pose_world_imu_msg.orientation.x = q_Enu_C[0]
            self._pose_world_imu_msg.orientation.y = q_Enu_C[1]
            self._pose_world_imu_msg.orientation.z = q_Enu_C[2]

            rovio_reset_srv(self._pose_world_imu_msg)

            if self._verbose:
                q_Enu_I = tf.quaternion_from_matrix(self._T_Enu_I)
                (yaw, pitch, roll) = tf.euler_from_quaternion(q_Enu_I, 'rzyx')
                rospy.loginfo(rospy.get_name() + ": body frame of MAV assumed with " +
                    str(math.degrees(roll)) + " (deg) roll, " +
                    str(math.degrees(pitch)) + " (deg) pitch, " +
                    str(math.degrees(yaw)) + " (deg) yaw from local ENU (local axis, ZYX)")

            self.create_rovio_init_info()

            success = True
            message = "Service call to reset Rovio internal state sent"

            return (success, message)

        except rospy.ServiceException, e:
            success = False
            message = "Service call to reset Rovio internal state failed: %s"%e
            return (success, message)

    def create_rovio_init_info(self):
        # Debugging file for Rviz
        # current path of init_rovio_enu.py file
        script_path = os.path.dirname(os.path.realpath(sys.argv[0]))
        # write debug tf launch file in parent launch folder of parent directory
        desired_path = "%s/../launch/debug_tf_frames.launch" % (script_path)
        file_obj = open(desired_path, 'w')
        file_obj.write("<?xml version=\"1.0\"?> \n")
        file_obj.write("<launch> \n \n")
        # final pose of Sensor IMU used to reset Rovio
        self.create_tf_debug_node("ENU", "sensor_imu_rovio",
                                  self._pose_world_imu_msg.position.x,
                                  self._pose_world_imu_msg.position.y,
                                  self._pose_world_imu_msg.position.z,
                                  self._pose_world_imu_msg.orientation.x,
                                  self._pose_world_imu_msg.orientation.y,
                                  self._pose_world_imu_msg.orientation.z,
                                  self._pose_world_imu_msg.orientation.w,
                                  file_obj)

        q_Enu_I = tf.quaternion_from_matrix(self._T_Enu_I)
        self.create_tf_debug_node("ENU", "mav_imu",
                                  self._T_Enu_I[0, 3],
                                  self._T_Enu_I[1, 3],
                                  self._T_Enu_I[2, 3],
                                  q_Enu_I[0],
                                  q_Enu_I[1],
                                  q_Enu_I[2],
                                  q_Enu_I[3],
                                  file_obj)

        # following tf should overlap with enu_to_sensor_imu
        # full Sensor IMU posed form msf_parameters_vision
        q_I_C = tf.quaternion_from_matrix(self._T_I_C)
        self.create_tf_debug_node("mav_imu", "sensor_imu_check",
                                  self._T_I_C[0, 3],
                                  self._T_I_C[1, 3],
                                  self._T_I_C[2, 3],
                                  q_I_C[0],
                                  q_I_C[1],
                                  q_I_C[2],
                                  q_I_C[3],
                                  file_obj)

        file_obj.write("\n</launch> \n\n")
        file_obj.close()

    def create_tf_debug_node(self, frame_id, child_frame_id, x, y, z, qx, qy, qz, qw, file_obj):
        buffer = "<node name=\"%s_to_%s_boadcaster\" pkg=\"tf\" type=\"static_transform_publisher\" \
                        args=\" %.4f %.4f %.4f %.6f %.6f %.6f %.6f %s %s 100\" />" % (
                        frame_id, child_frame_id,
                        x, y, z,
                        qx, qy, qz, qw,
                        frame_id, child_frame_id)

        file_obj.write(buffer)
        file_obj.write("\n")

if __name__ == '__main__':
    rospy.init_node('init_rovio_enu')
    rospy.loginfo(rospy.get_name() + " start")

    # go to class functions that do all the heavy lifting. Do error checking.
    try:
      init_rovio_enu = InitRovioEnu()
    except rospy.ROSInterruptException: pass

