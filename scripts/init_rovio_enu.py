#!/usr/bin/env python

#
#  Title:        init_rovio_state.py
#  Description:  ROS module to initialize Rovio world frame and align it with local ENU frame.
#  Convention: Quaternion qAB: A_r = rotation_matrix(qAB) * B_r
#  Frames: I = MAV IMU; C = Sensor IMU; Enu = East-North-Up
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
    # read settings
    self._samples_before_reset = rospy.get_param('~samples_before_reset', 50)
    self._send_reset_automatically = rospy.get_param('~send_reset_automatically', False)
    # this is a crucial node, be verbose per default
    self._verbose = rospy.get_param('~verbose', True) 

    # quaternion from IMU of the camera-sensor (C frame) to IMU of the MAV (I frame)
    qIC_w = rospy.get_param('~pose_sensor/init/q_ic/w', 1.0)
    qIC_x = rospy.get_param('~pose_sensor/init/q_ic/x', 0.0)
    qIC_y = rospy.get_param('~pose_sensor/init/q_ic/y', 0.0)
    qIC_z = rospy.get_param('~pose_sensor/init/q_ic/z', 0.0)
    self._qIC = [qIC_x, qIC_y, qIC_z, qIC_w]

    # position of IMU of the camera-sensor (C frame) from IMU of the MAV (I frame)
    # yes, it's the other way around respect the quaternion ...
    pIC_x = rospy.get_param('~pose_sensor/init/p_ic/x', 0.0)
    pIC_y = rospy.get_param('~pose_sensor/init/p_ic/y', 0.0)
    pIC_z = rospy.get_param('~pose_sensor/init/p_ic/z', 0.0)
    self._pIC = [pIC_x, pIC_y, pIC_z]

    if self._verbose:
        rospy.loginfo(rospy.get_name() +
                      ": transformation from vi-sensor IMU and MAV IMU [x, y, z, w]: " + 
                      str(self._qIC))

        if self._send_reset_automatically:
            rospy.loginfo(rospy.get_name() +
                      ": reset will be sent after " + 
                      str(self._samples_before_reset) + " of IMU messages")
        else:
            rospy.loginfo(rospy.get_name() +
                      ": reset has to be sent by calling service 'send_reset_to_rovio' ")

    # init other variables
    self._num_external_pose_read = 0
    self._resent_to_send = True
    self._qEnuI = [0.0, 0.0, 0.0, 1.0]
    self._pose_world_imu_msg = Pose()

    # if init is not automatically, advertise service
    if not self._send_reset_automatically:
        self._reset_rovio_srv_server = rospy.Service(rospy.get_name() + 
                                                     '/send_reset_to_rovio', 
                                                     std_srvs.srv.Empty, 
                                                     self.handle_send_reset_to_rovio_service)

    # subscribe to Imu topic which contains the yaw orientation
    rospy.Subscriber("mag_imu", Imu, self.mag_imu_callback)

    rospy.spin()

  def mag_imu_callback(self, imu_msg):
    self._num_external_pose_read += 1

    # compute new pose to use when resetting ROVIO
    # orientation of the IMU frame of the MAV (body frame, or I frame according to MSF)
    self._qEnuI = [imu_msg.orientation.x,
                   imu_msg.orientation.y,
                   imu_msg.orientation.z,
                   imu_msg.orientation.w]

    if  self._send_reset_automatically and self._resent_to_send and \
        self._num_external_pose_read > self._samples_before_reset:

        self.send_reset_to_rovio()
        # TODO (marco-tranzatto) check return of service call before setting this to false
        self._resent_to_send = False 

  def handle_send_reset_to_rovio_service(self, request):
    self.send_reset_to_rovio()
    return std_srvs.srv.EmptyResponse()

  def send_reset_to_rovio(self):
    rospy.wait_for_service('rovio/reset_to_pose')
    try:
        rovio_reset_srv = rospy.ServiceProxy('rovio/reset_to_pose', SrvResetToPose)

        # compute pose from local ENU (East-North-Up frame) to
        # IMU frame of the MAV (== body frame or C frame, according to MSF)
        qEnuC = tf.quaternion_multiply(self._qEnuI, self._qIC)

        # set new Sensor IMU position and orientation respect to World frame 
        # (which is now aligend to local ENU)
        # World and ENU AND MAV IMU are assumed to have the same origin for now. 
        # TODO (marco-tranzatto) check this for CH. 3!
        self._pose_world_imu_msg.position.x = 0.0 # TODO check me (marco-tranzatto)
        self._pose_world_imu_msg.position.y = 0.0 # TODO check me (marco-tranzatto)
        self._pose_world_imu_msg.position.z = 0.0 # TODO check me (marco-tranzatto)
        self._pose_world_imu_msg.orientation.w = qEnuC[3]
        self._pose_world_imu_msg.orientation.x = qEnuC[0]
        self._pose_world_imu_msg.orientation.y = qEnuC[1]
        self._pose_world_imu_msg.orientation.z = qEnuC[2]
        
        response = rovio_reset_srv(self._pose_world_imu_msg)
        rospy.loginfo(rospy.get_name() + ": sent reset to Rovio")

        if self._verbose:
            self.print_rovio_init_info()

    except rospy.ServiceException, e:
        print "Service call to reset rovio internal state failed: %s"%e

  def print_rovio_init_info(self):
    (yaw, pitch, roll) = tf.euler_from_quaternion(self._qEnuI, 'rzyx')
    rospy.loginfo(rospy.get_name() + ": body frame of MAV assumed with " +
                  str(math.degrees(roll)) + " (deg) roll, " +
                  str(math.degrees(pitch)) + " (deg) pitch, " +
                  str(math.degrees(yaw)) + " (deg) yaw from local ENU (local axis, ZYX)")

    rospy.loginfo("------------ Debugging info for Rviz ------------")
    # final pose of Sensor IMU used to reset Rovio
    self.print_tf_debug_node("enu", "sensor_imu_rovio", # rovio_world == ENU
                             self._pose_world_imu_msg.position.x,
                             self._pose_world_imu_msg.position.y,
                             self._pose_world_imu_msg.position.z,
                             self._pose_world_imu_msg.orientation.x,
                             self._pose_world_imu_msg.orientation.y,
                             self._pose_world_imu_msg.orientation.z,
                             self._pose_world_imu_msg.orientation.w)

    self.print_tf_debug_node("enu", "mav_imu", 
                             0.0, # TODO check me (marco-tranzatto)
                             0.0, # TODO check me (marco-tranzatto)
                             0.0, # TODO check me (marco-tranzatto)
                             self._qEnuI[0],
                             self._qEnuI[1],
                             self._qEnuI[2],
                             self._qEnuI[3])

    # following tf should overlap with enu_to_sensor_imu
    # full Sensor IMU posed form msf_parameters_vision
    self.print_tf_debug_node("mav_imu", "sensor_imu_check", 
                             self._pIC[0], # TODO check me (marco-tranzatto)
                             self._pIC[1], # TODO check me (marco-tranzatto)
                             self._pIC[2], # TODO check me (marco-tranzatto)
                             self._qIC[0],
                             self._qIC[1],
                             self._qIC[2],
                             self._qIC[3])
    rospy.loginfo("------------------------------------------------- ")

  def print_tf_debug_node(self, frame_id, child_frame_id, x, y, z, qx, qy, qz, qw):
    rospy.loginfo("<node name=\"%s_to_%s_boadcaster\" pkg=\"tf\" type=\"static_transform_publisher\" \
                    args=\" %.4f %.4f %.4f %.6f %.6f %.6f %.6f %s %s 100\" />",
                    frame_id, child_frame_id,
                    x, y, z, 
                    qx, qy, qz, qw,
                    frame_id, child_frame_id)        

if __name__ == '__main__':
  rospy.init_node('init_rovio_enu')
  rospy.loginfo(rospy.get_name() + " start")

  # go to class functions that do all the heavy lifting. Do error checking.
  try:
      init_rovio_enu = InitRovioEnu()
  except rospy.ROSInterruptException: pass
