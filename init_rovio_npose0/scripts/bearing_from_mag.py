#!/usr/bin/env python

#
#  Title:        bearing_from_mag.py
#  Description:  ROS module to calculate bearing angle from magnetometer data, when the MAV is placed
#                horizontally, i.e. WITH pitch == roll == 0 (or at least very small).
#

import rospy
import tf.transformations as tf
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
import numpy as np
import math

class BearingFromMag():
    def __init__(self):
        # Read Settings
        self.read_settings()

        # Init other variables
        self._num_magnetometer_reads = 0
        self._latest_bearings = np.zeros(shape = (self._number_samples_average, 1))
        self._received_enough_samples = False

        # Subscribe to magnetometer topic
        rospy.Subscriber("magnetic_field", Vector3Stamped, self.magnetic_field_callback)

        # Publishers
        self._pub_bearing_raw = rospy.Publisher(rospy.get_name() + '/bearing_raw_deg',
                                                Float64, queue_size = 10)
        self._pub_bearing_avg = rospy.Publisher(rospy.get_name() + '/bearing_avg_deg',
                                                Float64, queue_size = 10)
        self._pub_imu_bearing_avg = rospy.Publisher(rospy.get_name() + '/imu_bearing_avg',
                                                Imu, queue_size = 10)

        if self._verbose:
            self._pub_mag_corrected = rospy.Publisher(rospy.get_name() + '/mag_corrected',
                                                      Vector3Stamped, queue_size = 10)

        rospy.spin()

    def read_settings(self):
        # Declination
        self._declination = math.radians(rospy.get_param('~declination_deg', 0.0))

        # Calibration offset
        self._calibration_offset = np.array(rospy.get_param('~calibration_offset', [0.0, 0.0, 0.0]))

        # Calibration compensation
        self._calibration_compensation = np.array(rospy.get_param('~calibration_compensation',
                                                                  [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]))
        # create matrix from array
        self._calibration_compensation = self._calibration_compensation.reshape(3,3)

        # Constant bearing offset
        self._bearing_offset = math.radians(rospy.get_param('~bearing_constant_offset_deg', 0.0))

        # Number of samples used to compute average
        self._number_samples_average = rospy.get_param('~number_samples_average', 10)

        # Verbose
        self._verbose = rospy.get_param('~verbose', "True")

        # Print some useful information
        rospy.logwarn(rospy.get_name() +
                      " declination: " +
                      str(math.degrees(self._declination)) + " (deg)")

        rospy.logwarn(rospy.get_name() +
                      " constant bearing offset added to final bearing: " +
                      str(math.degrees(self._bearing_offset)) + " (deg)")

        if self._verbose:
            rospy.loginfo(rospy.get_name() +
                          " calibration offset: " + str(self._calibration_offset))
            rospy.loginfo(rospy.get_name() +
                          " calibration compensation: \n" +
                          str(self._calibration_compensation))
            rospy.loginfo(rospy.get_name() +
                          " number of samples to average: " +
                          str(self._number_samples_average))

    def magnetic_field_callback(self, magnetometer_msg):

        # Correct magnetic filed
        raw_mag = np.array([magnetometer_msg.vector.x,
                            magnetometer_msg.vector.y,
                            magnetometer_msg.vector.z])

        # corrected_mag = compensation * (raw_mag - offset)
        corrected_mag = np.dot(self._calibration_compensation,
                               raw_mag - self._calibration_offset)

        # compute yaw angle using corrected magnetometer measurements
        # and ASSUMING ZERO pitch and roll of the magnetic sensor!
        # adapted from
        # https://github.com/KristofRobot/razor_imu_9dof/blob/indigo-devel/src/Razor_AHRS/Compass.ino
        corrected_mag = corrected_mag / np.linalg.norm(corrected_mag)
        mag_bearing = math.atan2(corrected_mag[1], -corrected_mag[0])

        # add declination and constant bearing offset
        mag_bearing = mag_bearing + self._declination + self._bearing_offset

        # publish unfiltered bearing, degrees only for debug purposes
        self._pub_bearing_raw.publish(Float64(math.degrees(mag_bearing)))

        # compute mean
        self._latest_bearings[self._num_magnetometer_reads] = mag_bearing
        self._num_magnetometer_reads += 1

        if self._num_magnetometer_reads >= self._number_samples_average:
            self._num_magnetometer_reads = 0 # delete oldest samples
            self._received_enough_samples = True

        if self._received_enough_samples:
            bearing_avg = self.angular_mean(self._latest_bearings)
        else:
            # not enough samples, use latest value
            bearing_avg = mag_bearing

        # WARNING: we assume zero roll and zero pitch!
        q_avg = tf.quaternion_from_euler(0.0, 0.0, bearing_avg);
        imu_msg = Imu()
        imu_msg.orientation.w = q_avg[3]
        imu_msg.orientation.x = q_avg[0]
        imu_msg.orientation.y = q_avg[1]
        imu_msg.orientation.z = q_avg[2]

        self._pub_bearing_avg.publish(Float64(math.degrees(bearing_avg)))
        self._pub_imu_bearing_avg.publish(imu_msg)

        # debug
        if self._verbose:
            rospy.loginfo("bearing_avg : " +
                          str(math.degrees(bearing_avg)) + " deg")

            mag_corrected_msg = magnetometer_msg
            mag_corrected_msg.vector.x = corrected_mag[0]
            mag_corrected_msg.vector.y = corrected_mag[1]
            mag_corrected_msg.vector.z = corrected_mag[2]
            self._pub_mag_corrected.publish(mag_corrected_msg)

    def angular_mean(self, angles_array):
        #  choose one of the following
        return self.atan2_mean(angles_array)
        #TODO (marco-tranzatto) remove once above has been tested
        #return self.mitsuta_mean(angles_array) 

    # From Wikipedia:
    # https://en.wikipedia.org/wiki/Mean_of_circular_quantities
    def atan2_mean(self, angles_array):
        sum_sin = 0.0
        sum_cos = 0.0

        for angle in angles_array:
            sum_sin += math.sin(angle)
            sum_cos += math.cos(angle)

        return math.atan2(sum_sin, sum_cos)

    # Mitsuta mean used to average angles. This is necessary in order to avoid
    # misleading behaviours. For example, if the measurements are swtiching between
    # -180 and +180 (they are the same angle, just with differente representation)
    # then a normal mean algorithm would give you 0, which is completely wrong.
    # Code adapted from:
    # https://github.com/SodaqMoja/Mitsuta/blob/master/mitsuta.py
    def mitsuta_mean(self, angles_array):
        # Function meant to work with degrees, covert inputs
        # from radians to degrees and output from degrees to radians
        D = math.degrees(angles_array[0])
        mysum = D
        for val in angles_array[1:]:
            val = math.degrees(val)
            delta = val - D
            if delta < -180.0:
                D = D + delta + 360.0
            elif delta < 180.0:
                D = D + delta
            else:
                D = D + delta - 360.0
            mysum = mysum + D
        m = mysum / len(angles_array)

        avg = math.radians((m + 360.0) % 360.0)
        # make sure avg is between -pi and pi
        if avg > math.pi:
            avg = avg - 2.0 * math.pi
        elif avg < -math.pi:
            avg = avg + 2.0 * math.pi

        return avg

if __name__ == '__main__':

    rospy.init_node('bearing_from_mag')
    rospy.loginfo(rospy.get_name() + " start")

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        bearing_from_mag = BearingFromMag()
    except rospy.ROSInterruptException: pass
