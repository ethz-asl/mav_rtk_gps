#!/usr/bin/env python

#
#  Title:        bearing_from_mag.py
#  Description:  ROS module to calculate bearing angle from magnetometer data, when the MAV is placed
#                horizontally, i.e. WITH pitch == roll == 0 (or at leaset very small).
#                If the X axis of the sensor is aligned North, then bearing is zero.
#

import rospy
import tf.transformations as tf
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
import numpy as np
import math

def magnetic_field_callback(magMsg):

    global num_magnetometer_reads
    global number_samples_average
    global array_bearings
    global constant_offset

    # Correct magnetic filed
    raw_mag = np.array([magMsg.vector.x,
                        magMsg.vector.y,
                        magMsg.vector.z])

    # corrected_mag = compensation * (raw_mag - offset)
    corrected_mag = np.dot(mag_compensation, raw_mag - mag_offset)

    # compute yaw angle using corrected magnetometer measurements
    # and ASSUMING ZERO pitch and roll of the magnetic sensor!
    # adapted from
    # https://github.com/KristofRobot/razor_imu_9dof/blob/indigo-devel/src/Razor_AHRS/Compass.ino
    corrected_mag = corrected_mag / np.linalg.norm(corrected_mag)
    mag_bearing = math.atan2(corrected_mag[1], -corrected_mag[0]);

    # add declination and constant offset
    mag_bearing = mag_bearing + mag_declination + constant_offset

    # publish unfiltered bearing
    pub_bearing_raw.publish(Float64(math.degrees(mag_bearing)))

    # compute mean 
    array_bearings[num_magnetometer_reads] = mag_bearing
    num_magnetometer_reads += 1

    if num_magnetometer_reads >= number_samples_average:
        num_magnetometer_reads = 0 # delete oldest samples

    bearing_avg = mitsuta_mean(array_bearings)

    # WARNING: we assume zero roll and zero pitch!
    q_avg = tf.quaternion_from_euler(0.0, 0.0, bearing_avg);
    imu_msg = Imu()
    imu_msg.orientation.w = q_avg[3]
    imu_msg.orientation.x = q_avg[0]
    imu_msg.orientation.y = q_avg[1]
    imu_msg.orientation.z = q_avg[2]

    pub_bearing_avg.publish(Float64(math.degrees(bearing_avg)))
    pub_imu_bearing_avg.publish(imu_msg)

    # debug
    if print_debug:
        rospy.loginfo("bearing_avg (deg): " +
                      str(math.degrees(bearing_avg)))

        mag_corrected_msg = magMsg
        mag_corrected_msg.vector.x = corrected_mag[0]
        mag_corrected_msg.vector.y = corrected_mag[1]
        mag_corrected_msg.vector.z = corrected_mag[2]
        pub_mag_corrected.publish(mag_corrected_msg)

    

# Mitsuta mean used to average angles. This is necessary in order to avoid
# misleading behaviours. For example, if the measurements are swtiching between 
# -180 and +180 (they are the same angle, just with differente representation)
# then a normal mean algorithm would give you 0, which is completely wrong.
# Code adapted from:
# https://github.com/SodaqMoja/Mitsuta/blob/master/mitsuta.py
def mitsuta_mean(angles_array):
    # Function meant to work with degrees, covert inputs
    # from radians to degrees and output from degrees to radians
    D = math.degrees(angles_array[0])
    mysum = D
    for val in angles_array[1:]:
        val = math.degrees(val)
        delta = val - D
        if delta < -180:
            D = D + delta + 360
        elif delta < 180:
            D = D + delta
        else:
            D = D + delta - 360
        mysum = mysum + D
    m = mysum / len(angles_array)

    avg = math.radians((m + 360) % 360)
    # make sure avg is between -pi and pi
    if avg > math.pi:
        avg = avg - 2 * math.pi
    elif avg < -math.pi:
        avg = avg + 2 * math.pi

    return avg



if __name__ == '__main__':

    rospy.init_node('bearing_from_mag')
    rospy.loginfo(rospy.get_name() + " start")
    
    # Read Settings
    # Magnetometer
    if not rospy.has_param('~magnetometer/declination'):
        declination = 0.0
    else:
        declination = rospy.get_param('~magnetometer/declination')

    if not rospy.has_param('~magnetometer/declination'):
        mag_declination = 0.0
    else:
        mag_declination = rospy.get_param('~magnetometer/declination')

    if not rospy.has_param('~magnetometer/offset'):
        mag_offset = np.array([0.0, 0.0, 0.0])
    else:
        mag_offset = np.array(rospy.get_param('~magnetometer/offset'))
        if mag_offset.size != 3:
            rospy.logerr("param 'magnetometer/offset' must be an array with 3 elements.")
            mag_offset = np.array([0.0, 0.0, 0.0])

    if not rospy.has_param('~magnetometer/compensation'):
        mag_compensation = np.array([0.0, 0.0, 0.0],
                                    [0.0, 0.0, 0.0],
                                    [0.0, 0.0, 0.0])
    else:
        mag_compensation = np.array(rospy.get_param('~magnetometer/compensation'))
        if mag_compensation.size != 9:
            rospy.logerr("param 'magnetometer/compensation' must be an array with 9 elements")
            mag_compensation = np.array([0.0, 0.0, 0.0],
                                        [0.0, 0.0, 0.0],
                                        [0.0, 0.0, 0.0])
        else:
            # create matrix from array
            mag_compensation = mag_compensation.reshape(3,3)

    if not rospy.has_param('~magnetometer/constant_offset'):
        constant_offset = math.pi / 4
        #WARNING: Hummingbird version of the autopilot used to test this
        # script has a constant offset of 45 degrees added to the
        # magnetic field raw measurements (they're not so "raw" ...)
    else:
        constant_offset = rospy.get_param('~magnetometer/constant_offset')

    # Other Settings
    if not rospy.has_param('~number_samples_average'):
        number_samples_average = 10
    else:
        number_samples_average = rospy.get_param('~number_samples_average')

    num_magnetometer_reads = 0
    array_bearings = np.zeros(shape = (number_samples_average, 1))


    # Debug
    if not rospy.has_param('~print_debug'):
        print_debug = False
    else:
        print_debug = rospy.get_param('~print_debug')

    if print_debug:
        rospy.loginfo(rospy.get_name() +
                      " magnetometer offset: " + str(mag_offset))
        rospy.loginfo(rospy.get_name() +
                      " magnetometer compensation: \n" + str(mag_compensation))

    # Print these information in any case
    rospy.logwarn(rospy.get_name() +
                  " constant offset added to final measurements: " +
                  str(math.degrees(constant_offset)) + " (deg)")

    rospy.logwarn(rospy.get_name() +
                  " declination: " +
                  str(math.degrees(declination)) + " (deg)")

    # Subscribe to magnetometer topic
    rospy.Subscriber("magnetic_field", Vector3Stamped, magnetic_field_callback)

    # Publishers
    pub_bearing_raw = rospy.Publisher(rospy.get_name() + '/bearing_raw', Float64, queue_size = 10)
    pub_bearing_avg = rospy.Publisher(rospy.get_name() + '/bearing_avg', Float64, queue_size = 10)
    pub_imu_bearing_avg = rospy.Publisher(rospy.get_name() + '/imu_bearing_avg',
                                          Imu, queue_size = 10)

    if print_debug:
        pub_mag_corrected = rospy.Publisher(rospy.get_name() + '/mag_corrected',
                                            Vector3Stamped, queue_size = 10)

    # Spin
    rospy.spin()



