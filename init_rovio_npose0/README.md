init_rovio_npose0
======

Reset Rovio to allow fusing GPS external pose measurements and visual inertial odometry.
In order to do that, it is mandatory to align (same orientation) GPS frame and Rovio odometry frame.


GPS measurements are assumed to be expressed in a local ENU (East-North-Up) frame.
If you have Navigation Satellite fix measurements (Latitude, Longitude and Altitude) you can use the repo [Geodetic Utils](https://github.com/ethz-asl/geodetic_utils) to convert them in local ENU.

Parameters `pose_sensor/init/q_*` and `pose_sensor/init/p_*` can be obtained with [Kalibr](https://github.com/ethz-asl/kalibr).

Dependencies
------
  * [ethz\_piksi\_ros](https://github.com/ethz-asl/ethz_piksi_ros)

Published and subscribed topics/services
------

- Subscribed topics:
  - **`init_rovio_enu/mag_imu`** of type `sensor_msgs/Imu Message`. This is the orientation of the MAV IMU with respect to local ENU frame (i.e., **`yaw` is with respect to East axis**).
  - **`init_rovio_enu/gps_transform`** of type `geometry_msgs/TransformStamped`. This is the transformation (rotation part does not really matter in this case) from local ENU frame to GPS antenna.
  - **`init_rovio_enu/gps_pose`** of type `geometry_msgs/PoseWithCovarianceStamped`. This is the pose (rotation part does not really matter in this case) from local ENU frame to GPS antenna. (use either `init_rovio_enu/gps_transform` or `init_rovio_enu/gps_pose`).
  
- Advertised services:
  - **`init_rovio_enu/send_reset_to_rovio`** of type `std_srvs/Trigger`. This resets Rovio internal state and alignes Rovio odometry frame to local ENU frame.
  
Parameters
------
A summary of the parameters:

| Parameter                  | Description                                                                     |
| --------------------       |:-------------------------------------------------------------------------------:| 
| `mavimu_p_mavimu_gps`      | position of GPS antenna from MAV IMU, expressed in IMU MAV.                     |
| `send_reset_automatically` | should reset to Rovio be sent automatically?                                    |
| `samples_before_reset`     | # samples of `mag_imu` after which automatic reset is sent                      |
| `verbose`                  | verbose output of the node                                                      |
| `pose_sensor/init/q_ic/w`  | W of quaternion from IMU of the camera-sensor to MAV IMU.                       |
| `pose_sensor/init/q_ic/x`  | X of quaternion from IMU of the camera-sensor to MAV IMU.                       |
| `pose_sensor/init/q_ic/y`  | Y of quaternion from IMU of the camera-sensor to MAV IMU.                       |
| `pose_sensor/init/q_ic/z`  | Z of quaternion from IMU of the camera-sensor to MAV IMU.                       |
| `pose_sensor/init/p_ic/x`  | X of position of IMU of the camera-sensor from MAV IMU, expressed in MAV IMU.   |
| `pose_sensor/init/p_ic/y`  | Y of position of IMU of the camera-sensor from MAV IMU, expressed in MAV IMU.   |
| `pose_sensor/init/p_ic/z`  | Z of position of IMU of the camera-sensor from MAV IMU, expressed in MAV IMU.   |

Contact
-------
Marco Tranzatto marcot(at)ethz.ch
