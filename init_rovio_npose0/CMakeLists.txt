cmake_minimum_required(VERSION 2.8.3)
project(init_rovio_npose0)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    rospy
    std_msgs
    std_srvs
    geometry_msgs
    rovio
    tf
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES
  CATKIN_DEPENDS
    std_msgs
    std_srvs
    geometry_msgs
    rovio
    tf
    piksi_multi_rtk_gps
#  DEPENDS
)
