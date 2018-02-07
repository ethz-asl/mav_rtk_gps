cmake_minimum_required(VERSION 2.8.3)
project(mav_rtk_gui)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    rospy
    piksi_rtk_msgs
    nav_msgs
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
    piksi_rtk_msgs
    nav_msgs
    piksi_multi_rtk_ros
#  DEPENDS
)

#############
## Install ##
#############

catkin_python_setup()

catkin_install_python(PROGRAMS nodes/rtk_info nodes/rtk_npose0 nodes/rtk_npose1
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
