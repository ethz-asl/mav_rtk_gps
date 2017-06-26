cmake_minimum_required(VERSION 2.8.3)
project(mav_rtk_gui)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

catkin_python_setup()

catkin_install_python(PROGRAMS nodes/rtk_info nodes/rtk_npose0 nodes/rtk_npose1
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

cs_install()
cs_export()
