mav_rtk_gps
======

**!!!**

**Piksi ROS drivers have been moved here: [ethz\_piksi\_ros](https://github.com/ethz-asl/ethz_piksi_ros)**

**!!!**


This repository contains python tools, launch files, and wikis about how to use Piksi Real Time Kinematic (RTK) GPS device on MAVs, i.e. fuse GPS measurements into state estimator.

Overview
------
- [init_rovio_npose0](https://github.com/ethz-asl/mav_rtk_gps/tree/master/init_rovio_npose0) : initialize [Rovio](https://github.com/ethz-asl/rovio) to use external GPS pose measurements, when built with [NPOSE=0](https://github.com/ethz-asl/rovio/wiki/Configuration#build-configuration)
- [mav_rtk_gui](https://github.com/ethz-asl/mav_rtk_gps/tree/master/mav_rtk_gui) : handy Graphical User Interfaces to check the status of RTK fix.

Installing From Source
------
```
cd ~/catkin_ws/src
git clone https://github.com/ethz-asl/mav_rtk_gps.git
catkin build init_rovio_npose0
catkin build mav_rtk_gui
```

License
-------
The source code is released under a [BSD 3-Clause license](https://github.com/ethz-asl/mav_rtk_gps/blob/master/LICENSE).

Credits
-------
Marco Tranzatto - ETHZ ASL & RSL - 30 November 2017

Contact
-------
Marco Tranzatto marcot(at)ethz.ch


Bugs & Feature Requests
-------
Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/mav_rtk_gps/issues).
