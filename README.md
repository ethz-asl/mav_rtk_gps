mav_rtk_gps
======

This repository contains ROS driver, tools, launch files, and wikis about how to use Piksi Real Time Kinematic (RTK) GPS device on MAVs.

Check the [Wiki](https://github.com/ethz-asl/mav_rtk_gps/wiki) for instructions on how to get started with Piksi RTK GPS receiver.

Overview
------
- [init_rovio_npose0](https://github.com/ethz-asl/mav_rtk_gps/tree/master/init_rovio_npose0) : initialize [Rovio](https://github.com/ethz-asl/rovio) to use external GPS pose measurements, when built with [NPOSE=0](https://github.com/ethz-asl/rovio/wiki/Configuration#build-configuration)
- [mav_rtk_gui](https://github.com/ethz-asl/mav_rtk_gps/tree/master/mav_rtk_gui) : handy Graphical User Interfaces to check the status of RTK fix.
- [piksi_rtk_gps](https://github.com/ethz-asl/mav_rtk_gps/tree/master/piksi_rtk_gps): ROS driver for Piksi RTK receiver device, hardware version [V2](http://docs.swiftnav.com/pdfs/piksi_datasheet_v2.3.1.pdf).
- [piksi_multi_rtk_gps](https://github.com/ethz-asl/mav_rtk_gps/tree/master/piksi_multi_rtk_gps): ROS driver for Piksi RTK receiver device, hardware version [Multi](https://www.swiftnav.com/piksi-multi).
- [piksi_rtk_msgs](https://github.com/ethz-asl/mav_rtk_gps/tree/master/piksi_rtk_msgs): ROS messages used by the driver(s).
- [utils](https://github.com/ethz-asl/mav_rtk_gps/tree/master/utils): collection of configurations and useful scripts.

Contact
-------
Marco Tranzatto marcot(at)ethz.ch
