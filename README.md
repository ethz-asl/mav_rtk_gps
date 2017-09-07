mav_rtk_gps
======

This repository contains (python) ROS drivers, tools, launch files, and wikis about how to use Piksi Real Time Kinematic (RTK) GPS device on MAVs (or more in general on any moving robot). There are two different driver versions: one for Piksi V2 and one for Piksi Multi. 

**Check the [Wiki](https://github.com/ethz-asl/mav_rtk_gps/wiki) for instructions on how to get started with Piksi RTK GPS receiver.**

**The main advantage of these ROS drivers is supporting a two link communication for GPS corrections: Xbee and Wifi (see [Correction Over WiFi](https://github.com/ethz-asl/mav_rtk_gps/wiki/Corrections-Over-WiFi) for more info).**

Overview
------
- [init_rovio_npose0](https://github.com/ethz-asl/mav_rtk_gps/tree/master/init_rovio_npose0) : initialize [Rovio](https://github.com/ethz-asl/rovio) to use external GPS pose measurements, when built with [NPOSE=0](https://github.com/ethz-asl/rovio/wiki/Configuration#build-configuration)
- [mav_rtk_gui](https://github.com/ethz-asl/mav_rtk_gps/tree/master/mav_rtk_gui) : handy Graphical User Interfaces to check the status of RTK fix.
- [piksi_multi_rtk_gps](https://github.com/ethz-asl/mav_rtk_gps/tree/master/piksi_multi_rtk_gps): ROS driver for Piksi RTK receiver device, hardware version [Multi](https://www.swiftnav.com/piksi-multi).
- [piksi_rtk_gps](https://github.com/ethz-asl/mav_rtk_gps/tree/master/piksi_rtk_gps): ROS driver for Piksi RTK receiver device, hardware version [V2](http://docs.swiftnav.com/pdfs/piksi_datasheet_v2.3.1.pdf).
- [piksi_rtk_kml](https://github.com/ethz-asl/mav_rtk_gps/tree/master/piksi_rtk_kml): ROS node to generate KML files (Keyhole Markup Language) from Piksi messages. These files can be visualized in Google Earth.
- [piksi_rtk_msgs](https://github.com/ethz-asl/mav_rtk_gps/tree/master/piksi_rtk_msgs): ROS messages used by the driver(s).
- [utils](https://github.com/ethz-asl/mav_rtk_gps/tree/master/utils): collection of configurations and useful scripts.

Impatient Users
------
### Piksi Multi
RTK fix obtained in average in 3 minutes.
 - [Install Piksi Multi ROS Driver](https://github.com/ethz-asl/mav_rtk_gps/tree/master/piksi_multi_rtk_gps#installation).
 - [Configure Your Piksi Multi](https://github.com/ethz-asl/mav_rtk_gps/wiki/Installing-and-Configuring-Piksi#settings-piksi-multi).
  - [Example Launch Files](https://github.com/ethz-asl/mav_rtk_gps/tree/master/piksi_multi_rtk_gps#usage).
  
### Piksi V2
RTK fix obtained in average in 10 minutes.
 - [Install Piksi V2 ROS Driver](https://github.com/ethz-asl/mav_rtk_gps/tree/master/piksi_rtk_gps#installation).
 - [Configure Your Piksi V2](https://github.com/ethz-asl/mav_rtk_gps/wiki/Installing-and-Configuring-Piksi#settings-piksi-v2).
 - [Example Launch Files](https://github.com/ethz-asl/mav_rtk_gps/tree/master/piksi_rtk_gps#usage).

License
-------
The source code is released under a [BSD 3-Clause license](https://github.com/ethz-asl/mav_rtk_gps/blob/master/LICENSE).

Credits
-------
Marco Tranzatto, Michael Pantic, Kai Holtmann - ETHZ ASL & RSL - 17 July 2017

Based on the initial work of Daniel Eckert.

Contact
-------
Marco Tranzatto marcot(at)ethz.ch


Bugs & Feature Requests
-------
Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/mav_rtk_gps/issues).
