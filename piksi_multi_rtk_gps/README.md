piksi_multi_rtk_gps
======
ROS node to read SBP messages from an attached Piksi **Multi** RTK device.

The piksi_multi_rtk_gps package has been tested under [ROS] Indigo and Ubuntu 14.04, and [ROS] Kinetic and Ubuntu 16.04. The latest version relies on Piksi Multi Firmware **1.2** release.
  
## Installation and Configuration

### Dependencies
  * python-pip `sudo apt-get install python-pip`
  * python-tox `sudo apt-get install python-tox`
  * pandoc     `sudo apt-get install pandoc`
  
### Installation
**WARNING: install __ONLY ONE__ version of SBP library, depending of which Hardware version you are using. This page cointains the driver for [Piksi Multi](https://www.swiftnav.com/piksi-multi).
If you are using [Piksi V2](http://docs.swiftnav.com/pdfs/piksi_datasheet_v2.3.1.pdf) please check its driver version: [piksi_rtk_gps](https://github.com/ethz-asl/mav_rtk_gps/tree/master/piksi_rtk_gps)** (it is not supported anymore).

The following code will automatically download the required version of libsbp and install it in the default folder `/usr/local/lib/python2.7/dist-packages/sbp-2.2.1-py2.7.egg/sbp/`.

```
# From the repository folder
source install/install_piksi_multi.sh
```

## Usage
Make sure you configured your Piksi(s) by following [these instructions](https://github.com/ethz-asl/mav_rtk_gps/wiki/Installing-and-Configuring-Piksi#settings-piksi-multi).

**Base station**

If you have already configured Piksi Multi to act as a bse station (and sampled its position), then simply launch the following file.
```
roslaunch piksi_multi_rtk_gps piksi_multi_base_station.launch
```
If you have already configured Piksi Multi to act as a bse station (and did **NOT** sample its position), then launch the following file.
Once the survey is over kill the launched node and then start the Piksi Multi driver for the base station, by using the above instructions.
```
roslaunch piksi_multi_rtk_gps geodetic_survey.launch
```

**MAV**
```
roslaunch piksi_multi_rtk_gps piksi_multi_mav.launch
```
Check the status of your receiver (RQT GUI):
```
roslaunch rqt_gps_rtk_plugin gui.launch
```

Check the status of your receiver (Python GUI):
```
roslaunch mav_rtk_gui rtk_multi_info_example.launch
```

## Corrections Over Wifi
It is possible to send/receive corrections over Wifi between multiple Piksi modules.
Set configurations in `cfg/piksi_multi_driver_settings.yaml`
- Base station configuration (sends corrections)
 - `base_station_mode: true`
 - `broadcast_addr: <put here Bcast address obtained from command ifconfig>`
 - `broadcast_port: 26078`
- Rover configuration (receives corrections)
 - `base_station_mode: false`
 - `broadcast_addr: <put here Bcast address obtained from command ifconfig>`
 - `broadcast_port: 26078`
 - `base_station_ip_for_latency_estimation: <IP address of base station or router, to estimate latency over wifi>`

## Rosservice
The following services are available:
 - `piksi/settings_write`: writes a specific setting to Piksi Multi;
 - `piksi/settings_read_req`: requests a parameter to Piksi Multi;
 - `piksi/settings_read_resp`: reads the latest response of a "settings_read_req" request;
 - `piksi/settings_save`: saves the current settings of Piksi Multi to its flash memory.