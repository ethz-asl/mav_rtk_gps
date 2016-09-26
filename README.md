# piksi_rtk_gps
ROS node to read SBP messages from an attached Piksi RTK device.


Based on the work of Daniel Eckert: [Original repo](https://bitbucket.org/Daniel-Eckert/mav_localization).


## Dependencies
  * Tested with libsbp 1.2.1: [Release on Github](https://github.com/swift-nav/libsbp/tree/v1.2.1).
  
**Warning**: For now DO NOT install libsbp using pip, but clone the libsbp repo, go in its python subfolder and
```
sudo python setup.py install
```
**Temporary Workaround**: update the launch file "piksi.launch" in this repo with the correct path of the python subfolder of libsbp in your local machine.
