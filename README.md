# piksi_rtk_gps
ROS node to read SBP messages from an attached Piksi RTK device.


Based on the work of Daniel Eckert: [Original repo](https://bitbucket.org/Daniel-Eckert/mav_localization).


## Dependencies
  * libsbp (tested wit: [libsbp 1.2.1](https://github.com/swift-nav/libsbp/tree/v1.2.1))
  
## Installation and Configuration
The following code will automatically download the required version of libsbp, install it and add its python subfolder to your PYTHONPATH by adding a line in your .bashrc file.

```
#from the repository folder
chmod +x install/configure_environment.sh  #make sure the script is excutable
./install/configure_environment.sh         # WARNING: as side effect this script will add a line to your .bashrc
                                           # that appends the python subfolder of this repo to your $PYTHONPATH variable.
```
