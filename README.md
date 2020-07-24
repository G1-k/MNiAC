# Project MNiAC

## Pre-requisites

#### Firmware (tested) - Arducopter 
#### Python version - 2.xx
#### ROS version (tested) - Kinetic , Melodic
#### Mavros, Mavlink package - Latest


## Square_path Node

Move in square path with user defined dimension by publishing twist msgs to /cmd_vel topic of mavros.

#### Subscribers : 
* /mavros/global_position/raw/fix

#### Publishers :
* /mavros/setpoint_velocity/cmd_vel

#### Services :
* /mavros/set_mode
* /mavros/cmd/arming
* /mavros/cmd/takeoff
* /mavros/cmd/land

