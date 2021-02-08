# Project MNiAC

## Pre-requisites

#### Firmware (tested) - Arducopter 
#### Python version - 2.xx
#### ROS version (tested) - Kinetic , Melodic
#### Mavros, Mavlink package - Latest

## PX4 - Setup

sudo apt-get install ros-melodic-geographic-msgs
sudo apt-get install libgeographic-dev 


git clone https://github.com/PX4/PX4-Autopilot.git --recursive


pip3 install --user toml
pip3 install --user empy
pip3 install --user jinja2
pip3 install --user packaging
pip3 install --user numpy
pip install pymavlink



make px4_sitl gazebo

sudo apt-get install libprotobuf-dev libprotoc-dev protobuf-compiler libeigen3-dev libxml2-utils python-rospkg python-jinja2
sudo apt-get install libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly -y


source ~/PX4-Autopilot/Tools/setup_gazebo.bash $(pwd)/PX4-Autopilot $(pwd)/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/PX4-Autopilot/Tools/sitl_gazebo

### PX4 Guide Flow
1. https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html#rosgazebo
2. https://docs.px4.io/master/en/ros/mavros_installation.html
3. https://docs.px4.io/master/en/dev_setup/building_px4.html
4. https://docs.px4.io/master/en/simulation/gazebo.html
5. https://docs.px4.io/master/en/simulation/ros_interface.html
6. https://docs.px4.io/master/en/ros/offboard_control.html
7. https://docs.px4.io/master/en/ros/mavros_offboard.html

## Square_path Node

Move in square path with user defined dimension by publishing twist msgs to /cmd_vel topic of mavros.

#### Subscribed To : 
* /mavros/global_position/raw/fix

#### Published To :
* /mavros/setpoint_velocity/cmd_vel

#### Services :
* /mavros/set_mode
* /mavros/cmd/arming
* /mavros/cmd/takeoff
* /mavros/cmd/land

