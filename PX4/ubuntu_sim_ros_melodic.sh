#!/bin/bash

## Bash script for setting up ROS Melodic (with Gazebo 9) development environment for PX4 on Ubuntu LTS (18.04).
## It installs the common dependencies for all targets (including Qt Creator)
##
## Installs:
## - Common dependencies libraries and tools as defined in `ubuntu_sim_common_deps.sh`
## - ROS Melodic (including Gazebo9)
## - MAVROS

if [[ $(lsb_release -sc) == *"xenial"* ]]; then
  echo "OS version detected as $(lsb_release -sc) (16.04)."
  echo "ROS Melodic requires at least Ubuntu 18.04."
  echo "Exiting ...."
  return 1;
fi

echo "Downloading dependent script 'ubuntu_sim_common_deps.sh'"
# Source the ubuntu_sim_common_deps.sh script directly from github
common_deps=$(wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_common_deps.sh -O -)
wget_return_code=$?
# If there was an error downloading the dependent script, we must warn the user and exit at this point.
if [[ $wget_return_code -ne 0 ]]; then echo "Error downloading 'ubuntu_sim_common_deps.sh'. Sorry but I cannot proceed further :("; exit 1; fi
# Otherwise source the downloaded script.
. <(echo "${common_deps}")

# ROS Melodic
## Gazebo simulator dependencies
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y
pip install future


## Install rosinstall and other dependencies
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y



# MAVROS: https://dev.px4.io/en/ros/mavros_installation.html
## Install dependencies
sudo apt-get install python-catkin-tools python-rosinstall-generator -y


cd ~/catkin_ws
wstool init src


## Install MAVLink
###we use the Kinetic reference for all ROS distros as it's not distro-specific and up to date
rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall

## Build MAVROS
### Get source (upstream - released)
rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall

### Setup workspace & install deps
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
if ! rosdep install --from-paths src --ignore-src -y; then
    # (Use echo to trim leading/trailing whitespaces from the unsupported OS name
    unsupported_os=$(echo $(rosdep db 2>&1| grep Unsupported | awk -F: '{print $2}'))
    rosdep install --from-paths src --ignore-src --rosdistro melodic -y --os ubuntu:bionic
fi

if [[ ! -z $unsupported_os ]]; then
    >&2 echo -e "\033[31mYour OS ($unsupported_os) is unsupported. Assumed an Ubuntu 18.04 installation,"
    >&2 echo -e "and continued with the installation, but if things are not working as"
    >&2 echo -e "expected you have been warned."
fi

#Install geographiclib
sudo apt install geographiclib-tools -y
echo "Downloading dependent script 'install_geographiclib_datasets.sh'"
# Source the install_geographiclib_datasets.sh script directly from github
install_geo=$(wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh -O -)
wget_return_code=$?
# If there was an error downloading the dependent script, we must warn the user and exit at this point.
if [[ $wget_return_code -ne 0 ]]; then echo "Error downloading 'install_geographiclib_datasets.sh'. Sorry but I cannot proceed further :("; exit 1; fi
# Otherwise source the downloaded script.
sudo bash -c "$install_geo"

## Build!
catkin build
## Re-source environment to reflect new packages/build environment
catkin_ws_source="source ~/catkin_ws/devel/setup.bash"
if grep -Fxq "$catkin_ws_source" ~/.bashrc; then echo ROS catkin_ws setup.bash already in .bashrc;
else echo "$catkin_ws_source" >> ~/.bashrc; fi
eval $catkin_ws_source
