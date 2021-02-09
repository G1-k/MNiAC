# Project MNiAC

### Platform:
##### - Firmware (tested) - PX4 
##### - Python version - 2.xx
##### - ROS version (tested) - Melodic
##### - Mavros, Mavlink package - Latest


## PX4 SITL - Setup

## 1.Pre-requisite

#### Install Dependencies
```
sudo apt-get install ros-melodic-geographic-msgs
sudo apt-get install libgeographic-dev 
sudo apt-get install libprotobuf-dev libprotoc-dev protobuf-compiler libeigen3-dev libxml2-utils python-rospkg python-jinja2
sudo apt-get install libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly -y
pip3 install --user toml
pip3 install --user empy
pip3 install --user jinja2
pip3 install --user packaging
pip3 install --user numpy
pip install pymavlink
```
#### 1.1 Create a ROS Workspace
1. Create
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
```
2. Add source in .bashrc
``` source ~/catkin_ws/devel/setup.bash```

#### 1.2 Install MAVLink, Mavros (Other ROS Dependencies)

Before this, make sure `~/catkin_ws/` is present

1. Go to folder where ubuntu_sim_ros_melodic.sh is present
```
bash ubuntu_sim_ros_melodic.sh
```

NOTE: ubuntu_sim_ros_melodic.sh present in this repo doesnt install ros-melodic,gazebo etc. It only install mavlink,mavros and its dependencies

## 2.SITL Setup 

1. Clone PX4 repo
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

2. cd to PX4 Repo, then Build 

```
make px4_sitl gazebo
```

3. Source your workspace and add gazebo path
* ```gedit ~/.bashrc```
* In .bashrc, Add below lines and save it
```
source ~/PX4-Autopilot/Tools/setup_gazebo.bash $(pwd)/PX4-Autopilot $(pwd)/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/PX4-Autopilot/Tools/sitl_gazebo
```

## 3. Getting Started

1. Give a star to this repo (at the top) 

2. Clone the repository

- `cd ~/catkin_ws/src`
- `https://github.com/G1-k/MNiAC.git`

3. Build the workspace
```
cd ..
catkin build
```

### 3.1 Node Position Control

Move in square path of user defined dimension by publishing Pose msgs to  'mavros/setpoint_position/local' topic of mavros.

1. Launch PX4 SITL along with Gazebo and MAVROS
``` roslaunch px4 mavros_posix_sitl.launch```

2. Run the position_control node
``` rosrun px4_examples node_position_control.py```

### 3.2 Node Velocity Control

Move in circular path of user defined dimension by publishing TwistStamped msgs to  'mavros/setpoint_velocity/cmd_vel' topic of mavros.

1. Launch PX4 SITL along with Gazebo and MAVROS
``` roslaunch px4 mavros_posix_sitl.launch```

2. Run the velocity_control node
``` rosrun px4_examples node_velocity_control.py```


## 4. Info
#### Published To Topics:
#### 1.Position Control
* mavros/setpoint_position/local

#### 2.Velocity Control
* mavros/setpoint_velocity/cmd_vel

#### 3.Acceleration Control
* mavros/setpoint_acceleration/accel

##### Services Used:
* /mavros/set_mode
* /mavros/cmd/arming
* /mavros/cmd/takeoff
* /mavros/cmd/land

#### (DOCS)PX4 Dev Setup and Starting Guide (stepwise)
1. https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html#rosgazebo
2. https://docs.px4.io/master/en/ros/mavros_installation.html
3. https://docs.px4.io/master/en/dev_setup/building_px4.html
4. https://docs.px4.io/master/en/simulation/gazebo.html
5. https://docs.px4.io/master/en/simulation/ros_interface.html
6. https://docs.px4.io/master/en/ros/offboard_control.html
7. https://docs.px4.io/master/en/ros/mavros_offboard.html

#### USEFUL LINKS:
1. Python examples
https://github.com/PX4/PX4-Autopilot/tree/master/integrationtests/python_src/px4_it/mavros