
This repository is based on the following repositories, considering some changes in the codes to have compatibility with ubuntu 20.4 and with Optitrack's Motion Capture system and in addition control algorithms applied to individual nano quadrotors and in swarm will be added.


*** Repository of the original unmodified codes

crazyflie_ros

https://github.com/whoenig/crazyflie_ros

vrpn

https://github.com/ros-drivers/vrpn_client_ros


## Crazyflie ros
Make sure you have installed the installation steps from the following repository.
https://github.com/juliordzcer/crazyflie-firmware/blob/tec/README.md


ROS stack for Bitcraze Crazyflie (http://www.bitcraze.se/), with the following features:

* Support for Crazyflie 1.0 and Crazyflie 2.0 (using stock firmware)
* Publishes on-board sensors in ROS standard message formats
* Supports ROS parameters to reconfigure crazyflie parameters
* Support for using multiple Crazyflies with a single Crazyradio
* Includes external controller for waypoint navigation (if motion capture system is available)
* No dependency to the Bitcraze SDK (Driver and Controller written in C++)

A tutorial (for a slightly older version) is available in W. Hönig and N. Ayanian. "Flying Multiple UAVs Using ROS", Chapter in Robot Operating System (ROS): The Complete Reference (Volume 2), Springer, 2017. (see http://act.usc.edu/publications.html for a free pre-print).

## Requirements
### Ros Noetic
#### Setup your sources.list
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
#### Set up your keys
```
sudo apt install curl # if you haven't already installed curl
```
```
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
#### Installation
```
sudo apt update
```
```
sudo apt install ros-noetic-desktop-full
```
#### Environment setup
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
#### Dependencies for building packages
```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```
##### Initialize rosdep
```
sudo apt install python3-rosdep
```
```
sudo rosdep init
rosdep update
```


To be able to execute the programs it is necessary to install the following dependencies, executing the following commands in the console
```
sudo apt-get update
```
```
sudo apt-get install ros-noetic-vrpn-client-ros -y
sudo apt-get install ros-noetic-joy -y
```
## Create a catkin workspace
```
mkdir -p ~/crazyflie_ws/src
cd crazyflie_ws/src
catkin_init_workspace
```
With these commands a new workspace called Crazyflie_ws has been created

## Installation

Clone the package into your catkin workspace (in src folder): 
```
cd
cd crazyflie_ws/src
git clone --recursive https://github.com/juliordzcer/crazyflie_ros.git
cd crazyflie_ros
git submodule init
git submodule update
cd
```

Use `catkin_make` on your workspace to compile.
```
cd crazyflie_ws
catkin_make
```

Finally run the following command in terminal
```
echo "source ~/crazyflie_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```


## Usage

There are five packages included: crazyflie, crazyflie_description, crazyflie_controller, crazyflie_demo and vrpn_client_ros.
Note that the below description might be slightly out-of-date, as we continue merging the Crazyswarm and crazyflie_ros.

### Crazyflie

This package contains the driver. In order to support multiple Crazyflies with a single Crazyradio, there is crazyflie_server (communicating with all Crazyflies) and crazyflie_add to dynamically add Crazyflies.
The server does not communicate to any Crazyflie initially, hence crazyflie_add needs to be used.

### Crazyflie_description

This package contains a 3D model of the Crazyflie (1.0). This is for visualization purposes in rviz.

### Crazyflie_controller

This package contains the position controller for trajectory tracking.

### Crazyflie_demo

This package contains a set of examples to quickly get started with Crazyflie.

To follow a trajectory using a crazyflie:
```
roslaunch crazyflie_demo Run_trajectory.launch uri:=radio://0/100/2M
```
where uri specifies the uri of your Crazyflie.

To start the path of two agents:
```
roslaunch crazyflie_demo Multiagents.launch
```
You can modify the crazyflie uri parameters as well as the VRPN parameters in the Multiagent.launch launch file.
located in the crazyflie_demo/launch folder

For multiple Crazyflies, make sure all Crazyflies have a different address.
Crazyflies sharing a dongle should use the same channel and data rate for the best performance.
Performance degrades with number of Crazyflies per dongle due to bandwidth limitations, however successfully tested to use 3 CF per Crazyradio.
### Vrpn_client_ros
This package contains the code for the external motion capture system, which has been modified to change the frames sent by the Optitrack system.
To know the position of a rigid body execute the following command

```
roslaunch vrpn_client_ros sample.launch server:=<ip>
rostopic echo /vrpn client node/<rigid body name>/pose
```

## ROS Features

### Parameters

The launch file supports the following arguments:
* uri: Specifier for the crazyflie, e.g. radio://0/80/2M
* tf_prefix: tf prefix for the crazyflie frame(s)
* roll_trim: Trim in degrees, e.g. negative if flie drifts to the left
* pitch_trim: Trim in degrees, e.g. negative if flie drifts forward

See http://wiki.bitcraze.se/projects:crazyflie:userguide:tips_and_tricks for details on how to obtain good trim values.

