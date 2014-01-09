HumaRobotics ROS Bindings for Dynamixel Motors
########################################################

HumaRobotics ROS Bindings for Dynamixel Motors is a ROS package that provides bindings to access Dynamixel motors. It relies on the HumaRobotics Dynamixel Library provided on https://github.com/HumaRobotics/dynamixel_hr .

.. toctree::
    :maxdepth: 1


Download
=======
You can obtain the latest sources using our Git repository:

git clone git@github.com/HumaRobotics:dynamixel_hr_ros

You can also download the latest release as a compressed archive from:
http://www.humarobotics.com/downloads/dynamixel_hr_ros.zip



Installation
============


Ubuntu
------
First make sure that you that you have installed the HumaRobotics Dynamixel Library from https://github.com/HumaRobotics/dynamixel_hr and ROS.

Make sure your environment variables are properly set, and that the dynamixel_hr_ros folder is set in the ROS_PACKAGE_PATH environment variable. Check this by running::

  env | grep ROS
    
Typical output would be::

  ROS_ROOT=/opt/ros/groovy/share/ros
  ROS_PACKAGE_PATH=/home/hr/ros-dynamixel/dynamixel_hr_ros:/opt/ros/groovy/share:/opt/ros/groovy/stacks
  ROS_MASTER_URI=http://localhost:11311
  ROSLISP_PACKAGE_DIRECTORIES=
  ROS_DISTRO=groovy
  ROS_ETC_DIR=/opt/ros/groovy/etc/ros

Before first usage, messages have to be built. You need to run the following from the dynamixel_hr_ros folder::

  rosmake

Usage
=============
We assume that you have a ROS master already running and pointed to in your ROS_MASTER_URI. If not you can start a local master by running::

  roscore
  
To start the Dynamixel ROS bindings, run::

  rosrun dynamixel_hr_ros expose.py --device /dev/ttyUSB0 --baudrate 3000000 --rate 10
  
replacing the device and baudrate according to your setup (you may first want to check your setup using the Dynamixel Lab). The rate option defines the ROS messages publishing rate.

Now you can try the two example scripts provided::

  rosrun dynamixel_hr_ros record.py
  
This will disable the motor chain so that you can move it by hand and record its position for 5 seconds. These positions can then be replayed by running::

  rosrun dynamixel_hr_ros replay.py
  
Speed is not preserved in this simple example.




Library
=======

The ROS bindings are provided by a node called /dxl. The three following topics are provided:
  * /dxl/chain_state: provides the current position of the motors
  * /dxl/command_position: listens for position (and optional speed) commands
  * /dxl/enable: listens for activation/deactivation messages

The /dxl/chain_state topic will provide messages about the current position of the motors in radians. The message structure is the following::

  int8[] id
  float32[] angle

The /dxl/enable topic listens to Bool messages that allows to activate (True) or deactivate (False) the entire motor chain.

The /dxl/command_position topic listens for messages that specifies angles to be reached by the motors and optionally a set of speeds::

  int8[] id
  float32[] angle
  float32[] speed
  
If no speed is provided then the last speed settings are used. Otherwise all speeds will be set at the same time as the goal position. To maximize performance,  synchronized write commands are used on the serial link.

