# A ROS2 wrapper for The Open Source Race Car Simulator (TORCS)

This is a ROS implementation of the client and driver from the [TORCS SCR C++ client](https://sourceforge.net/projects/cig/files/SCR%20Championship/Client%20C%2B%2B/). 
For the code in this repository to work, you need a [patched version](https://github.com/fmirus/torcs-1.3.7) of [torcs1.3.7](https://sourceforge.net/projects/torcs/). 
This has been tested with Ubuntu 24.04 and [ROS2 Jazzy](https://docs.ros.org/en/jazzy/index.html).
For reference, the old ROS1 version is still available in the [ros1 branch](https://github.com/fmirus/torcs_ros/tree/ros1).

## Installation 

 - Install the patched version of torcs1.3.7 according to its [installation instructions](https://github.com/fmirus/torcs-1.3.7)
 - Install ROS2 Jazzy according to the [installation instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) 
 - Clone this repository into your workspace and build it by running 
 ```bash
    colcon build --packages-up-to torcs_ros_bringup && source install/setup.bash
 ```

## Usage

### Run TORCS 

From the folder you cloned [torcs-1.3.7](https://github.com/fmirus/torcs-1.3.7) to, run 

```bash
   ./BUILD/bin/torcs
``` 

or 

```bash
   ./BUILD/bin/torcs -noisy
``` 
if you want noisy sensors (see the [SCR-Manual](https://arxiv.org/pdf/1304.1672.pdf) for details)

### Run the ROS components 

To start all nodes including the simple driver, the image publisher and rviz, run 

```bash
  ros2 launch torcs_ros_bringup torcs_ros_bringup_launch.py
``` 

If you want to use your own driver instead of the simple driver in ```torcs_ros_drive_ctrl```, run 

```bash
  ros2 launch torcs_ros_bringup torcs_ros_bringup_launch.py use_driver:=False
``` 

## Preview

![torcs_ros preview](media/torcs_ros.gif)
