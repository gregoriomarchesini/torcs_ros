# torcs_ros_bringup

This package holds config and launch files to start the whole ROS machinery.

To start all nodes including the simple driver, the image publisher and rviz, run 

```bash
  ros2 launch torcs_ros_bringup torcs_ros_bringup_launch.py
``` 

If you want to use your own driver instead of the simple driver in ```torcs_ros_drive_ctrl```, run 

```bash
  ros2 launch torcs_ros_bringup torcs_ros_bringup_launch.py use_driver:=False
