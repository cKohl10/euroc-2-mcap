---
title: "EuRoC MAV Dataset in Foxglove"
video_url: "TODO"
blog_post_url: "TODO"
visualize_url: "TODO"
short_description: "Drone SLAM of the MH01 Environment from EuRoC MAV"
---

# EuRoC MAV Dataset in Foxglove

This tutorial shows how to convert the EuRoC MAV dataset into an MCAP file for visualization in Foxglove.

## Run this tutorial locally:
1. Download one of the EuRoC environments from [ETH Zurich](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) as a directory of images and csvs. The Machine Hall 01 set is visualized in the tutorial.
2. Make sure to have a working [ROS distribution](https://docs.ros.org/en/humble/Installation.html) and install dependancies:
```bash
sudo apt install \
 ros-$ROS_DISTRO-foxglove-bridge \
    ros-$ROS_DISTRO-imu-tools \
 ros-$ROS_DISTRO-rtabmap-ros \
    ros-$ROS_DISTRO-rosbag2-storage-mcap
```
In a new python environment, install:
```bash
pip install foxglove-sdk numpy opencv-python pyyaml
```
3. Build the ROS 2 package called euroc_slam
```bash
cd ~/tutorials/EuRoC_MAV/ros2_ws
colcon build --symlink-install
source ~/tutorials/EuRoC_MAV/ros2_ws/install/setup.bash
```
4. Run SLAM and visualize in Foxglove:
```bash
ros2 launch euroc_slam firefly.launch.py
```