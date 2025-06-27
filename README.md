# Roller-Coaster-SLAM-2MCAP
MCAP conversion of the Roller Coaster SLAM dataset!
We want to visualize https://www.youtube.com/watch?v=g6IYMR6LCec in foxglove

## Prerequisites
1. SDK Installation: 
`pip install foxglove-sdk`

2. MCAP CLI installation https://mcap.dev/guides/cli

3. RTAB-map `sudo apt install ros-$ROS_DISTRO-rtabmap-ros`

4. Data uses an Oak-D camera, so we need depth-ai `sudo apt install ros-$ROS_DISTRO-depthai-ros`

4. Download dataset https://github.com/Factor-Robotics/Roller-Coaster-SLAM-Dataset/tree/main

## About the dataset
 - Formatted as rosbag

 - From original github
    - Recorded on OAK-D Pro W
    - Outputs: Stereo (640×400 @ 30 FPS) + IMU (BMI270 @ 100 Hz)
    - Allan Variance of BMI270 (explain what this is):

    ```yaml
    accelerometer_noise_density: 0.01 # [ m / s^2 / sqrt(Hz) ]
    accelerometer_random_walk: 0.001 # [ m / s^3 / sqrt(Hz) ]
    gyroscope_noise_density: 0.001 # [ rad / s / sqrt(Hz) ]
    gyroscope_random_walk: 0.0001 # [ rad / s^2 / sqrt(Hz) ]
    ```
    - Intrinsics and Extrinsics: See 'camera_info' and 'tf_static' topics
    - Same Boarding and Alighting points: False
    - Note: Due to the driver issue, the IMU timestamp has an offset of approximately 0.01 ms

## Outline
We will be using RTAB-map to do stero SLAM. Refer to this tutorial https://github.com/introlab/rtabmap/wiki/Stereo-mapping

Rough: bag 2 mcap -> topics 2 images w/ sdk -> images to slam with rtabmap -> data to mcap

## RTABMAP
Once the topics are playing, we need to align them with the following command
<!-- 
```
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/zed/zed_node/rgb/image_rect_color \
    depth_topic:=/zed/zed_node/depth/depth_registered \
    camera_info_topic:=/zed/zed_node/rgb/camera_info \
    frame_id:=zed_camera_center \
    approx_sync:=false \
    wait_imu_to_init:=true \
    imu_topic:=/zed/zed_node/imu/data \
    qos:=1 \
    use_sim_time:=true \
    rviz:=true
``` -->
Lets inspec the camera intrinsics! Open a raw input panel and inspect the camera_info topic
- Outputs a d, k, r, and p matrix. Stands for:
K = Camera intrinsic matrix -> Used this to calibrate camera
D = Distortion coefficients
R = Rectification Matrix R
P = Projection Matrix
baseline = -p[3] / fx

- Need to find these values for both the left and right cameras, use the raw messages panel
- Need to include the configs in the package, and also include the config folder:
`(os.path.join('share', package_name, 'config'), glob('config/*'))`

## MCAP to Images
Start by creating mcap files for the rosbags
`mcap convert seven_dwarfs_2023-12-06.bag seven-dwwarfs.mcap`
`mcap convert tron_2023-12-06.bag tron.mcap`
use `mcap info tron.mcap` to see how they are stored as ros1_msg

We need a directory of images for rtabmap, so we will use the foxglove sdk to save a right and left images directory


## Convert rosbag to MCAP with ROS
!!! This doesn't work because it is still in ros1 format !!!
use `mcap info tron.mcap` to see how they are stored as ros1_msg


## Convert rosbag to MCAP with SDK

## Euroc Dataset Test
Make sure to have `sudo apt install ros-humble-imu-tools`
`ros2 launch rtabmap_examples euroc_datasets.launch.py gt:=false`
`ros2 bag play MH_01_easy.db3 --clock` <- Clock is imprtant

## Load in the drone URDF without sdk
Create new package
Download urdf files
Launch file
Update setup.py
!!! Need to source the foxglove workspace before running foxglove_bridge!!!

