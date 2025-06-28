# Foxglove SLAM Vizualization
Foxglove is a data visualization app that streamlines multimodal robotics workflows through the universal [MCAP](https://mcap.dev/) data format. Please refer to the original article [here](TODO). In this tutorial, we will convert the EuRoC MAV dataset, one of the most pominent and standardized simultaneous localization and mapping (SLAM) datasets, to MCAP format and visualize it through the [Foxglove Web App](https://app.foxglove.dev)
![Foxglove Euroc Viz](imgs/far.png)

## Sections
 - Intro to the dataset and mcap
 - Describe the flow of the article
 - Prerequisites
 - RTABMAP and SLAM
 - Drag, drop, you're done!
 - Data conversion
 ---> Bulk of writup
 - Create ros package
 - URDF Setup
 - Foxglove panels
 - Conclude with visualization

## 3 Levels
 - convert bag file directly
 - manual conversion
 - visualize the drone

## Prerequisites
 - `pip install foxglove-sdk`
 - Install mcap
 - Working ROS2 distro. Humble used in this tutorial
 - `sudo apt install ros-$ROS_DISTRO-rtabmap-ros`
 - `sudo apt install ros-$ROS_DISTRO-imu-tools`
 - `sudo apt install rosbags`
 - `sudo apt instal ros-$ROS_DISTRO-foxglove-bridge`

## RTABMAP
 - ROS2 SLAM package
 - Camera calibration
 - launch file

## Command Line Version
 - Download .bag file
 - rtabmap interfaces with ros2, so we convert it to a db3 format
 - `rosbags-convert --src euroc.bag --dst euroc`
 - Convert to mcap `mcap convert euroc.bag euroc.mcap` or `mcap convert euroc.db3 euroc.mcap`
    - Keep in mind, all the topics are still stored as ros1msg or ros2msg type in an mcap, so converting the bag to a db3 beforehand will allow it to interface with rtabmap.
 - Open the foxglove bridge for live visualization: `ros2 run foxglove_bridge foxglove_bridge`
 - Run rtabmap euroc example `ros2 launch rtabmap_examples euroc_datasets.launch.py gt:=false` for machine hall datasets without ground truth data and `ros2 launch rtabmap_examples euroc_datasets.launch.py gt:=true` for vicon datasets with ground truth.
 - Replay the mcap topics in real time to interface with rtabmap `ros2 bag play euroc.mcap --clock -r 1`
    - The `-r` argument sets the replay *rate*, meaning data can be sped up or slowed down. This may be useful if rtabmap is losing accuracy due to processing speeds.
 - Open the foxglove websocket or desktop app, open a connection, and view SLAM in real time! This example launch file comes preloaded with the calibrations and transformations needed to start visualizing this dataset.

## Data conversion
 - What if the data is not processed in bag format? Euroc is one of the more popular standards in csv and image format.
 - With the foxglove sdk, we can convert time synced datasets to self contained mcap files that can be visualized or played back in real time.
``` 
imu0
├── data.csv
└── sensor.yaml
cam<camera_number>
├── data
│   ├── <timestamp>.png
│   └── ...
├── data.csv
└── sensor.yaml
...
```
We want to convert this infomation to the following topics to interface with rtabmap:
```
/cam0/image_raw -> sensor_msgs/msg/Image
/cam1/image_raw -> sensor_msgs/msg/Image
/imu0 -> sensor_msgs/msg/Imu
```

### Prerequisites
`pip install foxglove-sdk numpy opencv-python pyyaml`

```python
import foxglove
import csv
import cv2
import yaml
import numpy as np
from foxglove import Schema, Channel
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3
from rclpy.serialization import serialize_message
```

### Main Function
Create a writer
```python
writer = foxglove.open_mcap(out_mcap_path, allow_overwrite=True) # Open the mcap file for writing
```

Define the schemas and channels
```python
# Direct foxglove to our message definitions
imu_msg_path = Path("msgs/imu_flat.msg")
img_msg_path = Path("msgs/image_flat.msg")

# Define our custom ros2 schemas
img_schema = Schema(
    name="sensor_msgs/msg/Image",
    encoding="ros2msg",
    data=img_msg_path.read_bytes(),
)
imu_schema = Schema(
    name="sensor_msgs/msg/Imu",
    encoding="ros2msg",
    data=imu_msg_path.read_bytes(),
)

# ros2msg channels require cdr encoding type
cam0_channel = Channel(topic="/cam0/image_raw", schema=img_schema, message_encoding="cdr")
cam1_channel = Channel(topic="/cam1/image_raw", schema=img_schema, message_encoding="cdr")
imu_channel = Channel(topic="/imu0", schema=imu_schema, message_encoding="cdr")

# Iterate through the data and save!
read_images(cam0_dir, cam0_channel)
print("Done writing cam0")
read_images(cam1_dir, cam1_channel)
print("Done writing cam1")
read_imu(imu_dir, imu_channel, imu_yaml_path)
print("Done writing imu")
```
The foxglove sdk requires that ros2 messages are formatted in a deliminated concatenated format. This format is nearly captured using built in ros2 tools such as `ros2 interface show sensor_msgs/msg/Image --no-comments`. More information about schema formatting is available [here](https://mcap.dev/spec/registry). For convenience, [these message definitions](https://github.com/cKohl10/euroc-2-mcap/tree/main/msgs) can be directly copied and pasted into a new `msgs` folder in your working directory.

### Image Writer
We will loop through the directory of images and log them in out image channel
```python
def getImageMsg(img_path: str, timestamp: int) -> Image:
    # Load as grayscale image data
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError(f"Could not load image: {img_path}")
    
    height, width = img.shape
    
    sec = int(timestamp // 1e9)
    nsec = int(timestamp % 1e9)

    # Fill in the image message data
    ros_image = Image()
    ros_image.header = Header()
    ros_image.header.stamp.sec = sec
    ros_image.header.stamp.nanosec = nsec
    ros_image.header.frame_id = "cam0"
    ros_image.height = height
    ros_image.width = width
    ros_image.encoding = "mono8" #Stereo images
    ros_image.step = width  # For mono8, 1 byte per pixel
    ros_image.data = img.tobytes()

    return ros_image

def read_images(cam_directory, channel):
    # Loop through the data.csv file and read in the image files
    with open(cam_directory + "/data.csv", "r") as csv_file:
        reader = csv.reader(csv_file)
        next(reader)  # Skip the first row with headers
        for row in reader:
            timestamp = int(row[0])
            image_name = row[1]
            image_path = os.path.join(cam_directory, "data", image_name)
            if not os.path.exists(image_path):
                print(f"Image {image_path} does not exist")
                continue

            # Convert image to ROS2 message and write to channel
            image_msg = getImageMsg(image_path, timestamp)
            channel.log(serialize_message(image_msg), log_time=timestamp)
```

### IMU Writer
```python
def read_imu(imu_data_path, imu_channel, imu_yaml_path):
    '''
    IMU data is in the format:
    timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]

    We will convert this to a custom IMU message that interfaces with sensor_msgs/msgs/Imu.
    This will be in the form:
     - header (header)
     - orientation (quaternion)
     - orientation covariance (float32[9])
     - angular velocity (vector3)
     - angular velocity covariance (float32[9])
     - linear acceleration (vector3)
     - linear acceleration covariance (float32[9])
    '''

    # Get the IMU config with covariance information
    with open(imu_yaml_path, "r") as imu_yaml_file:
        imu_yaml = yaml.load(imu_yaml_file, Loader=yaml.FullLoader)

    # Get the noise and bias parameters, see https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model for more details
    sample_sqr_dt = np.sqrt(1.0/float(imu_yaml["rate_hz"]))
    sigma_gd = imu_yaml["gyroscope_noise_density"]*sample_sqr_dt
    sigma_ad = imu_yaml["accelerometer_noise_density"]*sample_sqr_dt

    # Calculate the covariance matrices
    orientation_cov = np.zeros((3,3), dtype=np.float64)
    angular_velocity_cov = np.diag([sigma_gd**2, sigma_gd**2, sigma_gd**2]).astype(np.float64)
    linear_acceleration_cov = np.diag([sigma_ad**2, sigma_ad**2, sigma_ad**2]).astype(np.float64)

    with open(imu_data_path, "r") as imu_file:
        reader = csv.reader(imu_file)
        next(reader)  # Skip the first row with headers
        for row in reader:
            timestamp = int(row[0])
            angular_velocity = [float(row[1]), float(row[2]), float(row[3])]
            linear_acceleration = [float(row[4]), float(row[5]), float(row[6])]

            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp.sec = timestamp // int(1e9)
            imu_msg.header.stamp.nanosec = timestamp % int(1e9)
            imu_msg.header.frame_id = "imu4" # Transformation reference frame
            # Orientation
            imu_msg.orientation = Quaternion()
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = 0.0
            imu_msg.orientation.w = 1.0
            imu_msg.orientation_covariance = list(orientation_cov.flatten(order="C").astype(float))
            # Angular velocity
            imu_msg.angular_velocity = Vector3()
            imu_msg.angular_velocity.x = angular_velocity[0]
            imu_msg.angular_velocity.y = angular_velocity[1]
            imu_msg.angular_velocity.z = angular_velocity[2]
            imu_msg.angular_velocity_covariance = list(angular_velocity_cov.flatten(order="C").astype(float))
            # Linear acceleration
            imu_msg.linear_acceleration = Vector3()
            imu_msg.linear_acceleration.x = linear_acceleration[0]
            imu_msg.linear_acceleration.y = linear_acceleration[1]
            imu_msg.linear_acceleration.z = linear_acceleration[2]
            imu_msg.linear_acceleration_covariance = list(linear_acceleration_cov.flatten(order="C").astype(float))

            imu_channel.log(serialize_message(imu_msg), log_time=timestamp)
```
The full code is available [here](https://github.com/cKohl10/euroc-2-mcap)!

## Create a new ros2 package
 - `mkdir -p custom_ros_ws/src`
 - `cd src`
 - `ros2 pkg create --build-type ament_python --license Apache-2.0 euroc_slam`
 - `cd euroc_slam`

## 
