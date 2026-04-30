# ngd_slam_ros

ROS 2 Humble RGB-D node for running NGD-SLAM from live or recorded image topics.

## Prerequisites

- Build NGD-SLAM from the repository root first:

```bash
./build.sh
```

- Source ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
```

## Build

From a colcon workspace that contains this repository under `src/`:

```bash
colcon build --packages-select ngd_slam_ros --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Run

```bash
ros2 launch ngd_slam_ros rgbd.launch.py \
  voc_file:=/absolute/path/to/NGD-SLAM/Vocabulary/ORBvoc.txt \
  settings_file:=/absolute/path/to/NGD-SLAM/Examples/RGB-D/RealSense_D455.yaml \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/aligned_depth_to_color/image_raw
```

## Publish D455 Frames

The package also includes a ROS 2 node that opens a RealSense D455 directly and publishes:

- `/camera/color/image_raw`
- `/camera/aligned_depth_to_color/image_raw`
- `/camera/color/camera_info`
- `/camera/aligned_depth_to_color/camera_info`

Run it with:

```bash
ros2 launch ngd_slam_ros realsense_d455_rgbd.launch.py
```

To target a specific device:

```bash
ros2 launch ngd_slam_ros realsense_d455_rgbd.launch.py serial_no:=<device-serial>
```

## Publish TUM RGB-D as ROS 2 Topics

```bash
python3 scripts/publish_tum_rgbd.py \
  /absolute/path/to/NGD-SLAM/data/tum/rgbd_dataset_freiburg3_walking_xyz \
  /absolute/path/to/NGD-SLAM/Examples/RGB-D/associations/fr3_walk_xyz.txt
```

## Published Topics

- `pose_topic` (`geometry_msgs/msg/PoseStamped`): camera pose in the `map_frame`
- `tracking_state_topic` (`std_msgs/msg/Int32`): raw NGD-SLAM tracking state

## Parameters

- `voc_file`: required path to `ORBvoc.txt`
- `settings_file`: required camera/settings YAML
- `rgb_topic`: RGB image topic
- `depth_topic`: depth image topic registered to RGB
- `pose_topic`: pose output topic
- `tracking_state_topic`: tracking state output topic
- `map_frame`: pose frame id
- `camera_frame`: reserved for downstream use
- `queue_size`: approximate-sync queue size
- `use_viewer`: enable Pangolin viewer
- `save_keyframe_trajectory`: optional output path written on shutdown
- `save_camera_trajectory`: optional output path written on shutdown
