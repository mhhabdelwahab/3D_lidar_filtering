# 3D LiDAR Filtering

This ROS package removes ground and noise points from a LiDAR point cloud while preserving important features (e.g., buildings, trees).

## 1. Features
- **Ground removal** using RANSAC plane segmentation (PCL).
- **Noise removal** using Statistical Outlier Removal.
- Publishes two topics:
  - `/filtered_points`: The remaining LiDAR points (without ground, without noise).
  - `/removed_points`: The ground + noise points.

## 2. Requirements
- **Ubuntu 20.04** (or equivalent environment).
- **ROS Noetic**.
- **PCL** (via ROS: `pcl_ros`, `pcl_conversions`).

## 3. Build and Install
1. Clone the repository into your catkin workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/mhhabdelwahab/lidar_filtering_assignment.git

2. Build the workspace:
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash

3. Running the Node:

    roslaunch lidar_filtering_assignment lidar_filtering.launch

4. Play your bag file:

    rosbag play /path/to/LiDARFilteringAssignment.bag --loop
