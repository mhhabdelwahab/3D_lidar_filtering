#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL forward declarations
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>

// The LidarFiltering class definition
class LidarFiltering
{
public:
    // Constructor: we pass a NodeHandle to configure subscribers/publishers
    LidarFiltering(ros::NodeHandle &nh);

private:
    // The callback that processes incoming LiDAR data
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

    // ROS communication
    ros::Subscriber sub_;
    ros::Publisher  pub_filtered_;
    ros::Publisher  pub_removed_;

    // Tunable parameters (read from launch file)
    double plane_dist_threshold_;
    int    mean_k_;
    double stddev_mul_thresh_;
};