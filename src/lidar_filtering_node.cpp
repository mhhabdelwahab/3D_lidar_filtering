#include "lidar_filtering.hpp"

// PCL + ROS bridging
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>

#include <string>

LidarFiltering::LidarFiltering(ros::NodeHandle &nh)
{
    // 1) Load parameters from the private node handle (~param)
    nh.param<double>("plane_distance_threshold", plane_dist_threshold_, 0.2);
    nh.param<int>("mean_k", mean_k_, 50);
    nh.param<double>("stddev_mul_thresh", stddev_mul_thresh_, 1.0);

    // 2) Subscribe to a LiDAR topic (possibly loaded from param)
    std::string lidar_topic;
    nh.param<std::string>("lidar_topic", lidar_topic, "/mbuggy/os1/points");
    sub_ = nh.subscribe(lidar_topic, 1, &LidarFiltering::cloudCallback, this);

    // 3) Advertise output topics
    pub_filtered_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 1);
    pub_removed_  = nh.advertise<sensor_msgs::PointCloud2>("/removed_points", 1);

    ROS_INFO("LidarFiltering initialized. plane_dist=%.2f, mean_k=%d, stddev=%.2f, topic=%s",
             plane_dist_threshold_, mean_k_, stddev_mul_thresh_, lidar_topic.c_str());
}

void LidarFiltering::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    // ----------------------------------------------------------------------------
    // 1) Convert ROS->PCL
    // ----------------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud_in);

    // Debug: Show how many points in the incoming cloud
    ROS_INFO_STREAM("Received cloud with " << cloud_in->size() 
                    << " points. Frame ID: " << cloud_msg->header.frame_id);

    // ----------------------------------------------------------------------------
    // 2) Segment the largest plane -> assume it's the ground
    // ----------------------------------------------------------------------------
    // pass-through filter to keep only points within [−1.0,+1.0][−1.0,+1.0] (for example) in Z before running RANSAC
    // pcl::PassThrough<pcl::PointXYZ> pass;
    // pass.setInputCloud(cloud_in);
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(-2.0, 50.0); // Adjust to your environment
    // pass.filter(*cloud_in);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(plane_dist_threshold_);   // e.g., 0.2
    seg.setInputCloud(cloud_in);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    seg.segment(*inliers, *coefficients);

    // Debug: Check if a plane was actually found
    if (inliers->indices.empty())
    {
        ROS_WARN("No plane found. Skipping frame!");
        // Publish empty clouds or simply return without publishing
        return;
    }

    ROS_INFO_STREAM("Plane found with " << inliers->indices.size() 
                    << " inliers out of " << cloud_in->size() 
                    << " total points.");

    // ----------------------------------------------------------------------------
    // 3) Extract ground (inliers) and non-ground (outliers)
    // ----------------------------------------------------------------------------
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);

    // Ground
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setNegative(false); // false => extract inliers => ground
    extract.filter(*ground_cloud);

    // Non-ground
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setNegative(true);  // true => remove inliers => keep everything else
    extract.filter(*ground_removed_cloud);

    ROS_INFO_STREAM("  -> ground_cloud has " << ground_cloud->size() << " points.");
    ROS_INFO_STREAM("  -> above_ground_cloud has " << ground_removed_cloud->size() << " points.");

    // ----------------------------------------------------------------------------
    // 4) Noise Removal (Statistical Outlier Removal) on the "above_ground_cloud"
    // ----------------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr clean_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(ground_removed_cloud);
    sor.setMeanK(mean_k_);                // e.g. 50
    sor.setStddevMulThresh(stddev_mul_thresh_);  // e.g. 1.0
    sor.filter(*clean_cloud);

    // Debug: how many points remain after outlier removal
    ROS_INFO_STREAM("  -> after SOR, clean_cloud has " 
                     << clean_cloud->size() << " points.");

    // Retrieve outliers for debugging (noise)
    pcl::PointCloud<pcl::PointXYZ>::Ptr noise_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    sor.setNegative(true);
    sor.filter(*noise_cloud);
    ROS_INFO_STREAM("  -> noise_cloud has " << noise_cloud->size() 
                    << " outlier points removed by SOR.");

    // ----------------------------------------------------------------------------
    // 5) Merge ground + noise => "removed_points"
    // ----------------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr removed_points(new pcl::PointCloud<pcl::PointXYZ>);
    *removed_points = *ground_cloud + *noise_cloud;  // PCL operator+

    ROS_INFO_STREAM("  -> removed_points total " << removed_points->size() 
                    << " (ground + noise).");

    // ----------------------------------------------------------------------------
    // 6) Publish the final results
    // ----------------------------------------------------------------------------
    //   /filtered_points => just the "clean_cloud" (above ground, no outliers)
    //   /removed_points => ground + noise
    sensor_msgs::PointCloud2 filtered_msg;
    sensor_msgs::PointCloud2 removed_msg;

    // Convert PCL->ROS
    pcl::toROSMsg(*clean_cloud, filtered_msg);
    pcl::toROSMsg(*removed_points, removed_msg);

    // Retain the original header so RViz can transform it properly
    filtered_msg.header = cloud_msg->header;
    removed_msg.header  = cloud_msg->header;

    // Publish
    pub_filtered_.publish(filtered_msg);
    pub_removed_.publish(removed_msg);

    // Debug final message
    ROS_INFO_STREAM("Published /filtered_points (" << clean_cloud->size() 
                    << " pts) and /removed_points (" << removed_points->size() << " pts)");
}

// The ROS node's entry point
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_filtering");
    ros::NodeHandle nh("~");

    LidarFiltering node(nh);

    ros::spin();
    return 0;
}