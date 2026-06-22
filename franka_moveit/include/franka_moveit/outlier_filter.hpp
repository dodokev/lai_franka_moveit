#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/search/kdtree.h>

class OutlierFilter : public rclcpp::Node
{
private:
    /* data */
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_unfilter;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_Xaxis;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_Yaxis;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_Zaxis;
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;

    pcl::PassThrough<pcl::PointXYZ> pass_Xaxis;
    pcl::PassThrough<pcl::PointXYZ> pass_Yaxis;
    pcl::PassThrough<pcl::PointXYZ> pass_Zaxis;
    pcl::VoxelGrid<pcl::PointXYZ> vg_;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_;

    std::vector<pcl::PointIndices> cluster_indices_;

    long unsigned int min_keep_size_;

    void filter_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);

public:
    OutlierFilter(/* args */);
    ~OutlierFilter() = default;
};