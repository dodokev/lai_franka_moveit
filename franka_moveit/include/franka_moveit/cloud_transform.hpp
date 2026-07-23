#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

/**
 * Node to transmit a point cloud from a frame to another
 */
class CloudToWorld : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    // tf2 member to obtain the transformation between frames
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Input cloud topic
    std::string input_topic_;
    // Output cloud topic
    std::string output_topic_;

    // New frame of the point cloud
    std::string target_frame_;    

    void callback(const sensor_msgs::msg::PointCloud2);

public:
    CloudToWorld(/* args */);
    ~CloudToWorld() = default;

    std::string getTargetFrame()
    {
        return target_frame_;
    }
};