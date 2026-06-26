#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/crop_box.h>

class ObjectRemover : public rclcpp::Node {
 public:
  ObjectRemover(moveit::planning_interface::PlanningSceneInterface* ps);
  ~ObjectRemover() = default;

 private:
  /* data */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_unfilter_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_;

  moveit::planning_interface::PlanningSceneInterface* planning_scene_;

  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);  

  pcl::PointCloud<pcl::PointXYZ>::Ptr remover(pcl::PointCloud<pcl::PointXYZ>::Ptr current, moveit_msgs::msg::CollisionObject& obj_msg, const std::string& parent="");
};