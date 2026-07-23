#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/crop_box.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/search/kdtree.h>

/**
 * From a point cloud, get the collision object (and attached) of a moveit planning scene to remove corresponding points
 */
class ObjectRemover : public rclcpp::Node {
public:
  ObjectRemover(moveit::planning_interface::PlanningSceneInterface* ps);
  ~ObjectRemover() = default;

  // Initialization of certain member and monitor
  void init();
private: 
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_unfilter_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_;

  moveit::planning_interface::PlanningSceneInterface* planning_scene_;

  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);  

  // Remove with pcl cropBox with a padding
  pcl::PointCloud<pcl::PointXYZ>::Ptr remover(pcl::PointCloud<pcl::PointXYZ>::Ptr current, moveit_msgs::msg::CollisionObject& obj_msg, const std::string& parent="");
  // Remove with pcl clustering
  pcl::PointCloud<pcl::PointXYZ>::Ptr removerByCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr current, moveit_msgs::msg::CollisionObject& obj_msg, const std::string& parent="");

  // ================================================================================================================================================================
  // === Point insider check ===
  bool pointInsideBox(
    const pcl::PointXYZ& p,
    const shape_msgs::msg::SolidPrimitive& primitive,
    const geometry_msgs::msg::Pose& pose);
    
  bool pointInsideCylinder(
    const pcl::PointXYZ& p,
    const shape_msgs::msg::SolidPrimitive& primitive,
  const geometry_msgs::msg::Pose& pose);
  
  bool pointInsideCollisionObject(
    const pcl::PointXYZ& p,
    const moveit_msgs::msg::CollisionObject& obj, const std::string& parent);
    
  // ================================================================================================================================================================
  // Member needed to get the planning scene, and compute position for attached object
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
};