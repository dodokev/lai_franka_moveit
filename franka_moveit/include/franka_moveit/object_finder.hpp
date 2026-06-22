#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/search/kdtree.h>

#include <pcl/common/centroid.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include <visualization_msgs/msg/marker.hpp>

class ObjectFinder : public rclcpp::Node {
 public:
  enum class SHAPE { NONE, SPHERE, CYLINDER, BOX };
  ObjectFinder(moveit::planning_interface::PlanningSceneInterface* ps);
  ~ObjectFinder() = default;

 private:
  /* data */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_unfilter_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_centroid_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr table_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_;

  std::vector<pcl::PointIndices> cluster_indices_;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;

  std::vector<double> object_size_;
  SHAPE type_object_;

  double penality_factor_{1.1};

  void filter_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);
  void retreiveObject();
  void getCentroidAndOBB(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud,
                         Eigen::Vector4f& centroid,
                         pcl::PointXYZ& min,
                         pcl::PointXYZ& max,
                         pcl::PointXYZ& center,
                         Eigen::Matrix3f& rot);

  double boundingScore(std::vector<double> dim);

  double normalScore(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud,
                     pcl::PointCloud<pcl::Normal>::Ptr normals,
                     Eigen::Vector4f& centroid);

  double planeScore(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud);
  double cornerScore(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud);
  double cylinderScore(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud,
                       pcl::PointCloud<pcl::Normal>::Ptr normals);

  // Planning Scene Interface
  moveit::planning_interface::PlanningSceneInterface* planning_scene_;

  void centroidBias(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud, Eigen::Affine3d& pose);
  void centroidBiasBox(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud, Eigen::Affine3d& pose);
  void centroidBiasCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud, Eigen::Affine3d& pose);
    
  void createObstacle(Eigen::Affine3d& pose);
  void createBox(Eigen::Affine3d& pose);
  void createCylinder(Eigen::Affine3d& pose);

  bool obj_created{false};
  std::string obj_name_;
};