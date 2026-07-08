#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

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

#include "franka_moveit_msg/srv/enable_create.hpp"

enum Shape { NONE, SPHERE, CYLINDER, BOX };

struct Object {
  const Shape shape;
  const std::vector<double> dimension;
  
  unsigned int number;
  std::vector<Eigen::Affine3d> poses;
  std::vector<Eigen::Affine3d> candidates;

  std::vector<bool> have_stable;
  std::vector<bool> have_candidate;

  std::vector<int> confidences;

  Object(Shape& s, std::vector<double>& v) : shape(s), dimension(v), number(1) {}
};


class ObjectFinder : public rclcpp::Node {
 public:
  ObjectFinder(moveit::planning_interface::PlanningSceneInterface* psi);
  ~ObjectFinder() = default;

 private:
  bool enable_create_{true};

  rclcpp::Service<franka_moveit_msg::srv::EnableCreate>::SharedPtr service_;
  void handle_service(
        const std::shared_ptr<franka_moveit_msg::srv::EnableCreate::Request> request,
        std::shared_ptr<franka_moveit_msg::srv::EnableCreate::Response> response);
  
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_centroid_;
  
  /* data */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_unfilter_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_size_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr table_;
  Eigen::Vector3d table_normal_;
  double table_d_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud_;

  std::vector<pcl::PointIndices> cluster_indices_;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;

  std::vector<Object> objects_;

  moveit::planning_interface::PlanningSceneInterface* planning_scene_;


  double penality_factor_{1.1};
  bool begin_{false};

  Eigen::Affine3d poseMean(Eigen::Affine3d& p1, Eigen::Affine3d& p2);
  bool closePose(Eigen::Affine3d& old, Eigen::Affine3d& current, double, double);

  void filter_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);
  void request_callback(const std_msgs::msg::String::SharedPtr msg);

  bool retreiveObject();
  void getCentroidAndOBB(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud,
                         Eigen::Vector4f& centroid,
                         pcl::PointXYZ& min,
                         pcl::PointXYZ& max,
                         pcl::PointXYZ& center,
                         Eigen::Matrix3f& rot);

  double boundingScore(const std::vector<double>& dim, const std::vector<double>& ground_dim, const Shape& type);

  double normalScore(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud,
                     pcl::PointCloud<pcl::Normal>::Ptr normals,
                     Eigen::Vector4f& centroid);

  double planeScore(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud);
  double cornerScore(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud);
  double cylinderScore(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud,
                       pcl::PointCloud<pcl::Normal>::Ptr normals, const std::vector<double>& dim, const Shape& type);

  Eigen::Affine3d centroidBias(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud, const std::vector<double>& dim, const Shape& type);
  Eigen::Affine3d centroidBiasBox(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud, const std::vector<double>& dim);
  Eigen::Affine3d centroidBiasCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud, const std::vector<double>& dim);
    
  void createAllObjects();
  void createObstacle(Eigen::Affine3d& pose, const std::vector<double>& dim, const Shape& type, std::size_t& numero);
  void createBox(Eigen::Affine3d& pose, const std::vector<double>& dim, std::size_t& numero);
  void createCylinder(Eigen::Affine3d& pose, const std::vector<double>& dim, std::size_t& numero);
};