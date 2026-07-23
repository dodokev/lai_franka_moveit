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

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/transform.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

// Enumeration for shape type 
enum Shape { NONE, SPHERE, CYLINDER, BOX };

// Structure object to group the necessary component
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

/**
 * Node to find and create searched objects
 */
class ObjectFinder : public rclcpp::Node {
public:
  ObjectFinder();
  ~ObjectFinder() = default;

private:
  bool enable_create_{true};

  rclcpp::Service<franka_moveit_msg::srv::EnableCreate>::SharedPtr service_;
  void handle_service(
        const std::shared_ptr<franka_moveit_msg::srv::EnableCreate::Request> request,
        std::shared_ptr<franka_moveit_msg::srv::EnableCreate::Response> response);
  
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_centroid_;
  
  // Subscripter to add an object
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_size_;
  // Vector of searched objects
  std::vector<Object> objects_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_unfilter_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr table_;
  Eigen::Vector3d table_normal_;
  double table_d_;  

  // Coneverted point cloud of the sensor message 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  // Point cloud of the objects
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud_;
  // Vector of indices for each cluster
  std::vector<pcl::PointIndices> cluster_indices_;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;
  // Point cloud of a singular object
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud_;

  double penality_factor_{1.1};
  bool begin_{false}; // To avoid computation usage when no object is searched

  // Compute the mean 6d pose between two 6d poses
  Eigen::Affine3d poseMean(Eigen::Affine3d& p1, Eigen::Affine3d& p2);
  // Return true if the current pose is close enough to the old one
  bool closePose(Eigen::Affine3d& old, Eigen::Affine3d& current, double, double);

  void filter_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);
  void request_callback(const std_msgs::msg::String::SharedPtr msg);

  // Function to publish only the remaining cloud (without the object detected)
  void publishFilteredCloud(std_msgs::msg::Header& header, std::set<std::size_t>& leftover);

  // Separate the table and the object, and group points into cluster
  bool retreiveObject();

  void getCentroidAndOBB(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud,
                         Eigen::Vector4f& centroid,
                         pcl::PointXYZ& min,
                         pcl::PointXYZ& max,
                         pcl::PointXYZ& center,
                         Eigen::Matrix3f& rot);

  // ================================================================================================================================================================
  // === Scoring Function ===
  
  // Check the difference between OBB dimension detected and the known dimension
  double boundingScore(const std::vector<double>& dim, const std::vector<double>& ground_dim, const Shape& type);

  // Add the mean scalar value between the normal and the vector (point to centroid) 
  double normalScore(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud,
                     pcl::PointCloud<pcl::Normal>::Ptr normals,
                     Eigen::Vector4f& centroid);

  // Add fraction number of inliers and number of points -- for a plane fit
  double planeScore(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud);
  // Add how much two detected planes are perpendicular (if not enough/no planes detected, add 0 to the score)
  double cornerScore(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud);
  // Add fraction number of inliers and number of points -- for a cylinder fit
  double cylinderScore(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud,
                       pcl::PointCloud<pcl::Normal>::Ptr normals, const std::vector<double>& dim, const Shape& type);

  // ================================================================================================================================================================
  // === 6D pose computation ===
  
  Eigen::Affine3d centroidBias(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud, const std::vector<double>& dim, const Shape& type);
  Eigen::Affine3d centroidBiasBox(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud, const std::vector<double>& dim);
  Eigen::Affine3d centroidBiasCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud, const std::vector<double>& dim);
  
  // ================================================================================================================================================================
  // === Object creation function ===
  
  void createAllObjects();
  void createObstacle(Eigen::Affine3d& pose, const std::vector<double>& dim, const Shape& type, std::size_t& numero);
  void createBox(Eigen::Affine3d& pose, const std::vector<double>& dim, std::size_t& numero);
  void createCylinder(Eigen::Affine3d& pose, const std::vector<double>& dim, std::size_t& numero);
};