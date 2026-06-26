#include "franka_moveit/object_remover.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

static auto const LOGGER = rclcpp::get_logger("object_remover");

ObjectRemover::ObjectRemover(moveit::planning_interface::PlanningSceneInterface* ps)
    : Node("object_finder"), planning_scene_(ps) {
  
  sub_unfilter_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/points_used", rclcpp::SensorDataQoS(),
    std::bind(&ObjectRemover::callback, this, std::placeholders::_1));
  pub_filtered_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/octocloud",
    rclcpp::SensorDataQoS());
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ObjectRemover::remover(pcl::PointCloud<pcl::PointXYZ>::Ptr current, moveit_msgs::msg::CollisionObject& obj_msg, const std::string&) {
  const double margin = 0.03; // e.g. 0.01
  pcl::CropBox<pcl::PointXYZ> crop;
  crop.setInputCloud(current);
  crop.setNegative(true);
  
  const auto& pose = obj_msg.pose;
  crop.setTranslation(Eigen::Vector3f(pose.position.x, pose.position.y, pose.position.z));
  
  tf2::Quaternion tf_q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  crop.setRotation(Eigen::Vector3f(roll, pitch, yaw));

  if(obj_msg.primitives[0].type == shape_msgs::msg::SolidPrimitive::BOX)
  {
    double sx = obj_msg.primitives[0].dimensions[0];
    double sy = obj_msg.primitives[0].dimensions[1];
    double sz = obj_msg.primitives[0].dimensions[2];

    crop.setMin(Eigen::Vector4f(-sx/2 - margin, -sy/2 - margin, -sz/2 - margin, 1.0));
    crop.setMax(Eigen::Vector4f( sx/2 + margin,  sy/2 + margin,  sz/2 + margin, 1.0));
  }
  
  else if(obj_msg.primitives[0].type == shape_msgs::msg::SolidPrimitive::CYLINDER)
  {
    double h = obj_msg.primitives[0].dimensions[0];
    double r = obj_msg.primitives[0].dimensions[1];
  
    crop.setMin(Eigen::Vector4f(-r - margin, -r - margin, -h/2 - margin, 1.0));
    crop.setMax(Eigen::Vector4f( r + margin,  r + margin,  h/2 + margin, 1.0));
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr next(new pcl::PointCloud<pcl::PointXYZ>);
  crop.filter(*next);
  return next;
}

void ObjectRemover::callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
  RCLCPP_DEBUG(LOGGER, "Received point cloud");

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr current(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
  
  auto object_map = planning_scene_->getObjects();
  for (auto& obj : object_map)
  {
    auto& obj_msg = obj.second;
    if (obj_msg.primitives.empty()) continue;
    current = remover(current, obj_msg);
  }

  auto attached_map = planning_scene_->getAttachedObjects();
  for (auto& obj : attached_map)
  {
    auto& obj_msg = obj.second.object;
    if (obj_msg.primitives.empty()) continue;
    current = remover(current, obj_msg);
  }

  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(*current, output);
  output.header = cloud_msg->header;
  pub_filtered_->publish(output);
}

// ────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  moveit::planning_interface::PlanningSceneInterface _planning_scene;
  auto node = std::make_shared<ObjectRemover>(&_planning_scene);

  RCLCPP_INFO(LOGGER, "Object Remover Node ON");
  rclcpp::spin(node);
  RCLCPP_INFO(LOGGER, "Object Remover Node OFF");

  rclcpp::shutdown();
  return 0;
}