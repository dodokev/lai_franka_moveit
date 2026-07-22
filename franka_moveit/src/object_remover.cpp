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

void ObjectRemover::init()
{
    robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this());
    robot_model_ = robot_model_loader.getModel();

    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    robot_state_->setToDefaultValues();

    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      shared_from_this(),
      "robot_description"
    );
    planning_scene_monitor_->startStateMonitor();
   
    RCLCPP_INFO(this->get_logger(), "Robot model loaded: %s", robot_model_->getName().c_str());
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ObjectRemover::remover(pcl::PointCloud<pcl::PointXYZ>::Ptr current, moveit_msgs::msg::CollisionObject& obj_msg, const std::string& parent) {
  const double margin = 0.05; // e.g. 0.01
  pcl::CropBox<pcl::PointXYZ> crop;
  crop.setInputCloud(current);
  crop.setNegative(true);
  
  const auto& pose = obj_msg.pose;
  Eigen::Vector3f position(pose.position.x, pose.position.y, pose.position.z);
  tf2::Quaternion tf_q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

  if (parent != "")
  {
    moveit::core::RobotStatePtr current_state =
    planning_scene_monitor_->getStateMonitor()->getCurrentState();

    if (current_state) {
        robot_state_ = std::make_shared<moveit::core::RobotState>(*current_state);
        robot_state_->update();
    }

    const Eigen::Isometry3d& tf_w_h = robot_state_->getGlobalLinkTransform(parent);
    Eigen::Isometry3d tf_h_o = Eigen::Isometry3d::Identity();

    tf_h_o.linear() = Eigen::Quaterniond(
      pose.orientation.w,
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z
    ).toRotationMatrix();

    tf_h_o.translation() << pose.position.x,
                        pose.position.y,
                        pose.position.z;

    Eigen::Isometry3d tf_w_o = tf_w_h * tf_h_o;

    position(0) = tf_w_o.translation().x();
    position(1) = tf_w_o.translation().y();
    position(2) = tf_w_o.translation().z();

    Eigen::Quaterniond quat(tf_w_o.rotation());

    tf_q = tf2::Quaternion(
      quat.x(),
      quat.y(),
      quat.z(),
      quat.w());

    // RCLCPP_WARN_STREAM(LOGGER, "Position :" << position);
  }

  crop.setTranslation(position);

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

bool ObjectRemover::pointInsideBox(
    const pcl::PointXYZ& p,
    const shape_msgs::msg::SolidPrimitive& primitive,
    const geometry_msgs::msg::Pose& pose)
{
    Eigen::Vector3d point(p.x,p.y,p.z);

    Eigen::Quaterniond q(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z
    );

    Eigen::Vector3d local =
        q.inverse() *
        (point - Eigen::Vector3d(
            pose.position.x,
            pose.position.y,
            pose.position.z));


    double dx = primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] / 2.0;
    double dy = primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] / 2.0;
    double dz = primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] / 2.0;


    return 
        std::abs(local.x()) < dx &&
        std::abs(local.y()) < dy &&
        std::abs(local.z()) < dz;
}

bool ObjectRemover::pointInsideCylinder(
    const pcl::PointXYZ& p,
    const shape_msgs::msg::SolidPrimitive& primitive,
    const geometry_msgs::msg::Pose& pose)
{
  Eigen::Vector3d point(p.x,p.y,p.z);

    Eigen::Quaterniond q(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z
    );

    Eigen::Vector3d local =
        q.inverse() *
        (point - Eigen::Vector3d(
            pose.position.x,
            pose.position.y,
            pose.position.z));


    double dz = primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] / 2.0;
    double r = primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS];

    double r2 = local(0) * local(0) + local(1) * local(1);

    return 
        (r2 <= r*r) &&
        (std::abs(local.z()) < dz);
}

bool ObjectRemover::pointInsideCollisionObject(
    const pcl::PointXYZ& p,
    const moveit_msgs::msg::CollisionObject& obj, const std::string& parent)
{
    if(obj.primitives.empty())
        return false;

    geometry_msgs::msg::Pose pose = obj.pose;

    if (parent != "")
    {
      moveit::core::RobotStatePtr current_state =
      planning_scene_monitor_->getStateMonitor()->getCurrentState();

      if (current_state) {
          robot_state_ = std::make_shared<moveit::core::RobotState>(*current_state);
          robot_state_->update();
      }

      const Eigen::Isometry3d& tf_w_h = robot_state_->getGlobalLinkTransform(parent);
      Eigen::Isometry3d tf_h_o = Eigen::Isometry3d::Identity();

      tf_h_o.linear() = Eigen::Quaterniond(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z
      ).toRotationMatrix();

      tf_h_o.translation() << pose.position.x,
                          pose.position.y,
                          pose.position.z;

      Eigen::Isometry3d tf_w_o = tf_w_h * tf_h_o;

      pose.position.x = tf_w_o.translation().x();
      pose.position.y = tf_w_o.translation().y();
      pose.position.z = tf_w_o.translation().z();

      Eigen::Quaterniond quat(tf_w_o.rotation());

      pose.orientation.x = quat.x();
      pose.orientation.y = quat.y();
      pose.orientation.z = quat.z();
      pose.orientation.w = quat.w();
    }

    if (obj.primitives[0].dimensions.size() == 3)
      if(pointInsideBox(p, obj.primitives[0], pose))
        return true;

    if (obj.primitives[0].dimensions.size() == 2)
      if(pointInsideCylinder(p, obj.primitives[0], pose))
        return true;

    return false;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ObjectRemover::removerByCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr current, moveit_msgs::msg::CollisionObject& obj_msg, const std::string& parent)
{
  pcl::SACSegmentation<pcl::PointXYZ> _seg;
  _seg.setOptimizeCoefficients(true);
  _seg.setModelType(pcl::SACMODEL_PLANE);
  _seg.setMethodType(pcl::SAC_RANSAC);
  _seg.setDistanceThreshold(0.02);
  _seg.setInputCloud(current);

  pcl::PointIndices::Ptr _inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr _coefficients(new pcl::ModelCoefficients);
  _seg.segment(*_inliers, *_coefficients);

  pcl::ExtractIndices<pcl::PointXYZ> _extract;
  _extract.setInputCloud(current);
  _extract.setIndices(_inliers);
  
  _extract.setNegative(false);
  pcl::PointCloud<pcl::PointXYZ>::Ptr table(new pcl::PointCloud<pcl::PointXYZ>);
  _extract.filter(*table);

  _extract.setNegative(true);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  _extract.filter(*cluster_cloud);

  if (cluster_cloud->points.empty())
    return current;
    
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cluster_cloud);

  std::vector<pcl::PointIndices> cluster_indices;

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.06); // 2 cm
  ec.setMinClusterSize(0);
  ec.setMaxClusterSize(100000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cluster_cloud);
  ec.extract(cluster_indices);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
  // RCLCPP_INFO(LOGGER, "Start remove cluster");

  for (const auto& indices : cluster_indices)
  {
    bool collision = false;

    for (const auto& idx : indices.indices)
    {
      const auto& p = cluster_cloud->points[idx];

      if (pointInsideCollisionObject(p, obj_msg, parent))
      {
        // RCLCPP_WARN(LOGGER, "Inside");
        collision = true;
        break;
      }
    }

    if (!collision)
    {
      // RCLCPP_WARN(LOGGER, "Outside");
      for (auto idx : indices.indices)
      {
        filtered->push_back(cluster_cloud->points[idx]);
      }
    }
  }

  *filtered += *table;
  return filtered;
}


void ObjectRemover::callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
  RCLCPP_DEBUG(LOGGER, "Received point cloud");
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr current(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
  
  auto object_map = planning_scene_->getObjects();
  // RCLCPP_INFO(LOGGER, "Start remover");
  for (auto& obj : object_map)
  {
    
    auto& obj_msg = obj.second;
    std::string tmp_id = obj_msg.id;
    std::remove_if(tmp_id.begin(), tmp_id.end(), [](unsigned char c){return std::isdigit(c);});
    
    if (tmp_id == "static") continue;
    if (obj_msg.primitives.empty()) continue;
    current = removerByCluster(current, obj_msg);
    current = remover(current, obj_msg);
  }

  auto attached_map = planning_scene_->getAttachedObjects();
  for (auto& obj : attached_map)
  {
    auto& att_msg = obj.second;
    auto& obj_msg = att_msg.object;
    // RCLCPP_WARN(LOGGER, "Who : %s", obj_msg.id.c_str());
    // RCLCPP_WARN(LOGGER, "By : %s", att_msg.link_name.c_str());
    if (obj_msg.primitives.empty()) continue;
    current = removerByCluster(current, obj_msg, att_msg.link_name);
    current = remover(current, obj_msg, att_msg.link_name);
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
  node->init();

  RCLCPP_INFO(LOGGER, "Object Remover Node ON");
  rclcpp::spin(node);
  RCLCPP_INFO(LOGGER, "Object Remover Node OFF");

  rclcpp::shutdown();
  return 0;
}