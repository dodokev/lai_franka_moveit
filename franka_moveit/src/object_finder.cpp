#include "franka_moveit/object_finder.hpp"
#include <pcl/common/pca.h>
#include <pcl/filters/passthrough.h>
#include <boost/algorithm/string/split.hpp>

static auto const LOGGER = rclcpp::get_logger("object_finder");

/**
 * Instead of using RGB, cylinder fitting or save from the cylinderScoreFunction in the object
 * Add an inliers member vector
 * cylinderBias function : extract inlier, 
 * Give up the RGB pipeline, too much depth dependant to be reliable
 */

ObjectFinder::ObjectFinder(moveit::planning_interface::PlanningSceneInterface* ps)
    : Node("object_finder"), planning_scene_(ps) {

  /**
   * RGB Init Member
   */

  // image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/camera/color/image_raw", 10, 
  //   std::bind(&ObjectFinder::image_callback,this,std::placeholders::_1));
  
  // depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/camera/depth/image_rect_raw", 10, 
  //   std::bind(&ObjectFinder::depth_callback,this,std::placeholders::_1));
    
  // img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/visualize_img", 10);
  
  // contour_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  // intrinsic_ = Eigen::Matrix3d::Zero();
  // intrinsic_(0, 0) = 213.79100036621094;
  // intrinsic_(0, 2) = 212.39974975585938;
  // intrinsic_(1, 1) = 213.79100036621094;
  // intrinsic_(1, 2) = 119.67296600341797;
  // intrinsic_(2, 2) = 1.0;

  /**
   * Point Cloud Init Member
   */
  pub_centroid_ = this->create_publisher<visualization_msgs::msg::Marker>("/centroid", 10);
  
  service_ = this->create_service<franka_moveit_msg::srv::EnableCreate>(
        "enable_create",
        std::bind(&ObjectFinder::handle_service, this,
                std::placeholders::_1,
                std::placeholders::_2)
    );
  
  sub_unfilter_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/points_used", rclcpp::SensorDataQoS(),
      std::bind(&ObjectFinder::filter_callback, this, std::placeholders::_1));
  pub_filtered_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/visualize_cloud", rclcpp::SensorDataQoS());

  sub_size_ = this->create_subscription<std_msgs::msg::String>(
      "/add_lost_obj", rclcpp::SensorDataQoS(),
      std::bind(&ObjectFinder::request_callback, this, std::placeholders::_1));

  result_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  table_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cluster_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  object_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  tree_ = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
}

// ================================================================================================
/**
 * RGB Finder Fucntion Definition
 */
void ObjectFinder::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat rgb = img->image;
  cv::Mat resized;
  cv::resize(rgb, resized, cv::Size(424, 240), 0, 0, cv::INTER_LINEAR);

  // RCLCPP_WARN(LOGGER, "sizeRGB : %dx%d", rgb.rows, rgb.cols);

  cv::Mat hsv;
  cv::cvtColor(resized, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(hsv, cv::Scalar(0, 55, 165), cv::Scalar(25, 255, 255), mask);

  cv::findContours(mask, cluster_contour_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  cv::drawContours(resized, cluster_contour_, -1, cv::Scalar(0, 255, 0), 1);

  img->image = resized;
  img_pub_->publish(*img->toImageMsg());  
}

void ObjectFinder::depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{ 
  // RCLCPP_INFO(LOGGER, "Depth Call");
  if (!cluster_contour_.empty())
  {
    cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(msg);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    cv::Mat depth = img->image;

    contour_cloud_->clear();
    contour_cloud_->header.frame_id = "camera_depth_optical_frame";

    double fx = static_cast<double>(intrinsic_(0, 0));
    double fy = static_cast<double>(intrinsic_(1, 1));
    double cx = static_cast<double>(intrinsic_(0, 2));
    double cy = static_cast<double>(intrinsic_(1, 2));

    // for (const auto& contour : cluster_contour_)
    // {
    //   for (const auto& pt : contour)
    //   {
    //     const int u = pt.x; 
    //     const int v = pt.y;

    //     uint16_t d = depth.at<uint16_t>(u, v);
    //     double z =  static_cast<double>(d) / 1000;
    //     double x = (u - cx) * z / fx;
    //     double y = (v - cy) * z / fy;

    //     pcl::PointXYZ pt_tmp(x, y, z);
    //     contour_cloud_->points.push_back(pt_tmp);
    //   }
    // }

    for (int row = 0; row < depth.rows; row++)
    {
      for (int col = 0; col < depth.cols; col++)
      {
        uint16_t d = depth.at<uint16_t>(row, col);
        double z = static_cast<double>(d) / 1000;
        double x = (row - cx) * z / fx;
        double y = - (col - cy) * z / fy;
 
        // RCLCPP_WARN(LOGGER, "Point : %f, %f, %f", x, y, z);

        pcl::PointXYZ pt_tmp(x, y, z);
        contour_cloud_->points.push_back(pt_tmp);       
      }
    }

    sensor_msgs::msg::PointCloud2 _output;
    pcl::toROSMsg(*contour_cloud_, _output);
    _output.header.frame_id = "camera_depth_optical_frame";
    pub_filtered_->publish(_output);
  }
}

// ================================================================================================
/**
 * Point Cloud Finder Function Definition
 */

void ObjectFinder::handle_service(
    const std::shared_ptr<franka_moveit_msg::srv::EnableCreate::Request> request,
    std::shared_ptr<franka_moveit_msg::srv::EnableCreate::Response> response)
{
  RCLCPP_INFO(LOGGER, "Enable Create Called ...");
  
  RCLCPP_WARN(LOGGER, "Asking %s", request->enable ? "True" : "False");

  enable_create_ = request->enable;

  response->current = enable_create_;
  response->success = true;
}

void ObjectFinder::request_callback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string _param = msg->data;
  
  std::vector<std::string> _param_split;
  boost::split(_param_split, _param, boost::is_any_of(","));

  std::vector<double> _dim;
  Shape _type;

  for (const auto& _p : _param_split)
    _dim.push_back(std::stof(_p));
  _type = static_cast<Shape>(_dim.size());
  
  bool exist{false};
  for (auto& obj : objects_)
  {
    if (obj.shape == _type)
      if (obj.dimension == _dim)
      {
        ++(obj.number);
        obj.poses.push_back(Eigen::Affine3d::Identity());
        obj.candidates.push_back(Eigen::Affine3d::Identity());
        obj.confidences.push_back(0);

        obj.have_stable.push_back(false);
        obj.have_candidate.push_back(false);

        exist = true;
        break;
      }
  }
    
  if (!exist)
  {
    objects_.push_back(Object(_type, _dim));
    objects_.back().poses.push_back(Eigen::Affine3d::Identity());
    objects_.back().candidates.push_back(Eigen::Affine3d::Identity());
    objects_.back().confidences.push_back(0);
    objects_.back().have_stable.push_back(false);
    objects_.back().have_candidate.push_back(false);
  }
  
  begin_ = true;
  RCLCPP_WARN(LOGGER, "New object to find");
}

// ────────────────────────────────────────────────────────────────────────────
bool ObjectFinder::retreiveObject() {  
  pcl::SACSegmentation<pcl::PointXYZ> _seg;
  _seg.setOptimizeCoefficients(true);
  _seg.setModelType(pcl::SACMODEL_PLANE);
  _seg.setMethodType(pcl::SAC_RANSAC);
  _seg.setDistanceThreshold(0.02);
  _seg.setInputCloud(cloud_);

  // Eigen::Vector3f z(0, 0, 1);
  // _seg.setAxis(z);
  // _seg.setEpsAngle(M_PI / 64);
  
  pcl::PointIndices::Ptr _inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr _coefficients(new pcl::ModelCoefficients);
  _seg.segment(*_inliers, *_coefficients);

  pcl::ExtractIndices<pcl::PointXYZ> _extract;
  _extract.setInputCloud(cloud_);
  _extract.setIndices(_inliers);
  
  _extract.setNegative(false);
  _extract.filter(*table_);

  
  /**
   * Save table information
   */
  table_normal_ = Eigen::Vector3d(_coefficients->values[0], _coefficients->values[1], _coefficients->values[2]).normalized();
  if (table_normal_.z() < 0) table_normal_ = -table_normal_; // point "up"
    table_d_ = _coefficients->values[3];
    // table_d_ = 0.0;

  _extract.setNegative(true);
  _extract.filter(*cluster_cloud_);

  // BUG FIX: tree must be set before EuclideanClusterExtraction uses it
  if (cluster_cloud_->empty())
    return false;
  tree_->setInputCloud(cluster_cloud_);
  
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> _ec;
  _ec.setClusterTolerance(0.05);
  _ec.setMinClusterSize(20);
  _ec.setSearchMethod(tree_);
  _ec.setInputCloud(cluster_cloud_);
  _ec.extract(cluster_indices_);

  return true;
}

// ────────────────────────────────────────────────────────────────────────────
void ObjectFinder::getCentroidAndOBB(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud,
                                     Eigen::Vector4f& centroid,
                                     pcl::PointXYZ& min,
                                     pcl::PointXYZ& max,
                                     pcl::PointXYZ& center,
                                     Eigen::Matrix3f& rot) {
  pcl::compute3DCentroid(*pcl_cloud, centroid);

  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> _fe;
  _fe.setInputCloud(pcl_cloud);
  _fe.compute();
  _fe.getOBB(min, max, center, rot);
}

// ────────────────────────────────────────────────────────────────────────────
double ObjectFinder::boundingScore(const std::vector<double>& dim, const std::vector<double>& ground_dim, const Shape& type) {
  // dims already sorted descending by caller
  std::vector<double> _copy = ground_dim;
  std::sort(_copy.begin(), _copy.end(), std::greater<double>());

  double _score = 0.0;
  switch (type) {
    case Shape::SPHERE:
      _score = std::sqrt(std::pow(dim[0] - _copy[0], 2));
      break;
    case Shape::CYLINDER:
      _score = std::sqrt(std::pow(dim[0] - _copy[0], 2) + std::pow(2 * dim[1] - _copy[1], 2));
      break;
    case Shape::BOX:
      _score = std::sqrt(std::pow(dim[0] - _copy[0], 2) + std::pow(dim[1] - _copy[1], 2) +
                         std::pow(dim[2] - _copy[2], 2));
      break;
    default:
      RCLCPP_ERROR(LOGGER, "INCORRECT Shape TYPE");
  }

  // Returns [0,1]: 1 = perfect match, approaches 0 as dimensions diverge
  return std::exp(-_score / 0.2);
}

// ────────────────────────────────────────────────────────────────────────────
double ObjectFinder::normalScore(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud,
                                 pcl::PointCloud<pcl::Normal>::Ptr normals,
                                 Eigen::Vector4f& centroid) {
  // BUG FIX: use different normal heuristics depending on shape.
  //   - Cylinder/Sphere: normals should be radially outward → high dot = good
  //   - Box: normals should be axis-aligned flat → low variance per face
  //   Here we keep the generic radial version and let shape scorers dominate.
  double _normal_score = 0.0;
  int _valid = 0;
  for (size_t i = 0; i < pcl_cloud->size(); ++i) {
    if (!std::isfinite(normals->points[i].normal_x))
      continue;

    Eigen::Vector3f _p = pcl_cloud->points[i].getVector3fMap();
    Eigen::Vector3f _n(normals->points[i].normal_x, normals->points[i].normal_y,
                       normals->points[i].normal_z);
    Eigen::Vector3f _radial = (_p - centroid.head<3>()).normalized();

    _normal_score += std::fabs(_n.dot(_radial));
    ++_valid;
  }

  return (_valid > 0) ? _normal_score / _valid : 0.0;
}

// ────────────────────────────────────────────────────────────────────────────
double ObjectFinder::cornerScore(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud) {
  if (obj_cloud->points.size() < 12) {
    RCLCPP_DEBUG(LOGGER, "Not enough points for cornerScore");
    return 0.0;  // BUG FIX: was returning false (bool → double implicit cast)
  }

  pcl::SACSegmentation<pcl::PointXYZ> _seg;
  _seg.setOptimizeCoefficients(true);
  _seg.setModelType(pcl::SACMODEL_PLANE);
  _seg.setMethodType(pcl::SAC_RANSAC);
  _seg.setDistanceThreshold(0.005);  // slightly relaxed for partial views

  struct PlaneInfo {
    Eigen::Vector3d normal;
    std::size_t nb_inlier;
  };
  std::vector<PlaneInfo> planes;

  pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(new pcl::PointCloud<pcl::PointXYZ>(*obj_cloud));
  pcl::ExtractIndices<pcl::PointXYZ> _extract;

  // Require at least 5% of remaining cloud to accept a plane
  const float min_inlier_ratio = 0.05f;

  double threshold = 0.2;

  for (int i = 0; i < 5; ++i) {
    if (remaining->size() < 20)
      break;

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);

    _seg.setInputCloud(remaining);
    _seg.segment(*inliers, *coeff);

    if (inliers->indices.empty())
      break;

    if ((float)inliers->indices.size() / (float)remaining->size() < min_inlier_ratio)
      break;

    // Compute the centroid of inliers → a point on this face
    pcl::PointCloud<pcl::PointXYZ>::Ptr face_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    _extract.setInputCloud(remaining);
    _extract.setIndices(inliers);
    _extract.setNegative(false);
    _extract.filter(*face_cloud);

    Eigen::Vector3d normal(coeff->values[0], coeff->values[1], coeff->values[2]);
    normal.normalize();

    double align = std::abs(normal.dot(table_normal_));
    if (align > threshold && align < (1 - threshold))
      continue;

    planes.push_back({normal, inliers->indices.size()});

    // Remove inliers and continue
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    _extract.setNegative(true);
    _extract.filter(*tmp);
    remaining = tmp;
  }

  if (planes.size() < 2) {
    // RCLCPP_DEBUG(LOGGER, "Fewer than 2 planes found, not a possible corner");
    return 0;
  }

  Eigen::Vector3d n0 = planes[0].normal;
  Eigen::Vector3d n1 = planes[1].normal;

  std::size_t nb_inlier0 = planes[0].nb_inlier;
  std::size_t nb_inlier1 = planes[1].nb_inlier;

  double dot = std::abs(n0.dot(n1));
  if (dot > 0.3)
  {
    RCLCPP_DEBUG(
        rclcpp::get_logger("object_finder"),
        "centroidBias: plane normals are not perpendicular (|dot|=%.2f), result may be unreliable.",
        dot);
  }

  double _ratio0 = static_cast<double>(nb_inlier0) / obj_cloud->size();
  double _ratio1 = static_cast<double>(nb_inlier1) / obj_cloud->size();

  // Score: planes should be perpendicular (dot ≈ 0) and cover most points
  double _perp = 1.0 - std::fabs(n0.normalized().dot(n1.normalized()));
  return _perp * (_ratio0 + _ratio1) * 0.5;
}

// ────────────────────────────────────────────────────────────────────────────
double ObjectFinder::planeScore(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud) {
  if (obj_cloud->points.size() < 12) {
    RCLCPP_DEBUG(LOGGER, "Not enough points for planeScore");
    return 0.0;  // BUG FIX: was returning false
  }

  pcl::SACSegmentation<pcl::PointXYZ> _seg;
  _seg.setOptimizeCoefficients(true);
  _seg.setModelType(pcl::SACMODEL_PLANE);
  _seg.setMethodType(pcl::SAC_RANSAC);
  _seg.setDistanceThreshold(0.005);

  pcl::PointIndices::Ptr _inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr _coeff(new pcl::ModelCoefficients);

  _seg.setInputCloud(obj_cloud);
  _seg.segment(*_inliers, *_coeff);

  return static_cast<double>(_inliers->indices.size()) / obj_cloud->size();
}

// ────────────────────────────────────────────────────────────────────────────
double ObjectFinder::cylinderScore(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud,
                                   pcl::PointCloud<pcl::Normal>::Ptr normals, const std::vector<double>& dim, const Shape& type) {

  if (obj_cloud->points.size() < 20)
    return 0;

  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> _seg;
  _seg.setOptimizeCoefficients(true);
  _seg.setModelType(pcl::SACMODEL_CYLINDER);
  _seg.setMethodType(pcl::SAC_RANSAC);
  _seg.setInputNormals(normals);
  _seg.setNormalDistanceWeight(0.1);  // lowered: noisy normals on partial view
  _seg.setMaxIterations(5000);
  _seg.setDistanceThreshold(0.005);
  _seg.setEpsAngle(M_PI / 2.0);

  // BUG FIX: pick the right size index per object type.
  double _r = (type == Shape::SPHERE) ? dim[0] : dim[1];
  _seg.setRadiusLimits(_r * 0.85, _r * 1.15);  // wider tolerance for partial view

  _seg.setInputCloud(obj_cloud);

  pcl::ModelCoefficients::Ptr _coeff(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr _inliers(new pcl::PointIndices);
  _seg.segment(*_inliers, *_coeff);

  return static_cast<double>(_inliers->indices.size()) / obj_cloud->size();
}

// ────────────────────────────────────────────────────────────────────────────
Eigen::Affine3d ObjectFinder::centroidBiasBox(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud,
                                   const std::vector<double>& dim) {
  Eigen::Affine3d pose;

  // 1. Centroid
  Eigen::Vector4f centroid4;
  pcl::compute3DCentroid(*obj_cloud, centroid4);
  Eigen::Vector3d centroid = centroid4.head<3>().cast<double>();

  // 2. Plane segmentation (unchanged from your version)
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setMaxIterations(100);

  pcl::ExtractIndices<pcl::PointXYZ> extract;

  struct PlaneInfo {
    Eigen::Vector3d normal;
    Eigen::Vector3d mean_point;
    double alignment;   // |dot| with table_normal_
    int inlier_count;
  };
  std::vector<PlaneInfo> planes;

  pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(new pcl::PointCloud<pcl::PointXYZ>(*obj_cloud));
  const float min_inlier_ratio = 0.05f;
  const double threshold = 0.1;  // band around 0 / 1 considered "ambiguous"

  for (int i = 0; i < 5; ++i) {
    if (remaining->size() < 20) break;

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    seg.setInputCloud(remaining);
    seg.segment(*inliers, *coeff);

    if (inliers->indices.empty()) break;
    if ((float)inliers->indices.size() / (float)remaining->size() < min_inlier_ratio) break;

    pcl::PointCloud<pcl::PointXYZ>::Ptr face_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(remaining);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*face_cloud);

    Eigen::Vector4f face_centroid4;
    pcl::compute3DCentroid(*face_cloud, face_centroid4);
    Eigen::Vector3d face_centroid = face_centroid4.head<3>().cast<double>();

    Eigen::Vector3d normal(coeff->values[0], coeff->values[1], coeff->values[2]);
    normal.normalize();

    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setNegative(true);
    extract.filter(*tmp);
    remaining = tmp;

    double align = std::abs(normal.dot(table_normal_));
    if (align > threshold && align < (1 - threshold)) {
      RCLCPP_DEBUG(LOGGER, "rejected ambiguous plane, align=%f", align);
      continue;  // neither clearly horizontal nor clearly vertical -> discard
    }

    if (normal.dot(face_centroid - centroid) < 0)
      normal = -normal;

    planes.push_back({normal, face_centroid, align, (int)inliers->indices.size()});
  }

  if (planes.empty()) {
    RCLCPP_WARN(LOGGER, "centroidBias: no usable planes found, falling back to PCA centroid.");
    pose.linear() = Eigen::Matrix3d::Identity();
    pose.translation() = centroid;
    return pose;
  }

  // ---------------------------
  // 3. Anchor vertical axis to table_normal_ (trusted, low-noise prior)
  // ---------------------------
  Eigen::Vector3d z_up = table_normal_.normalized();

  // Split planes into "horizontal-ish" (top/bottom face, align near 1)
  // and "vertical-ish" (side face, align near 0), keep the most-confident
  // (most extreme alignment, NOT closest to threshold) of each.
  auto confidence = [&](const PlaneInfo& p) {
    return std::min(p.alignment, 1.0 - p.alignment);  // smaller = more confident
  };

  PlaneInfo* best_horizontal = nullptr;
  PlaneInfo* best_vertical   = nullptr;
  for (auto& p : planes) {
    if (p.alignment > 0.5) {  // horizontal-ish (parallel to table normal)
      if (!best_horizontal || confidence(p) < confidence(*best_horizontal))
        best_horizontal = &p;
    } else {                   // vertical-ish (perpendicular to table normal)
      if (!best_vertical || confidence(p) < confidence(*best_vertical))
        best_vertical = &p;
    }
  }

  // Side-face normal resolves yaw. Snap it to be EXACTLY perpendicular to
  // z_up by removing any component along z_up — this is the step that
  // kills most of the angular noise from a slightly tilted RANSAC fit.
  Eigen::Vector3d x_axis;
  bool have_side_normal = false;
  if (best_vertical) {
    Eigen::Vector3d n = best_vertical->normal;
    n -= n.dot(z_up) * z_up;
    if (n.norm() > 1e-6) {
      x_axis = n.normalized();
      have_side_normal = true;
    }
  }

  // RCLCPP_INFO(LOGGER, "X AXIS ");
  // RCLCPP_WARN_STREAM(LOGGER, "" << x_axis);

  if (!have_side_normal) {
    // No reliable side face — fall back to PCA on the horizontal plane
    // to at least get a consistent (if possibly less semantically
    // meaningful) yaw.
    RCLCPP_WARN(LOGGER, "centroidBias: no vertical side face found, yaw is unconstrained.");
    // Pick an arbitrary axis perpendicular to z_up, deterministic.
    Eigen::Vector3d arbitrary = std::abs(z_up.x()) < 0.9 ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
    x_axis = (arbitrary - arbitrary.dot(z_up) * z_up).normalized();
  }

  Eigen::Vector3d y_axis = z_up.cross(x_axis).normalized();
  x_axis = y_axis.cross(z_up).normalized();  // re-orthogonalize

  std::array<Eigen::Vector3d, 3> axes = {x_axis, y_axis, z_up};

  // ---------------------------
  // 4. Match axes to known box dimensions via observed extent (same idea
  //    as your original, but axes are now far more stable)
  // ---------------------------
  std::array<double, 3> sizes = {dim[0], dim[1], dim[2]};
  std::array<int, 3> size_order = {0, 1, 2};
  std::sort(size_order.begin(), size_order.end(),
            [&](int a, int b) { return sizes[a] > sizes[b]; });

  std::array<double, 3> extents;
  for (int i = 0; i < 3; ++i) {
    float min_proj = 1e9f, max_proj = -1e9f;
    for (const auto& pt : *obj_cloud) {
      double proj = Eigen::Vector3d(pt.x, pt.y, pt.z).dot(axes[i]);
      min_proj = std::min(min_proj, (float)proj);
      max_proj = std::max(max_proj, (float)proj);
    }
    extents[i] = max_proj - min_proj;
  }

  std::array<int, 3> axis_order = {0, 1, 2};
  std::sort(axis_order.begin(), axis_order.end(),
            [&](int a, int b) { return extents[a] > extents[b]; });

  std::array<double, 3> half_sizes;
  std::array<Eigen::Vector3d, 3> sorted_axes;
  for (int k = 0; k < 3; ++k) {
    sorted_axes[k] = axes[axis_order[k]];
    half_sizes[k] = sizes[size_order[k]] / 2.0;
  }

  Eigen::Vector3d fx = sorted_axes[2];
  Eigen::Vector3d fy = sorted_axes[1];
  Eigen::Vector3d fz = sorted_axes[0];
  if (fx.cross(fy).dot(fz) < 0) fz = -fz;

  // ---------------------------
  // 5. Inlier-weighted center estimate (instead of plain average)
  // ---------------------------
  Eigen::Vector3d box_center = Eigen::Vector3d::Zero();
  double weight_sum = 0.0;

  for (const auto& plane : planes) {
    double best_dot = -1.0, half_L = 0.0;
    for (int k = 0; k < 3; ++k) {
      double d = std::abs(plane.normal.dot(sorted_axes[k]));
      if (d > best_dot) { best_dot = d; half_L = half_sizes[k]; }
    }
    double w = static_cast<double>(plane.inlier_count);
    box_center += w * (plane.mean_point - plane.normal * half_L);
    weight_sum += w;
  }
  box_center /= weight_sum;

  box_center(2) += table_d_;

  // ---------------------------
  // 6. Final pose
  // ---------------------------
  Eigen::Matrix3d R;
  R.col(0) = fx;
  R.col(1) = fy;
  R.col(2) = fz;

  pose.linear() = R;
  pose.translation() = box_center;

  return pose;
}

// ────────────────────────────────────────────────────────────────────────────
Eigen::Affine3d ObjectFinder::centroidBiasCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud,
                                        const std::vector<double>& dim) {
  double known_radius = dim[1];
  double known_height = dim[0];

  Eigen::Affine3d pose;


  // --------------------------------------------------------------------------------
  /**
   * Cleaning
   */

  auto _cluster_tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
  _cluster_tree->setInputCloud(obj_cloud);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> _ne;
  _ne.setInputCloud(obj_cloud);
  _ne.setSearchMethod(_cluster_tree);
  _ne.setKSearch(20);

  pcl::PointCloud<pcl::Normal>::Ptr _normals(new pcl::PointCloud<pcl::Normal>);
  _ne.compute(*_normals);
  
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> _seg;
  _seg.setOptimizeCoefficients(true);
  _seg.setModelType(pcl::SACMODEL_CYLINDER);
  _seg.setMethodType(pcl::SAC_RANSAC);
  _seg.setInputNormals(_normals);
  _seg.setNormalDistanceWeight(0.1);
  _seg.setMaxIterations(5000);
  _seg.setDistanceThreshold(0.04);
  _seg.setEpsAngle(M_PI / 2.0);
  _seg.setRadiusLimits(known_radius * 0.75, known_radius * 1.25);
  _seg.setInputCloud(obj_cloud);

  pcl::ModelCoefficients::Ptr _coeff(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr _inliers(new pcl::PointIndices);
  _seg.segment(*_inliers, *_coeff);

  pcl::ExtractIndices<pcl::PointXYZ> _extract;
  _extract.setInputCloud(obj_cloud);
  _extract.setIndices(_inliers);
  _extract.setNegative(false);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  _extract.filter(*cylinder_cloud);
  // --------------------------------------------------------------------------------
  
  // for (std::size_t iPt = 0; iPt < obj_cloud->size(); iPt++)
  // {
  //     auto pt = obj_cloud->points[iPt];
  //     if (pt.z > known_height * 0.2)
  //     {
  //       obj_cloud->erase(obj_cloud->begin() + iPt);
  //       --iPt;
  //     }
  // }
  for (std::size_t iPt = 0; iPt < cylinder_cloud->size(); iPt++)
  {
      auto pt = cylinder_cloud->points[iPt];
      if (pt.z > known_height * 0.5)
      {
        cylinder_cloud->erase(cylinder_cloud->begin() + iPt);
        --iPt;
      }
  }

  sensor_msgs::msg::PointCloud2 _output;
  pcl::toROSMsg(*cylinder_cloud, _output);
  _output.header.frame_id = "world";
  pub_filtered_->publish(_output);

  Eigen::Vector4f centroid4;
  // pcl::compute3DCentroid(*obj_cloud, centroid4);
  pcl::compute3DCentroid(*cylinder_cloud, centroid4);
  Eigen::Vector3d centroid = centroid4.head<3>().cast<double>();

  Eigen::Vector3d z_axis = table_normal_.normalized();

  Eigen::Vector3d abs_z = z_axis.cwiseAbs();
  Eigen::Vector3d ref;
  if (abs_z.x() <= abs_z.y() && abs_z.x() <= abs_z.z())
    ref = Eigen::Vector3d::UnitX();
  else if (abs_z.y() <= abs_z.x() && abs_z.y() <= abs_z.z())
    ref = Eigen::Vector3d::UnitY();
  else
    ref = Eigen::Vector3d::UnitZ();

  Eigen::Vector3d x_axis = (ref - ref.dot(z_axis) * z_axis).normalized();
  Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

  // int N = obj_cloud->size();
  int N = cylinder_cloud->size();
  Eigen::MatrixXd A(N, 3);
  Eigen::VectorXd b(N);

  for (int i = 0; i < N; ++i) {
    // Eigen::Vector3d p(obj_cloud->points[i].x, obj_cloud->points[i].y, obj_cloud->points[i].z);
    Eigen::Vector3d p(cylinder_cloud->points[i].x, cylinder_cloud->points[i].y, cylinder_cloud->points[i].z);

    Eigen::Vector3d dp = p - centroid;
    double u = dp.dot(x_axis);
    double v = dp.dot(y_axis);

    A(i, 0) = 2.0 * u;
    A(i, 1) = 2.0 * v;
    A(i, 2) = -1.0;
    b(i) = u * u + v * v;
  }
  
  Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
  double cu = x(0);  // circle center in 2D (relative to raw centroid)
  double cv = x(1);
  Eigen::Vector2d center(cu, cv);
  
  Eigen::Vector3d tmp_position = centroid + cu * x_axis + cv * y_axis;
  Eigen::Vector2d tmp_center(tmp_position(0), tmp_position(1));

  for (int iter = 0; iter < 10; ++iter)
  {
    
    Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
    Eigen::Vector2d g = Eigen::Vector2d::Zero();
    
    // for (const auto& pt : *obj_cloud)
    for (const auto& pt : *cylinder_cloud)
    {
        Eigen::Vector3d p3(pt.x, pt.y, pt.z);
        
        Eigen::Vector3d dp = p3 - centroid;

        Eigen::Vector2d p(
            dp.dot(x_axis),
            dp.dot(y_axis) - 0.01);

        Eigen::Vector2d d = center - p;
        double dist = d.norm();
        if (dist < 1e-6)
        continue;
        
        double r = dist - known_radius;
        
        Eigen::Vector2d J = d / dist;
        
        // Huber
        double w = 1.0;
        double a = std::fabs(r);
        const double delta = 0.02;
        if (a > delta)
          w = delta / a;

        H += w * J * J.transpose();
        g += w * J * r;
      }

      center -= H.ldlt().solve(g);
  }

  cu = center.x();
  cv = center.y();

  Eigen::Vector3d correction = cu * x_axis + cv * y_axis;
  Eigen::Vector3d axis_center = centroid + 1.0 * correction;

  // ---------------------------
  // 4. Bias along Z (axial correction) — relative to axis_center now
  // ---------------------------

  // ===============================================================================
  // Fixed to known height for now
  Eigen::Vector3d true_center = axis_center;
  true_center(2) = known_height / 2 + 0.01;
  // ===============================================================================

  // double min_t = 1e9;
  // double max_t = -1e9;

  // for (const auto& pt : *obj_cloud) {
  //   Eigen::Vector3d p(pt.x, pt.y, pt.z);
  //   double t = (p - axis_center).dot(z_axis);
  //   min_t = std::min(min_t, t);
  //   max_t = std::max(max_t, t);
  // }

  // double observed_height = max_t - min_t;
  // double bias = 0.0;

  // if (observed_height >= known_height * 0.85) {
  //   bias = (min_t + max_t) / 2.0;
  //   RCLCPP_DEBUG(rclcpp::get_logger("object_finder"),
  //                "centroidBiasCylinder: both caps visible, bias=%.3f", bias);
  // } else {
  //   if (std::abs(max_t) >= std::abs(min_t)) {
  //     bias = max_t - known_height / 2.0;
  //     RCLCPP_DEBUG(rclcpp::get_logger("object_finder"),
  //                  "centroidBiasCylinder: max cap anchor (max_t=%.3f), bias=%.3f", max_t, bias);
  //   } else {
  //     bias = min_t + known_height / 2.0;
  //     RCLCPP_DEBUG(rclcpp::get_logger("object_finder"),
  //                  "centroidBiasCylinder: min cap anchor (min_t=%.3f), bias=%.3f", min_t, bias);
  //   }
  // }

  // Eigen::Vector3d true_center = axis_center + (bias) * z_axis;

  // RCLCPP_DEBUG(rclcpp::get_logger("object_finder"),
  //              "centroidBiasCylinder: true_center=[%.3f %.3f %.3f]", true_center.x(),
              //  true_center.y(), true_center.z());

  // ---------------------------
  // 6. Build final pose
  // ---------------------------
  Eigen::Matrix3d R;
  R.col(0) = x_axis;
  R.col(1) = y_axis;
  R.col(2) = z_axis;

  pose.linear() = R;
  pose.translation() = true_center;

  return pose;
}

// ────────────────────────────────────────────────────────────────────────────
Eigen::Affine3d ObjectFinder::centroidBias(
  pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud,
  const std::vector<double>& dim, const Shape& type) {

  if (type == Shape::BOX)
    return centroidBiasBox(obj_cloud, dim);
  else if (type == Shape::CYLINDER)
    return centroidBiasCylinder(obj_cloud, dim);

  return Eigen::Affine3d();
}

// ────────────────────────────────────────────────────────────────────────────
void ObjectFinder::createBox(Eigen::Affine3d& pose, const std::vector<double>& dim, std::size_t& numero) {
  moveit_msgs::msg::CollisionObject object;
  object.id = "object" + std::to_string(numero);
  object.header.frame_id = "world";
  object.primitives.resize(1);

  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions = {dim[2], dim[1], dim[0]};

  geometry_msgs::msg::Pose p;
  p.position.x = pose.translation().x();
  p.position.y = pose.translation().y();
  p.position.z = pose.translation().z();

  Eigen::Quaterniond quat(pose.linear());
  p.orientation.x = quat.x();
  p.orientation.y = quat.y();
  p.orientation.z = quat.z();
  p.orientation.w = quat.w();

  object.primitive_poses.push_back(p);

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

// ────────────────────────────────────────────────────────────────────────────
void ObjectFinder::createCylinder(Eigen::Affine3d& pose, const std::vector<double>& dim, std::size_t& numero) {
  moveit_msgs::msg::CollisionObject object;
  object.id = "object" + std::to_string(numero);
  object.header.frame_id = "world";
  object.primitives.resize(1);

  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = {dim[0], dim[1]};

  geometry_msgs::msg::Pose p;
  p.position.x = pose.translation().x();
  p.position.y = pose.translation().y();
  p.position.z = pose.translation().z();

  Eigen::Quaterniond quat(pose.linear());
  p.orientation.x = quat.x();
  p.orientation.y = quat.y();
  p.orientation.z = quat.z();
  p.orientation.w = quat.w();

  object.primitive_poses.push_back(p);

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

// ────────────────────────────────────────────────────────────────────────────
void ObjectFinder::createObstacle(Eigen::Affine3d& pose, const std::vector<double>& dim, const Shape& type, std::size_t& numero) {
  if (type == Shape::BOX)
    createBox(pose, dim, numero);
  if (type == Shape::CYLINDER)
    createCylinder(pose, dim, numero);
}

// ────────────────────────────────────────────────────────────────────────────
void ObjectFinder::filter_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
  if (!begin_ && !enable_create_)
    return;

  pcl::fromROSMsg(*cloud_msg, *cloud_);

  if (!retreiveObject())
    return;

  if (cluster_indices_.empty())
    return;

  std::vector<std::vector<double>> _tab_score;
  std::size_t cluster_num = 0;

  for (const auto& _c : cluster_indices_) {
    // ── Build per-cluster cloud ───────────────────────────────────────
    _tab_score.push_back(std::vector<double>());

    object_cloud_->clear();
    for (const auto& _idx : _c.indices)
      object_cloud_->points.push_back(cluster_cloud_->points[_idx]);

    object_cloud_->width = object_cloud_->size();
    object_cloud_->height = 1;
    object_cloud_->is_dense = true;

    if (object_cloud_->size() < 12) {
      for (std::size_t n_object = 0; n_object < objects_.size(); n_object++)
        for (std::size_t nb = 0; nb < objects_[n_object].number; nb++)
          _tab_score[cluster_num].push_back(-1.0);
      ++cluster_num;
      continue;
    }

    // ── Centroid + OBB ───────────────────────────────────────────────
    Eigen::Vector4f _centroid;
    pcl::PointXYZ _min_OBB, _max_OBB, _pos_OBB;
    Eigen::Matrix3f _rot_OBB;
    getCentroidAndOBB(object_cloud_, _centroid, _min_OBB, _max_OBB, _pos_OBB, _rot_OBB);

    double _dx = _max_OBB.x - _min_OBB.x;
    double _dy = _max_OBB.y - _min_OBB.y;
    double _dz = _max_OBB.z - _min_OBB.z;

    std::vector<double> _dim = {_dx, _dy, _dz};
    std::sort(_dim.begin(), _dim.end(), std::greater<double>());

    // ── Normal estimation (rebuild KdTree per cluster) ────────────────
    auto _cluster_tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    if (object_cloud_->empty())
      return;

    _cluster_tree->setInputCloud(object_cloud_);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> _ne;
    _ne.setInputCloud(object_cloud_);
    _ne.setSearchMethod(_cluster_tree);
    _ne.setKSearch(20);

    pcl::PointCloud<pcl::Normal>::Ptr _normals(new pcl::PointCloud<pcl::Normal>);
    _ne.compute(*_normals);

    for (std::size_t n_object = 0; n_object < objects_.size(); n_object++) {
      double _score = 0.0;
      std::vector<double> object_dim = objects_[n_object].dimension;
      Shape object_type = objects_[n_object].shape;

      // Penalty if object is high above the table
      _score -= (_centroid[2] > 1.0) ? 0.5 : 0;

      _score += penality_factor_ * boundingScore(_dim, object_dim, object_type);

      double _ns = normalScore(object_cloud_, _normals, _centroid);
      _score += _ns;

      if (object_type == Shape::BOX) {
        double _corner = cornerScore(object_cloud_);
        _score += penality_factor_ * _corner;

        double _cyl = cylinderScore(object_cloud_, _normals, object_dim, object_type);
        double _plane = planeScore(object_cloud_);
        _score -= penality_factor_ * (_cyl + _plane) * 0.5;
      }

      if (object_type == Shape::CYLINDER) {
        double _cyl = cylinderScore(object_cloud_, _normals, object_dim, object_type);
        _score += penality_factor_ * _cyl;

        double _corner = cornerScore(object_cloud_);
        double _plane = planeScore(object_cloud_);
        _score -= penality_factor_ * (_corner + _plane) * 0.5;
      }

      // Same score applies to every instance of this object type
      for (std::size_t nb = 0; nb < objects_[n_object].number; nb++)
        _tab_score[cluster_num].push_back(_score);
    }
    ++cluster_num;
  }

  // Current pose of each objects
  std::vector<Eigen::Vector3d> cluster_positions;
  for (const auto& c : cluster_indices_)
  {
      object_cloud_->clear();

      for (const auto& idx : c.indices)
          object_cloud_->push_back(cluster_cloud_->points[idx]);

      if(object_cloud_->empty())
      {
          cluster_positions.push_back(Eigen::Vector3d::Zero());
          continue;
      }

      Eigen::Vector4d centroid;
      pcl::compute3DCentroid(*object_cloud_, centroid);
      cluster_positions.push_back(centroid.head<>(3));
  }

  // ── Assignment: greedily match each object instance to its best cluster ──
  result_cloud_->clear();
  std::size_t nb_cluster = cluster_indices_.size();
  std::size_t flat_offset = 0;  // running offset into the flattened object-instance axis

  for (std::size_t n_object = 0; n_object < objects_.size(); n_object++) {
    auto type   = objects_[n_object].shape;
    auto dim    = objects_[n_object].dimension;
    auto number = objects_[n_object].number;
    
    
    for (std::size_t counter = 0; counter < number; counter++) {
      // Scores of every cluster against THIS specific instance
      std::vector<double> tmp_score;
      for (std::size_t n_cluster = 0; n_cluster < nb_cluster; n_cluster++)
      {
        double score = _tab_score.at(n_cluster).at(flat_offset + counter);

        if(objects_[n_object].have_stable[counter] && number > 1)
        {
            Eigen::Vector3d old = objects_[n_object].poses[counter].translation();
            Eigen::Vector3d detected = cluster_positions[n_cluster];

            double dist = (old - detected).norm();
            score -= dist * 2.0;
        }

        tmp_score.push_back(score);
        // RCLCPP_WARN(LOGGER, "Clust index: %ld  (score: %.3f)", n_cluster, score);
        // tmp_score.push_back(_tab_score.at(n_cluster).at(flat_offset + counter));
      }

      auto _it = std::max_element(tmp_score.begin(), tmp_score.end());
      int _index = static_cast<int>(std::distance(tmp_score.begin(), _it));

      
      // RCLCPP_WARN(LOGGER, "Best cluster index: %d  (score: %.3f)", _index, *_it);
      
      for (std::size_t i = 0; i < number; i++)
      _tab_score.at(_index).at(flat_offset + i) = 0.0;
      
      object_cloud_->clear();
      for (const auto& idx : cluster_indices_[_index].indices)
        object_cloud_->points.push_back(cluster_cloud_->points[idx]);
      
      if (*_it < 0.4)
      {
        objects_[n_object].poses[counter] = Eigen::Affine3d::Identity();
        continue;
      }

      // =====================================================================================================
      // Pose computation part
      
      if (!object_cloud_->points.empty())
      {
        // RCLCPP_WARN(LOGGER, "pose compute");
        Eigen::Affine3d current_pose = centroidBias(object_cloud_, dim, type);
        if (!objects_[n_object].have_stable[counter])
        {
          objects_[n_object].have_stable[counter] = true;
          objects_[n_object].poses[counter] = current_pose;
        }
        else
        {
          Eigen::Affine3d stable_pose = objects_[n_object].poses[counter];
          if(closePose(stable_pose, current_pose, 0.01, 0.17))
          {
            // RCLCPP_WARN(LOGGER, "stable");
            objects_[n_object].confidences[counter] = 0;
            objects_[n_object].candidates[counter] = Eigen::Affine3d::Identity();
            objects_[n_object].have_candidate[counter] = false;
            objects_[n_object].poses[counter] = poseMean(stable_pose, current_pose);
          }

          else
            if (!objects_[n_object].have_candidate[counter])
            {
              // RCLCPP_WARN(LOGGER, "NEW candidate");
              objects_[n_object].have_candidate[counter] = true;
              objects_[n_object].candidates[counter] = current_pose;
            }
            else
            {
              Eigen::Affine3d candidate_pose = objects_[n_object].candidates[counter];
              if (closePose(current_pose, candidate_pose, 0.01, 0.17))
              {
                // RCLCPP_WARN(LOGGER, "Close candidate");
                ++(objects_[n_object].confidences[counter]);
                objects_[n_object].candidates[counter] = poseMean(candidate_pose, current_pose);
              }
              else {
                // RCLCPP_WARN(LOGGER, "close to nothing");
                objects_[n_object].confidences[counter] = 0;
                objects_[n_object].poses[counter] = current_pose;
                objects_[n_object].candidates[counter] = Eigen::Affine3d::Identity();
                objects_[n_object].have_candidate[counter] = false;
              }
              
              if (objects_[n_object].confidences[counter] >= 3)
              {
                // RCLCPP_WARN(LOGGER, "enough Condifnde");
                objects_[n_object].confidences[counter] = 0;
                objects_[n_object].poses[counter] = poseMean(candidate_pose, current_pose);
                objects_[n_object].candidates[counter] = Eigen::Affine3d::Identity();
                objects_[n_object].have_candidate[counter] = false;
              }
            }
        }
      }
      // =====================================================================================================

      *result_cloud_ += *object_cloud_;
    }

    flat_offset += number;
  }

  // sensor_msgs::msg::PointCloud2 _output;
  // pcl::toROSMsg(*cluster_cloud_, _output);
  // _output.header = cloud_msg->header;
  // pub_filtered_->publish(_output);

  cluster_indices_.clear();  

  if (enable_create_)
    createAllObjects();
}

void ObjectFinder::createAllObjects()
{
  // ── Construction of collision objects ─────────────────────────────────
  for (std::size_t n_object = 0; n_object < objects_.size(); n_object++) {
    auto type = objects_[n_object].shape;
    auto dim  = objects_[n_object].dimension;

    for (std::size_t counter = 0; counter < objects_[n_object].number; counter++) {
      
        Eigen::Affine3d pose = objects_[n_object].poses[counter];
        if (pose.translation() == Eigen::Affine3d::Identity().translation())
          continue;
        
        createObstacle(pose, dim, type, counter);
    }
  }
}

bool ObjectFinder::closePose(Eigen::Affine3d& stable, Eigen::Affine3d& current, double pos_thrsld, double ang_thrsld)
{
  Eigen::Vector3d current_position = current.translation();
  Eigen::Quaterniond current_quat(current.rotation());
  
  Eigen::Vector3d stable_position = stable.translation();
  Eigen::Quaterniond stable_quat(stable.rotation());

  double pos_err = (stable_position - current_position).norm();
  bool pos_close = pos_err < pos_thrsld;
  
  Eigen::Quaterniond quat_err = (stable_quat.conjugate() * current_quat);
  quat_err.normalize();

  double ang_err = Eigen::AngleAxisd(quat_err).angle();
  bool rot_close = ang_err < ang_thrsld;

  return pos_close && rot_close;
}

Eigen::Affine3d ObjectFinder::poseMean(Eigen::Affine3d& p1, Eigen::Affine3d& p2)
{
  Eigen::Vector3d p1_position = p1.translation();
  Eigen::Quaterniond p1_quat(p1.rotation());

  Eigen::Vector3d p2_position = p2.translation();
  Eigen::Quaterniond p2_quat(p2.rotation());

  Eigen::Vector3d pos_mean = 0.5 * (p1_position + p2_position);
  Eigen::Quaterniond quat_mean = p1_quat.slerp(0.5, p2_quat);

  Eigen::Affine3d mean_pose = Eigen::Affine3d::Identity();
  mean_pose.translation() = pos_mean;
  mean_pose.linear() = quat_mean.toRotationMatrix();

  return mean_pose;
}

// ================================================================================================

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  // moveit::planning_interface::PlanningSceneInterface _planning_scene;
  // auto node = std::make_shared<ObjectFinder>(&_planning_scene);
  auto node = std::make_shared<ObjectFinder>();

  RCLCPP_INFO(LOGGER, "Object Finder Node ON");
  rclcpp::spin(node);
  RCLCPP_INFO(LOGGER, "Object Finder Node OFF");

  rclcpp::shutdown();
  return 0;
}