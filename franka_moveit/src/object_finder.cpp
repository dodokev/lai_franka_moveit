#include "franka_moveit/object_finder.hpp"
#include <pcl/common/pca.h>
#include <pcl/filters/passthrough.h>
#include <boost/algorithm/string/split.hpp>

static auto const LOGGER = rclcpp::get_logger("object_finder");

ObjectFinder::ObjectFinder(moveit::planning_interface::PlanningSceneInterface* ps)
    : Node("object_finder"), planning_scene_(ps) {
  sub_unfilter_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/points_used", rclcpp::SensorDataQoS(),
      std::bind(&ObjectFinder::filter_callback, this, std::placeholders::_1));
  pub_filtered_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/visualize_cloud",
                                                                        rclcpp::SensorDataQoS());

  pub_centroid_ = this->create_publisher<visualization_msgs::msg::Marker>("/centroid", 10);

  table_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  object_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  object_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  tree_ = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();

  this->declare_parameter<std::string>("size", "");
  std::string _param = this->get_parameter("size").as_string();
  std::vector<std::string> _param_split;
  boost::split(_param_split, _param, boost::is_any_of(","));
  for (const auto& _p : _param_split)
    object_size_.push_back(std::stof(_p));

  type_object_ = static_cast<SHAPE>(object_size_.size());
  // BUG FIX: cast to enum so switch/comparison is type-safe
}

// ────────────────────────────────────────────────────────────────────────────
void ObjectFinder::retreiveObject() {
  pcl::PassThrough<pcl::PointXYZ> pass_;
  pass_.setFilterFieldName("y");
  pass_.setFilterLimits(-0.5, 0.45);  // TUNE
  pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_(new pcl::PointCloud<pcl::PointXYZ>);

  pass_.setInputCloud(cloud_);
  pass_.filter(*cropped_);

  pcl::SACSegmentation<pcl::PointXYZ> _seg;
  _seg.setOptimizeCoefficients(true);
  _seg.setModelType(pcl::SACMODEL_PLANE);
  _seg.setMethodType(pcl::SAC_RANSAC);
  _seg.setDistanceThreshold(0.025);
  _seg.setInputCloud(cropped_);

  pcl::PointIndices::Ptr _inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr _coefficients(new pcl::ModelCoefficients);
  _seg.segment(*_inliers, *_coefficients);

  pcl::ExtractIndices<pcl::PointXYZ> _extract;
  _extract.setInputCloud(cloud_);
  _extract.setIndices(_inliers);

  _extract.setNegative(false);
  _extract.filter(*table_);

  _extract.setNegative(true);
  _extract.filter(*object_cloud_);

  // BUG FIX: tree must be set before EuclideanClusterExtraction uses it
  tree_->setInputCloud(object_cloud_);

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> _ec;
  _ec.setClusterTolerance(0.025);
  _ec.setMinClusterSize(50);
  _ec.setSearchMethod(tree_);
  _ec.setInputCloud(object_cloud_);
  _ec.extract(cluster_indices_);
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
double ObjectFinder::boundingScore(std::vector<double> dim) {
  // dims already sorted descending by caller
  std::vector<double> _copy = object_size_;
  std::sort(_copy.begin(), _copy.end(), std::greater<double>());

  double _score = 0.0;
  switch (type_object_) {
    case SHAPE::SPHERE:
      _score = std::sqrt(std::pow(dim[0] - _copy[0], 2));
      break;
    case SHAPE::CYLINDER:
      _score = std::sqrt(std::pow(dim[0] - _copy[0], 2) + std::pow(2 * dim[1] - _copy[1], 2));
      break;
    case SHAPE::BOX:
      _score = std::sqrt(std::pow(dim[0] - _copy[0], 2) + std::pow(dim[1] - _copy[1], 2) +
                         std::pow(dim[2] - _copy[2], 2));
      break;
    default:
      RCLCPP_ERROR(LOGGER, "INCORRECT SHAPE TYPE");
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

  pcl::PointIndices::Ptr _inliers1(new pcl::PointIndices);
  pcl::PointIndices::Ptr _inliers2(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr _coeff1(new pcl::ModelCoefficients);
  pcl::ModelCoefficients::Ptr _coeff2(new pcl::ModelCoefficients);

  pcl::PointCloud<pcl::PointXYZ>::Ptr _remaining(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> _extract;

  // First plane
  _seg.setInputCloud(obj_cloud);
  _seg.segment(*_inliers1, *_coeff1);

  if (_inliers1->indices.empty())
    return 0.0;

  _extract.setInputCloud(obj_cloud);
  _extract.setIndices(_inliers1);
  _extract.setNegative(true);
  _extract.filter(*_remaining);

  if (_remaining->points.size() < 12) {
    RCLCPP_DEBUG(LOGGER, "Not enough points remaining for second plane");
    return 0.0;
  }

  // Second plane from remaining cloud
  _seg.setInputCloud(_remaining);
  _seg.segment(*_inliers2, *_coeff2);

  if (_inliers2->indices.empty())
    return 0.0;

  Eigen::Vector3f _n1(_coeff1->values[0], _coeff1->values[1], _coeff1->values[2]);
  Eigen::Vector3f _n2(_coeff2->values[0], _coeff2->values[1], _coeff2->values[2]);

  // BUG FIX: _inliers1 / _inliers2 were used as raw pointers — undefined.
  // Correct: use ->indices.size() and cast to double.
  double _ratio1 = static_cast<double>(_inliers1->indices.size()) / obj_cloud->size();
  double _ratio2 = static_cast<double>(_inliers2->indices.size()) / obj_cloud->size();

  // Score: planes should be perpendicular (dot ≈ 0) and cover most points
  double _perp = 1.0 - std::fabs(_n1.normalized().dot(_n2.normalized()));
  return _perp * (_ratio1 + _ratio2) * 0.5;
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
                                   pcl::PointCloud<pcl::Normal>::Ptr normals) {
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
  // For CYLINDER: object_size_ = {radius, height} → radius is index 0
  double _r = (type_object_ == SHAPE::CYLINDER) ? object_size_[1] : object_size_[1];
  _seg.setRadiusLimits(_r * 0.85, _r * 1.15);  // wider tolerance for partial view

  _seg.setInputCloud(obj_cloud);

  pcl::ModelCoefficients::Ptr _coeff(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr _inliers(new pcl::PointIndices);
  _seg.segment(*_inliers, *_coeff);

  return static_cast<double>(_inliers->indices.size()) / obj_cloud->size();
}

// ────────────────────────────────────────────────────────────────────────────
void ObjectFinder::centroidBiasBox(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud,
                                   Eigen::Affine3d& pose) {
  // ---------------------------
  // 1. Compute centroid
  // ---------------------------
  Eigen::Vector4f centroid4;
  pcl::compute3DCentroid(*obj_cloud, centroid4);
  Eigen::Vector3d centroid = centroid4.head<3>().cast<double>();

  // ---------------------------
  // 2. Plane segmentation (up to 3 planes)
  // ---------------------------
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setMaxIterations(100);

  pcl::ExtractIndices<pcl::PointXYZ> extract;

  struct PlaneInfo {
    Eigen::Vector3d normal;
    Eigen::Vector3d mean_point;  // a point on the plane (inlier centroid)
  };
  std::vector<PlaneInfo> planes;

  pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(new pcl::PointCloud<pcl::PointXYZ>(*obj_cloud));

  // Require at least 5% of remaining cloud to accept a plane
  const float min_inlier_ratio = 0.05f;

  Eigen::Vector3d z_world(0.0, 0.0, 1.0);
  double align_threshold{0.1};

  for (int i = 0; i < 5; ++i) {
    if (remaining->size() < 20)
      break;

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);

    seg.setInputCloud(remaining);
    seg.segment(*inliers, *coeff);

    if (inliers->indices.empty())
      break;

    if ((float)inliers->indices.size() / (float)remaining->size() < min_inlier_ratio)
      break;

    // Compute the centroid of inliers → a point on this face
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

    double alignement = std::abs(normal.dot(z_world));
    if (alignement > align_threshold && alignement < (1 - align_threshold))
      continue;

    // Orient normal to point AWAY from the cloud centroid
    // (i.e. outward-facing, toward the sensor)
    if (normal.dot(face_centroid - centroid) < 0)
      normal = -normal;

    planes.push_back({normal, face_centroid});

    // Remove inliers and continue
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setNegative(true);
    extract.filter(*tmp);
    remaining = tmp;
  }

  if (planes.size() < 2) {
    // Fallback: PCA only, no bias
    RCLCPP_WARN(LOGGER, "centroidBias: fewer than 2 planes found, falling back to PCA centroid.");
    pose.linear() = Eigen::Matrix3d::Identity();
    pose.translation() = centroid;
    return;
  }

  // ---------------------------
  // 3. Build orthonormal frame from the two best normals
  //
  //  n0, n1 are two outward face normals → they are (ideally) orthogonal.
  //  Their cross product gives the third box axis (the "edge" direction).
  // ---------------------------
  Eigen::Vector3d n0 = planes[0].normal;
  Eigen::Vector3d n1 = planes[1].normal;

  // Sanity: normals should be roughly perpendicular for a box
  double dot = std::abs(n0.dot(n1));
  if (dot > 0.3)  // ~17° from parallel — something went wrong
  {
    RCLCPP_WARN(
        rclcpp::get_logger("object_finder"),
        "centroidBias: plane normals are not perpendicular (|dot|=%.2f), result may be unreliable.",
        dot);
  }

  // Third axis = edge shared by the two visible faces
  Eigen::Vector3d n2 = n0.cross(n1).normalized();

  // Re-orthogonalize for numerical stability
  n1 = n2.cross(n0).normalized();
  n0 = n1.cross(n2).normalized();  // now all three are truly orthonormal

  // ---------------------------
  // 4. Map axes → box dimensions
  //
  //  We want the longest dimension along Z, but we don't force it —
  //  instead we assign object_size_ dimensions to the nearest axis.
  // ---------------------------
  // Candidate axes and known half-lengths
  std::array<Eigen::Vector3d, 3> axes = {n0, n1, n2};
  std::array<double, 3> sizes = {
      object_size_[0],  // longest
      object_size_[1],
      object_size_[2]  // shortest
  };

  // Sort sizes descending
  std::array<int, 3> size_order = {0, 1, 2};
  std::sort(size_order.begin(), size_order.end(),
            [&](int a, int b) { return sizes[a] > sizes[b]; });

  // Assign: PCA on the three axes to match sizes
  // (use the spread of the cloud projected onto each axis)
  std::array<double, 3> extents;
  for (int i = 0; i < 3; ++i) {
    float min_proj = 1e9f;
    float max_proj = -1e9f;
    for (const auto& pt : *obj_cloud) {
      Eigen::Vector3d p(pt.x, pt.y, pt.z);
      double proj = p.dot(axes[i]);
      min_proj = std::min(min_proj, (float)proj);
      max_proj = std::max(max_proj, (float)proj);
    }
    extents[i] = max_proj - min_proj;
  }

  // Sort axes by observed extent (descending)
  std::array<int, 3> axis_order = {0, 1, 2};
  std::sort(axis_order.begin(), axis_order.end(),
            [&](int a, int b) { return extents[a] > extents[b]; });

  // axis_order[k] is the axis index with the k-th largest extent
  // size_order[k] is the dimension index with the k-th largest size
  // → axis axis_order[k] corresponds to dimension size_order[k]
  std::array<double, 3> half_sizes;
  std::array<Eigen::Vector3d, 3> sorted_axes;
  for (int k = 0; k < 3; ++k) {
    sorted_axes[k] = axes[axis_order[k]];
    half_sizes[k] = sizes[size_order[k]] / 2.0;
  }

  // Final frame: X=largest, Y=medium, Z=smallest  (or whatever your convention)
  Eigen::Vector3d x_axis = sorted_axes[0];
  Eigen::Vector3d y_axis = sorted_axes[1];
  Eigen::Vector3d z_axis = sorted_axes[2];

  // Ensure right-handed
  if (x_axis.cross(y_axis).dot(z_axis) < 0)
    z_axis = -z_axis;

  // ---------------------------
  // 5. Bias centroid toward true box center
  //
  //  For each visible face: its outward normal points away from center.
  //  The centroid is biased toward that face by half the box depth
  //  in that direction → shift it back inward by (L/2 - observed offset).
  //
  //  More robustly: shift = -n * (L/2) for each visible face normal,
  //  starting from the face centroid.
  //  Box center ≈ face_centroid - n * (L/2)
  //  Average over all visible faces.
  // ---------------------------
  Eigen::Vector3d box_center = Eigen::Vector3d::Zero();

  for (const auto& plane : planes) {
    const Eigen::Vector3d& n = plane.normal;
    const Eigen::Vector3d& fp = plane.mean_point;

    // Find which axis this normal is closest to, get the right half-size
    double best_dot = -1.0;
    double half_L = 0.0;
    for (int k = 0; k < 3; ++k) {
      double d = std::abs(n.dot(sorted_axes[k]));
      if (d > best_dot) {
        best_dot = d;
        half_L = half_sizes[k];
      }
    }

    // Center estimate from this face: move inward by half the box depth
    box_center += fp - n * half_L;
  }

  box_center /= (double)planes.size();

  // ---------------------------
  // 6. Build final pose
  // ---------------------------
  Eigen::Matrix3d R;
  R.col(0) = x_axis;
  R.col(1) = y_axis;
  R.col(2) = z_axis;

  pose.linear() = R;
  pose.translation() = box_center;
}

void ObjectFinder::centroidBiasCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud,
                                        Eigen::Affine3d& pose) {
  double known_radius = object_size_[1];
  double known_height = object_size_[0];

  // ---------------------------
  // 1. Raw centroid (for fallback + axial bias heuristic only)
  // ---------------------------
  Eigen::Vector4f centroid4;
  pcl::compute3DCentroid(*obj_cloud, centroid4);
  Eigen::Vector3d centroid = centroid4.head<3>().cast<double>();

  // ---------------------------
  // 2. PCA → reliable axis direction seed
  //    For a horizontal cylinder, col(0) is always the height axis.
  // ---------------------------
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(obj_cloud);
  Eigen::Vector3d z_axis = pca.getEigenVectors().col(0).cast<double>().normalized();

  // Pin: make dominant component always positive → no random 180° flips
  int dominant = 0;
  z_axis.cwiseAbs().maxCoeff(&dominant);
  if (z_axis[dominant] < 0)
    z_axis = -z_axis;

  RCLCPP_DEBUG(rclcpp::get_logger("object_finder"),
               "centroidBiasCylinder: cloud=%zu  h=%.3f r=%.3f", obj_cloud->size(), known_height,
               known_radius);
  RCLCPP_DEBUG(rclcpp::get_logger("object_finder"), "centroidBiasCylinder: z_axis=[%.3f %.3f %.3f]",
               z_axis.x(), z_axis.y(), z_axis.z());

  // ---------------------------
  // 3. Find true axis center radially
  //
  //  All surface points lie at distance known_radius from the true axis.
  //  Project every point into the 2D plane perpendicular to z_axis,
  //  then solve for the circle center of known radius that best fits them.
  //
  //  In 2D: |p_i - c|^2 = R^2
  //  Expanding: |p_i|^2 - 2*p_i·c + |c|^2 = R^2
  //  Rearranging: 2*p_i·c - |c|^2 + R^2 = |p_i|^2
  //  This is nonlinear in c, but linearized by substituting d = |c|^2 - R^2:
  //  2*p_i·c - d = |p_i|^2   → linear system Ax = b, x = [cx, cy, d]
  // ---------------------------

  // Build two orthonormal basis vectors spanning the plane ⊥ to z_axis
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

  // Project all points into 2D (u, v) plane centered at raw centroid
  int N = obj_cloud->size();
  Eigen::MatrixXd A(N, 3);
  Eigen::VectorXd b(N);

  for (int i = 0; i < N; ++i) {
    Eigen::Vector3d p(obj_cloud->points[i].x, obj_cloud->points[i].y, obj_cloud->points[i].z);

    // Project into plane ⊥ z_axis, relative to raw centroid
    Eigen::Vector3d dp = p - centroid;
    double u = dp.dot(x_axis);
    double v = dp.dot(y_axis);

    // 2*p·c - d = |p|^2   where d = |c|^2 - R^2
    A(i, 0) = 2.0 * u;
    A(i, 1) = 2.0 * v;
    A(i, 2) = -1.0;
    b(i) = u * u + v * v;
  }

  // Solve least squares: x = [cu, cv, d]
  Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
  double cu = x(0);  // circle center in 2D (relative to raw centroid)
  double cv = x(1);

  // Back to 3D: axis_center = centroid + cu*ex + cv*ey
  Eigen::Vector3d axis_center = centroid + cu * x_axis + cv * y_axis;

  // Sanity check: fitted radius should match known_radius
  double fitted_radius = std::sqrt(x(2) + known_radius * known_radius);
  RCLCPP_DEBUG(rclcpp::get_logger("object_finder"),
               "centroidBiasCylinder: fitted_radius=%.4f  known_radius=%.4f  error=%.4f",
               fitted_radius, known_radius, std::abs(fitted_radius - known_radius));
  RCLCPP_DEBUG(rclcpp::get_logger("object_finder"),
               "centroidBiasCylinder: radial correction=[%.4f %.4f %.4f]",
               (axis_center - centroid).x(), (axis_center - centroid).y(),
               (axis_center - centroid).z());

  // ---------------------------
  // 4. Bias along Z (axial correction) — relative to axis_center now
  // ---------------------------
  double min_t = 1e9;
  double max_t = -1e9;

  for (const auto& pt : *obj_cloud) {
    Eigen::Vector3d p(pt.x, pt.y, pt.z);
    double t = (p - axis_center).dot(z_axis);
    min_t = std::min(min_t, t);
    max_t = std::max(max_t, t);
  }

  double observed_height = max_t - min_t;
  double bias = 0.0;

  if (observed_height >= known_height * 0.85) {
    bias = (min_t + max_t) / 2.0;
    RCLCPP_DEBUG(rclcpp::get_logger("object_finder"),
                 "centroidBiasCylinder: both caps visible, bias=%.3f", bias);
  } else {
    if (std::abs(max_t) >= std::abs(min_t)) {
      bias = max_t - known_height / 2.0;
      RCLCPP_DEBUG(rclcpp::get_logger("object_finder"),
                   "centroidBiasCylinder: max cap anchor (max_t=%.3f), bias=%.3f", max_t, bias);
    } else {
      bias = min_t + known_height / 2.0;
      RCLCPP_DEBUG(rclcpp::get_logger("object_finder"),
                   "centroidBiasCylinder: min cap anchor (min_t=%.3f), bias=%.3f", min_t, bias);
    }
  }

  Eigen::Vector3d true_center = axis_center + bias * z_axis;

  RCLCPP_DEBUG(rclcpp::get_logger("object_finder"),
               "centroidBiasCylinder: true_center=[%.3f %.3f %.3f]", true_center.x(),
               true_center.y(), true_center.z());

  // ---------------------------
  // 6. Build final pose
  // ---------------------------
  Eigen::Matrix3d R;
  R.col(0) = x_axis;
  R.col(1) = y_axis;
  R.col(2) = z_axis;

  pose.linear() = R;
  pose.translation() = true_center;
}

// ────────────────────────────────────────────────────────────────────────────
void ObjectFinder::centroidBias(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud,
                                Eigen::Affine3d& pose) {
  if (type_object_ == SHAPE::BOX)
    centroidBiasBox(obj_cloud, pose);
  if (type_object_ == SHAPE::CYLINDER)
    centroidBiasCylinder(obj_cloud, pose);
}

// ────────────────────────────────────────────────────────────────────────────
void ObjectFinder::createBox(Eigen::Affine3d& pose) {
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);

  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions = {object_size_[0], object_size_[1], object_size_[2]};

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

  obj_created = true;
}

// ────────────────────────────────────────────────────────────────────────────
void ObjectFinder::createCylinder(Eigen::Affine3d& pose) {
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);

  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = {object_size_[0], object_size_[1]};

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

  obj_created = true;
}

// ────────────────────────────────────────────────────────────────────────────
void ObjectFinder::createObstacle(Eigen::Affine3d& pose) {
  if (type_object_ == SHAPE::BOX)
    createBox(pose);
  if (type_object_ == SHAPE::CYLINDER)
    createCylinder(pose);
}

// ────────────────────────────────────────────────────────────────────────────
void ObjectFinder::filter_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
  RCLCPP_DEBUG(LOGGER, "Received point cloud");
  pcl::fromROSMsg(*cloud_msg, *cloud_);

  retreiveObject();
  // NOTE: tree_ is now set inside retreiveObject() on object_cloud_

  if (cluster_indices_.empty()) {
    RCLCPP_WARN(LOGGER, "No clusters found");
    return;
  }

  std::vector<double> _tab_score;
  _tab_score.reserve(cluster_indices_.size());

  for (const auto& _c : cluster_indices_) {
    double _score = 0.0;

    // ── Build per-cluster cloud ───────────────────────────────────────
    object_->clear();
    for (const auto& _idx : _c.indices)
      object_->points.push_back(object_cloud_->points[_idx]);

    object_->width = object_->size();
    object_->height = 1;
    object_->is_dense = true;

    if (object_->size() < 12) {
      _tab_score.push_back(-1.0);
      continue;
    }

    // ── Centroid + OBB ───────────────────────────────────────────────
    Eigen::Vector4f _centroid;
    pcl::PointXYZ _min_OBB, _max_OBB, _pos_OBB;
    Eigen::Matrix3f _rot_OBB;
    getCentroidAndOBB(object_, _centroid, _min_OBB, _max_OBB, _pos_OBB, _rot_OBB);

    // Penality if object is high -- Concentrated on table
    _score -= (_centroid[2] > 1.0) ? 0.5 : 0;

    double _dx = _max_OBB.x - _min_OBB.x;
    double _dy = _max_OBB.y - _min_OBB.y;
    double _dz = _max_OBB.z - _min_OBB.z;

    std::vector<double> _dims = {_dx, _dy, _dz};
    std::sort(_dims.begin(), _dims.end(), std::greater<double>());

    // BUG FIX: was -= ; a high bounding match is a positive signal
    _score += penality_factor_ * boundingScore(_dims);

    // ── Normal estimation (rebuild KdTree per cluster) ────────────────
    // BUG FIX: tree_ was pointing to object_cloud_; rebuild on the cluster
    auto _cluster_tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    _cluster_tree->setInputCloud(object_);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> _ne;
    _ne.setInputCloud(object_);
    _ne.setSearchMethod(_cluster_tree);
    _ne.setKSearch(20);

    pcl::PointCloud<pcl::Normal>::Ptr _normals(new pcl::PointCloud<pcl::Normal>);
    _ne.compute(*_normals);

    // Generic radial normal score (useful for sphere/cylinder, neutral for box)
    double _ns = normalScore(object_, _normals, _centroid);
    _score += _ns;

    // ── Shape-specific scoring ────────────────────────────────────────
    if (type_object_ == SHAPE::BOX) {
      // Two perpendicular faces → boost score
      double _corner = cornerScore(object_);
      _score += penality_factor_ * _corner;

      // Cylinder or single-plane fit → penalise
      double _cyl = cylinderScore(object_, _normals);
      double _plane = planeScore(object_);
      // BUG FIX: penalty was sqrt(pow(a,factor)+pow(b,factor)) with
      // factor=penality_factor_ as exponent — semantically wrong.
      // Correct: weighted linear penalty.
      _score -= penality_factor_ * (_cyl + _plane) * 0.5;
    }

    if (type_object_ == SHAPE::CYLINDER) {
      double _cyl = cylinderScore(object_, _normals);
      _score += penality_factor_ * _cyl;

      double _corner = cornerScore(object_);
      double _plane = planeScore(object_);
      _score -= penality_factor_ * (_corner + _plane) * 0.5;
    }

    _tab_score.push_back(_score);
  }

  // ── Select best cluster ───────────────────────────────────────────────
  auto _it = std::max_element(_tab_score.begin(), _tab_score.end());
  int _index = static_cast<int>(std::distance(_tab_score.begin(), _it));

  RCLCPP_DEBUG(LOGGER, "Best cluster index: %d  (score: %.3f)", _index, *_it);

  // =====================================================================
  // ── Rebuild object_ from best cluster ────────────────────────────────
  object_->clear();
  for (const auto& idx : cluster_indices_[_index].indices)
    object_->points.push_back(object_cloud_->points[idx]);

  if (!object_->points.empty() && !obj_created) {
    Eigen::Affine3d _pose;
    centroidBias(object_, _pose);

    visualization_msgs::msg::Marker _marker;
    _marker.header.frame_id = "world";
    _marker.header.stamp = rclcpp::Clock().now();

    _marker.ns = "centroid";
    _marker.id = 0;
    _marker.type = visualization_msgs::msg::Marker::SPHERE;
    _marker.action = visualization_msgs::msg::Marker::ADD;

    // Position = your centroid
    _marker.pose.position.x = _pose.translation().x();
    _marker.pose.position.y = _pose.translation().y();
    _marker.pose.position.z = _pose.translation().z();

    if (!obj_created)
      createObstacle(_pose);

    // No rotation needed
    _marker.pose.orientation.w = 1.0;

    // Size of the sphere
    _marker.scale.x = 0.02;
    _marker.scale.y = 0.02;
    _marker.scale.z = 0.02;

    // Color (red here)
    _marker.color.r = 1.0;
    _marker.color.g = 0.0;
    _marker.color.b = 0.0;
    _marker.color.a = 1.0;

    pub_centroid_->publish(_marker);
  }

  // =====================================================================
  // ── Remove target from remaining cloud and re-add table ──────────────
  pcl::PointIndices::Ptr _best_indices(new pcl::PointIndices);
  _best_indices->indices = cluster_indices_[_index].indices;

  pcl::PointCloud<pcl::PointXYZ>::Ptr _no_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> _extract;
  _extract.setInputCloud(object_cloud_);
  _extract.setIndices(_best_indices);
  _extract.setNegative(true);
  _extract.filter(*_no_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr _result_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  *_result_cloud = *_no_cloud;
  *_result_cloud += *table_;  // BUG FIX: was a manual loop; use PCL concatenation

  _result_cloud->width = _result_cloud->points.size();
  _result_cloud->height = 1;
  _result_cloud->is_dense = true;

  // ── Publish the detected object cloud ────────────────────────────────
  sensor_msgs::msg::PointCloud2 _output;
  pcl::toROSMsg(*object_, _output);
  _output.header = cloud_msg->header;
  pub_filtered_->publish(_output);

  cluster_indices_.clear();
}

// ────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  moveit::planning_interface::PlanningSceneInterface _planning_scene;
  auto node = std::make_shared<ObjectFinder>(&_planning_scene);

  RCLCPP_INFO(LOGGER, "Object Finder Node ON");
  rclcpp::spin(node);
  RCLCPP_INFO(LOGGER, "Object Finder Node OFF");

  rclcpp::shutdown();
  return 0;
}