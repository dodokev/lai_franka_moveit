#include "franka_moveit/outlier_filter.hpp"

static auto const LOGGER = rclcpp::get_logger("outlier_filter");

OutlierFilter::OutlierFilter()
: Node("outlier_filter")
{
    // sub_unfilter = this->create_subscription<sensor_msgs::msg::PointCloud2>("/camera/camera/depth/color/points", rclcpp::SensorDataQoS(), std::bind(&OutlierFilter::filter_callback, this, std::placeholders::_1));
    sub_unfilter = this->create_subscription<sensor_msgs::msg::PointCloud2>("/points_filtered_world", rclcpp::SensorDataQoS(), std::bind(&OutlierFilter::filter_callback, this, std::placeholders::_1));
    pub_filtered = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_used", rclcpp::SensorDataQoS());

    table_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    object_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cropped_Xaxis = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cropped_Yaxis = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cropped_Zaxis = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    downsampled_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    tree_ = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();

    pass_Xaxis.setFilterFieldName("x");
    pass_Xaxis.setFilterLimits(-0.25, 1.0); // TUNE
    pass_Yaxis.setFilterFieldName("y");
    pass_Yaxis.setFilterLimits(-1.0, 0.5); // TUNE
    pass_Zaxis.setFilterFieldName("z");
    pass_Zaxis.setFilterLimits(0, 1.0); // TUNE

    const float voxel_size = 0.02f;

    vg_.setLeafSize(voxel_size, voxel_size, voxel_size); // TUNE

    ec_.setClusterTolerance(0.025); // TUNE

    min_keep_size_ = 50; // TUNE
    ec_.setMinClusterSize(min_keep_size_);
}

void OutlierFilter::planeFitting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::SACSegmentation<pcl::PointXYZ> _seg;
    _seg.setOptimizeCoefficients(true);
    _seg.setModelType(pcl::SACMODEL_PLANE);
    _seg.setMethodType(pcl::SAC_RANSAC);
    _seg.setDistanceThreshold(0.025);
    _seg.setInputCloud(cloud);

    Eigen::Vector3f z(0, 0, 1);
    _seg.setAxis(z);
    _seg.setEpsAngle(1e-6);

    pcl::PointIndices::Ptr _inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr _coefficients(new pcl::ModelCoefficients);
    _seg.segment(*_inliers, *_coefficients);

    double a = _coefficients->values[0];
    double b = _coefficients->values[1];
    double c = _coefficients->values[2];
    double d = _coefficients->values[3];

    pcl::ExtractIndices<pcl::PointXYZ> _extract;
    _extract.setInputCloud(cloud);
    _extract.setIndices(_inliers);

    _extract.setNegative(false);
    _extract.filter(*table_);

    _extract.setNegative(true);
    _extract.filter(*object_);

    Eigen::Vector3f n(a, b, c);
    float norm_sq = n.squaredNorm();

    for (auto& pt : table_->points)
    {
        Eigen::Vector3f p(pt.x, pt.y, pt.z);

        float dist = (a * pt.x + b * pt.y + c * pt.z + d) / norm_sq;
        Eigen::Vector3f p_proj = p - dist * n;

        pt.x = p_proj.x();
        pt.y = p_proj.y();
        pt.z = p_proj.z();
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cld(new pcl::PointCloud<pcl::PointXYZ>(*table_));
    vg_.setInputCloud(tmp_cld);
    vg_.filter(*table_);

    for (auto& pt : table_->points)
        pt.z -= 0.0225;
}

void OutlierFilter::filter_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
    // RCLCPP_INFO(LOGGER, "Cloud get ...");
    pcl::fromROSMsg(*cloud_msg, *cloud_);

    if (cloud_->points.empty())
        return;

    pass_Xaxis.setInputCloud(cloud_);
    pass_Xaxis.filter(*cropped_Xaxis);

    pass_Yaxis.setInputCloud(cropped_Xaxis);
    pass_Yaxis.filter(*cropped_Yaxis);
    
    pass_Zaxis.setInputCloud(cropped_Yaxis);
    pass_Zaxis.filter(*cropped_Zaxis);

    planeFitting(cropped_Zaxis);
    // *object_ += *table_;

    vg_.setInputCloud(object_);
    // vg_.setInputCloud(cropped_Zaxis);
    vg_.filter(*downsampled_);

    tree_->setInputCloud(downsampled_);

    ec_.setSearchMethod(tree_);
    ec_.setInputCloud(downsampled_);
    ec_.extract(cluster_indices_);
    
    // ================================================================================
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& indices : cluster_indices_)
    {
        if (indices.indices.size() > min_keep_size_)
            for ( int idx : indices.indices )
            {
                downsampled_->points[idx].z -= 0.02;
                filtered->points.push_back(downsampled_->points[idx]);
            }
    }

    *filtered += *table_;

    filtered->width = filtered->points.size();
    filtered->height = 1;

    filtered->is_dense = true;

    // ================================================================================

    // RCLCPP_INFO(LOGGER, "SIZE : %ld", filtered->points.size());

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*filtered, output);
    output.header = cloud_msg->header;
    
    pub_filtered->publish(output);
    
    cluster_indices_.clear();
    filtered->clear();
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<OutlierFilter>();
    
    RCLCPP_INFO(LOGGER, "Filter Node ON");

    rclcpp::spin(node);

    RCLCPP_INFO(LOGGER, "Stop Filter Node");
    rclcpp::shutdown();
    return 0;
}