#include "franka_moveit/outlier_filter.hpp"

static auto const LOGGER = rclcpp::get_logger("outlier_filter");

OutlierFilter::OutlierFilter()
: Node("outlier_filter")
{
    // sub_unfilter = this->create_subscription<sensor_msgs::msg::PointCloud2>("/camera/camera/depth/color/points", rclcpp::SensorDataQoS(), std::bind(&OutlierFilter::filter_callback, this, std::placeholders::_1));
    sub_unfilter = this->create_subscription<sensor_msgs::msg::PointCloud2>("/points_filtered_world", rclcpp::SensorDataQoS(), std::bind(&OutlierFilter::filter_callback, this, std::placeholders::_1));
    pub_filtered = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_used", rclcpp::SensorDataQoS());

    cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cropped_Xaxis = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cropped_Yaxis = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cropped_Zaxis = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    downsampled_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    tree_ = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();

    pass_Xaxis.setFilterFieldName("x");
    pass_Xaxis.setFilterLimits(-1.0, 1.0); // TUNE
    pass_Yaxis.setFilterFieldName("y");
    pass_Yaxis.setFilterLimits(-1.0, 1.0); // TUNE
    pass_Zaxis.setFilterFieldName("z");
    pass_Zaxis.setFilterLimits(0, 1.0); // TUNE

    vg_.setLeafSize(0.02f, 0.02f, 0.02f); // TUNE

    ec_.setClusterTolerance(0.1); // TUNE
    ec_.setMinClusterSize(200); // TUNE

    min_keep_size_ = 200; // TUNE
}

void OutlierFilter::filter_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
    // RCLCPP_INFO(LOGGER, "Cloud get ...");
    pcl::fromROSMsg(*cloud_msg, *cloud_);

    pass_Xaxis.setInputCloud(cloud_);
    pass_Xaxis.filter(*cropped_Xaxis);

    pass_Yaxis.setInputCloud(cropped_Xaxis);
    pass_Yaxis.filter(*cropped_Yaxis);
    
    pass_Zaxis.setInputCloud(cropped_Yaxis);
    pass_Zaxis.filter(*cropped_Zaxis);

    vg_.setInputCloud(cropped_Zaxis);
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