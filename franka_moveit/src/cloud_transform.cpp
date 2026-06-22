#include "franka_moveit/cloud_transform.hpp"

static auto const LOGGER = rclcpp::get_logger("cloud_to_world");

CloudToWorld::CloudToWorld()
  : Node("cloud_to_world"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
    // Declare parameters with default values
    this->declare_parameter<std::string>("input_topic", "/cloud_in");
    this->declare_parameter<std::string>("output_topic", "/cloud_out");
    this->declare_parameter<std::string>("target_frame", "/world");

    // Get parameter values
    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_topic_, rclcpp::SensorDataQoS(), std::bind(&CloudToWorld::callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
}

void CloudToWorld::callback(const sensor_msgs::msg::PointCloud2 msg)
{
    RCLCPP_DEBUG(LOGGER, "Cloud get");
    try
    {
        geometry_msgs::msg::TransformStamped transform =
        tf_buffer_.lookupTransform(
            target_frame_,                      // target
            msg.header.frame_id,         // source
            rclcpp::Time(0));
            //msg.header.stamp);           // timestamp

        sensor_msgs::msg::PointCloud2 cloud_out;
        tf2::doTransform(msg, cloud_out, transform);

        cloud_out.header.frame_id = target_frame_;
        pub_->publish(cloud_out);
        RCLCPP_DEBUG(LOGGER, "Publish");
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "TF failed: %s", ex.what());
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<CloudToWorld>();
    
    RCLCPP_INFO(LOGGER, "Transform Node ON");
    RCLCPP_INFO(LOGGER, "Cloud frame --> %s frame", node->getTargetFrame().c_str());

    rclcpp::spin(node);

    RCLCPP_INFO(LOGGER, "Stop Transform Node");
    rclcpp::shutdown();
    return 0;
}