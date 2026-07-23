#include "franka_moveit/octomap_generator.hpp"

static auto const LOGGER = rclcpp::get_logger("octomap_generator");
using namespace std::chrono_literals;

OctomapGenerator::OctomapGenerator()
    : Node("octomap_generator")
{
    planning_scene_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/octocloud",
        rclcpp::SensorDataQoS(),
        std::bind(&OctomapGenerator::cloudCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&OctomapGenerator::updateOctomap, this));

    map_.clear();
}

void OctomapGenerator::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    last_time_ = this->now();

    // Reattribute in the map the new points
    for (auto &pt : cloud.points)
    {
        auto key = voxelKey(pt);

        auto &v = map_[key];
        v.occupancy = 1.0f;
        v.last_seen = last_time_;
    }

    removeOld();
}


void OctomapGenerator::removeOld()
{
    for (auto it = map_.begin(); it != map_.end();)
    {
        // Check if last seen time is the current time 
        if (it->second.last_seen != last_time_)
        {
            // Erase in the map
            it = map_.erase(it);
            continue;
        }
        ++it;
    }
}

void OctomapGenerator::decayStep()
{
    auto now = this->now();

    for (auto it = map_.begin(); it != map_.end();)
    {
        double age = (now - it->second.last_seen).seconds();

        // Modify the occupancy for aging
        //   if occupancy <= 0.1 : remove the voxel from map
        if (age > decay_time_)
        {
            it->second.occupancy -= decay_rate_;

            if (it->second.occupancy <= 0.1)
            {
                it = map_.erase(it);
                continue;
            }
        }
        ++it;
    }
}

std::shared_ptr<octomap::OcTree> OctomapGenerator::buildOctree()
{
    auto tree = std::make_shared<octomap::OcTree>(resolution_);

    for (const auto &kv : map_)
    {
        const VoxelKey &k = kv.first;
        const VoxelInfo &v = kv.second;

        if (v.occupancy < 0.3)
            continue;

        // Convert voxel index -> world coordinate
        double x = (k.x + 0.5) * resolution_;
        double y = (k.y + 0.5) * resolution_;
        double z = (k.z + 0.5) * resolution_;

        // updateNode of the tree with the voxelKey of the map
        tree->updateNode(octomap::point3d(x, y, z), true);
    }

    tree->updateInnerOccupancy();
    return tree;
}

octomap_msgs::msg::Octomap OctomapGenerator::toMsg(std::shared_ptr<octomap::OcTree> tree)
{
    octomap_msgs::msg::Octomap msg;
    msg.header.frame_id = "base";
    msg.header.stamp = now();

    if (!octomap_msgs::fullMapToMsg(*tree, msg))
    {
        throw std::runtime_error("Failed to convert Octomap");
    }

    return msg;
}

void OctomapGenerator::publishOctomap(std::shared_ptr<octomap::OcTree> tree)
{
    // Publish to the planning scene to update octomap
    moveit_msgs::msg::PlanningScene ps;
    ps.is_diff = true;

    ps.world.octomap.header.frame_id = "base";
    ps.world.octomap.header.stamp = now();

    ps.world.octomap.origin.position.x = 0.0;
    ps.world.octomap.origin.position.y = 0.0;
    ps.world.octomap.origin.position.z = 0.0;
    ps.world.octomap.origin.orientation.w = 1.0;

    ps.world.octomap.octomap = toMsg(tree);

    planning_scene_pub_->publish(ps);
}

void OctomapGenerator::updateOctomap()
{
    decayStep();
    auto tree = buildOctree();
    publishOctomap(tree);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OctomapGenerator>();

    RCLCPP_INFO(LOGGER, "Cleaner ON ...");
    rclcpp::spin(node);

    RCLCPP_INFO(LOGGER, "Cleaner OFF ...");
    rclcpp::shutdown();
    return 0;
}