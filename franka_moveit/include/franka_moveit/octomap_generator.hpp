#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <moveit_msgs/msg/planning_scene.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <moveit_msgs/msg/planning_scene.hpp>

// Voxel occupancy and last time of apparition
struct VoxelInfo
{
    float occupancy;        // probability or log odds
    rclcpp::Time last_seen; // last observation time
};

// Voxel 3d position
struct VoxelKey
{
    int x;
    int y;
    int z;

    bool operator==(const VoxelKey &other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelHash
{
    std::size_t operator()(const VoxelKey &k) const
    {
        // Simple but effective hash combination
        return std::hash<int>()(k.x) ^
               (std::hash<int>()(k.y) << 1) ^
               (std::hash<int>()(k.z) << 2);
    }
};

/**
 * From a point cloud, generate a voxel map to construct an octomap
 */
class OctomapGenerator : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;

    // TimerBase to update the octomap
    rclcpp::TimerBase::SharedPtr timer_;

    // Function to update the octomap
    void updateOctomap();
    
    void cloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg);
    // Time based voxel removal
    void decayStep();
    // Subscription instant removal
    void removeOld();

    double decay_time_ = 2.0;
    double decay_rate_ = 0.3;
    const float resolution_ = 0.02f;
    rclcpp::Time last_time_;

    std::unordered_map<VoxelKey, VoxelInfo, VoxelHash> map_;

    // Function to create a voxelKey from a pcl point
    VoxelKey voxelKey(const pcl::PointXYZ &pt)
    {
        return VoxelKey{
            static_cast<int>(std::floor(pt.x / resolution_)),
            static_cast<int>(std::floor(pt.y / resolution_)),
            static_cast<int>(std::floor(pt.z / resolution_))};
    }

    // Build an OcTree from the voxel map
    std::shared_ptr<octomap::OcTree> buildOctree();
    // Convert a octomap to a message
    octomap_msgs::msg::Octomap toMsg(std::shared_ptr<octomap::OcTree> tree);
    void publishOctomap(std::shared_ptr<octomap::OcTree> tree);
    
public:
    OctomapGenerator(/* args */);
    ~OctomapGenerator() = default;
};