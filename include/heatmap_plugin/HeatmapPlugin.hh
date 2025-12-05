#pragma once

#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/AxisAlignedBox.hh>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <string>

namespace heatmap_plugin
{

/// Represents a building/obstacle that blocks RF signals
struct Obstacle
{
    std::string name;
    ignition::math::Vector3d position;
    ignition::math::Vector3d size;
    ignition::math::AxisAlignedBox bbox;
};

/// A Gazebo system plugin that computes a 2D RF signal-strength heatmap
/// around a gNB position with building occlusion support.
class HeatmapPlugin
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate
{
public:
    HeatmapPlugin() = default;
    ~HeatmapPlugin() override;

    void Configure(
        const ignition::gazebo::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        ignition::gazebo::EntityComponentManager &_ecm,
        ignition::gazebo::EventManager &_eventMgr) override;

    void PreUpdate(
        const ignition::gazebo::UpdateInfo &_info,
        ignition::gazebo::EntityComponentManager &_ecm) override;

private:
    /// Background thread function that computes the heatmap
    void WorkerLoop();

    /// Compute Free-Space Path Loss in dB
    static double FSPL_dB(double d, double f);

    /// Publish the current heatmap buffer as an Image message
    void PublishHeatmap();

    /// Update obstacle list from the ECM
    void UpdateObstacles(ignition::gazebo::EntityComponentManager &_ecm);

    /// Check if a ray intersects an axis-aligned bounding box
    static bool RayIntersectsAABB(
        const ignition::math::Vector3d &rayOrigin,
        const ignition::math::Vector3d &rayDir,
        double rayLength,
        const ignition::math::AxisAlignedBox &box);

    /// Count how many obstacles a ray passes through
    int CountOcclusions(
        const ignition::math::Vector3d &from,
        const ignition::math::Vector3d &to) const;

    /// Check if a point is inside any obstacle
    bool IsInsideObstacle(const ignition::math::Vector3d &point) const;

    // Transport
    ignition::transport::Node node;
    ignition::transport::Node::Publisher heatmapPub;

    // Grid parameters
    int gridW{128};
    int gridH{128};
    double cellSize{0.5};  // meters per cell

    // RF parameters
    double freqHz{3.5e9};
    double ptxDbm{30.0};
    double wallLossDb{15.0};  // Loss per wall penetration

    // gNB position
    ignition::math::Vector3d gnbPos{0, 0, 1.5};

    // Heatmap buffer (dBm values)
    std::vector<float> pixels;
    mutable std::mutex bufMutex;

    // Obstacles
    std::vector<Obstacle> obstacles;
    mutable std::mutex obstacleMutex;
    std::atomic<bool> obstaclesUpdated{false};

    // Worker thread control
    std::thread worker;
    std::atomic<bool> running{false};
    
    // Force recalculation when obstacles change
    std::atomic<bool> needsRecalculation{true};
};

}  // namespace heatmap_plugin