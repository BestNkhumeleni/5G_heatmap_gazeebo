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
#include <memory>

#include "heatmap_plugin/PropagationModels.hh"

namespace heatmap_plugin
{

/// Represents a building/obstacle that blocks RF signals
struct Obstacle
{
    std::string name;
    ignition::math::Vector3d position;
    ignition::math::Vector3d size;
    ignition::math::AxisAlignedBox bbox;
    MaterialProperties material;
};

/// A Gazebo system plugin that computes a 2D RF signal-strength heatmap
/// with multiple propagation model support.
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

    /// Publish the current heatmap buffer as an Image message
    void PublishHeatmap();

    /// Update obstacle list from the ECM
    void UpdateObstacles(ignition::gazebo::EntityComponentManager &_ecm);

    /// Handle model change requests
    void OnModelChangeRequest(const ignition::msgs::StringMsg &_msg);

    /// Handle config update requests
    void OnConfigUpdate(const ignition::msgs::StringMsg &_msg);

    /// Publish current configuration status
    void PublishStatus();

    // Transport
    ignition::transport::Node node;
    ignition::transport::Node::Publisher heatmapPub;
    ignition::transport::Node::Publisher statusPub;

    // Grid parameters
    int gridW{128};
    int gridH{128};
    double cellSize{0.5};

    // gNB position
    ignition::math::Vector3d gnbPos{0, 0, 10.0};

    // Propagation configuration
    PropagationConfig propConfig;
    std::unique_ptr<IPropagationModel> propModel;
    mutable std::mutex modelMutex;

    // Heatmap buffer (dBm values)
    std::vector<float> pixels;
    mutable std::mutex bufMutex;

    // Obstacles
    std::vector<Obstacle> obstacles;
    mutable std::mutex obstacleMutex;

    // Worker thread control
    std::thread worker;
    std::atomic<bool> running{false};
    std::atomic<bool> needsRecalculation{true};
    
    // Status publishing
    uint64_t lastStatusPublish{0};
};

}  // namespace heatmap_plugin