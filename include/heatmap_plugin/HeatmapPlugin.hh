#pragma once

#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include <ignition/math/Vector3.hh>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

namespace heatmap_plugin
{

/// A Gazebo system plugin that computes a 2D RF signal-strength heatmap
/// around a gNB position and publishes it as an ignition.msgs.Image.
class HeatmapPlugin
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate
{
public:
    HeatmapPlugin() = default;
    ~HeatmapPlugin() override;

    // ISystemConfigure: called once when the plugin loads
    void Configure(
        const ignition::gazebo::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        ignition::gazebo::EntityComponentManager &_ecm,
        ignition::gazebo::EventManager &_eventMgr) override;

    // ISystemPreUpdate: called each simulation step before physics
    void PreUpdate(
        const ignition::gazebo::UpdateInfo &_info,
        ignition::gazebo::EntityComponentManager &_ecm) override;

private:
    /// Background thread function that computes the heatmap
    void WorkerLoop();

    /// Compute Free-Space Path Loss in dB
    /// @param d Distance in meters
    /// @param f Frequency in Hz
    /// @return FSPL in dB
    static double FSPL_dB(double d, double f);

    /// Publish the current heatmap buffer as an Image message
    void PublishHeatmap();

    /// Check line-of-sight occlusion between two points using the ECM
    /// @param from Source position
    /// @param to Target position
    /// @param ecm Entity component manager
    /// @return true if path is blocked
    bool IsOccluded(
        const ignition::math::Vector3d &from,
        const ignition::math::Vector3d &to,
        ignition::gazebo::EntityComponentManager &ecm);

    // Transport
    ignition::transport::Node node;
    ignition::transport::Node::Publisher heatmapPub;

    // Grid parameters
    int gridW{128};
    int gridH{128};
    double cellSize{0.5};  // meters per cell

    // RF parameters
    double freqHz{3.5e9};      // 3.5 GHz (typical 5G n78 band)
    double ptxDbm{30.0};       // Transmit power in dBm
    double blockLossDb{20.0};  // Additional loss when blocked

    // gNB position
    ignition::math::Vector3d gnbPos{0, 0, 1.5};

    // Heatmap buffer (dBm values)
    std::vector<float> pixels;
    std::mutex bufMutex;

    // Worker thread control
    std::thread worker;
    std::atomic<bool> running{false};
    std::atomic<bool> needsUpdate{true};

    // Pointer to ECM for worker thread (protected by mutex)
    ignition::gazebo::EntityComponentManager *ecmPtr{nullptr};
    std::mutex ecmMutex;
};

}  // namespace heatmap_plugin