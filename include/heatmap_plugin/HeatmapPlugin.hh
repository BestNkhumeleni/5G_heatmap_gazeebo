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

struct Obstacle
{
    std::string name;
    ignition::math::Vector3d position;
    ignition::math::Vector3d size;
    ignition::math::AxisAlignedBox bbox;
    MaterialProperties material;
};

/// View state for pan and zoom
struct ViewState
{
    double centerX{0.0};      // View center X in world coordinates
    double centerY{0.0};      // View center Y in world coordinates
    double zoomLevel{1.0};    // 1.0 = default, 2.0 = 2x zoom in, 0.5 = zoom out
    double minZoom{0.25};     // Maximum zoom out (4x world size)
    double maxZoom{8.0};      // Maximum zoom in (1/8 world size)
    
    // Base world extents (at zoom=1.0)
    double baseWorldWidth{64.0};
    double baseWorldHeight{64.0};
    
    // Get current visible world extents
    double GetVisibleWidth() const { return baseWorldWidth / zoomLevel; }
    double GetVisibleHeight() const { return baseWorldHeight / zoomLevel; }
    
    // Get world bounds
    double GetMinX() const { return centerX - GetVisibleWidth() / 2.0; }
    double GetMaxX() const { return centerX + GetVisibleWidth() / 2.0; }
    double GetMinY() const { return centerY - GetVisibleHeight() / 2.0; }
    double GetMaxY() const { return centerY + GetVisibleHeight() / 2.0; }
};

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
    void WorkerLoop();
    void PublishHeatmap();
    void UpdateObstacles(ignition::gazebo::EntityComponentManager &_ecm);
    void OnModelChangeRequest(const ignition::msgs::StringMsg &_msg);
    void OnConfigUpdate(const ignition::msgs::StringMsg &_msg);
    void OnMouseClick(const ignition::msgs::Vector3d &_msg);
    void OnGnbPoseChange(const ignition::msgs::Pose &_msg);
    void QuerySignalAtPosition(const ignition::math::Vector3d &_pos);
    void PublishStatus();

    // View control handlers
    void OnZoom(const ignition::msgs::Double &_msg);
    void OnPan(const ignition::msgs::Vector2d &_msg);
    void OnSetView(const ignition::msgs::Pose &_msg);
    void OnResetView(const ignition::msgs::Empty &_msg);
    void OnCenterOnGnb(const ignition::msgs::Empty &_msg);
    void PublishViewInfo();

    // Transport
    ignition::transport::Node node;
    ignition::transport::Node::Publisher heatmapPub;
    ignition::transport::Node::Publisher statusPub;
    ignition::transport::Node::Publisher clickInfoPub;
    ignition::transport::Node::Publisher queryResultPub;
    ignition::transport::Node::Publisher viewInfoPub;

    // Grid parameters (output resolution)
    int gridW{256};
    int gridH{256};
    double cellSize{0.5};  // Only used for base world size calculation

    // gNB position
    ignition::math::Vector3d gnbPos{0, 0, 10.0};

    // View state
    ViewState viewState;
    mutable std::mutex viewMutex;

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
    uint64_t lastViewInfoPublish{0};
    
    // Interactive features
    bool enableClickQuery{true};
};

}  // namespace heatmap_plugin