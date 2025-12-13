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
#include <map>

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

/// Configuration for a single gNB
struct GnbConfig
{
    int id{0};
    std::string name{"gNB_0"};
    ignition::math::Vector3d position{0, 0, 10};
    double txPowerDbm{30.0};
    double txGainDbi{8.0};
    double frequencyHz{3.5e9};
    bool enabled{true};
    
    // Optional per-gNB overrides (if not set, use global config)
    std::optional<double> rxGainDbi;
    std::optional<double> rxHeightM;
};

/// Signal combination mode for multiple gNBs
enum class SignalCombineMode
{
    BEST_SERVER,      // Maximum signal (default for coverage)
    SUM_POWER,        // Linear power sum (for interference)
    INTERFERENCE      // Show SINR (signal vs interference)
};

/// View state for pan and zoom
struct ViewState
{
    double centerX{0.0};
    double centerY{0.0};
    double zoomLevel{1.0};
    double minZoom{0.25};
    double maxZoom{8.0};
    double baseWorldWidth{64.0};
    double baseWorldHeight{64.0};
    
    double GetVisibleWidth() const { return baseWorldWidth / zoomLevel; }
    double GetVisibleHeight() const { return baseWorldHeight / zoomLevel; }
    double GetMinX() const { return centerX - GetVisibleWidth() / 2.0; }
    double GetMaxX() const { return centerX + GetVisibleWidth() / 2.0; }
    double GetMinY() const { return centerY - GetVisibleHeight() / 2.0; }
    double GetMaxY() const { return centerY + GetVisibleHeight() / 2.0; }
};

/// Result of signal calculation at a point
struct SignalResult
{
    double bestSignalDbm{-150.0};
    int bestGnbId{-1};
    double totalPowerDbm{-150.0};
    double sinrDb{0.0};
    std::vector<std::pair<int, double>> allSignals;  // gNB id -> signal dBm
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
    
    // Model/config handlers
    void OnModelChangeRequest(const ignition::msgs::StringMsg &_msg);
    void OnConfigUpdate(const ignition::msgs::StringMsg &_msg);
    void OnCombineModeChange(const ignition::msgs::StringMsg &_msg);
    
    // Single gNB handlers (legacy compatibility)
    void OnGnbPoseChange(const ignition::msgs::Pose &_msg);
    
    // Multi-gNB handlers
    void OnAddGnb(const ignition::msgs::Pose &_msg);
    void OnRemoveGnb(const ignition::msgs::Int32 &_msg);
    void OnUpdateGnb(const ignition::msgs::StringMsg &_msg);
    void OnMoveGnb(const ignition::msgs::Pose &_msg);
    void OnEnableGnb(const ignition::msgs::Int32 &_msg);
    void OnDisableGnb(const ignition::msgs::Int32 &_msg);
    void OnListGnbs(const ignition::msgs::Empty &_msg);
    
    // Interactive handlers
    void OnMouseClick(const ignition::msgs::Vector3d &_msg);
    void QuerySignalAtPosition(const ignition::math::Vector3d &_pos);
    
    // View control handlers
    void OnZoom(const ignition::msgs::Double &_msg);
    void OnPan(const ignition::msgs::Vector2d &_msg);
    void OnSetView(const ignition::msgs::Pose &_msg);
    void OnResetView(const ignition::msgs::Empty &_msg);
    void OnCenterOnGnb(const ignition::msgs::Int32 &_msg);
    
    // Publishing
    void PublishStatus();
    void PublishViewInfo();
    void PublishGnbList();
    
    // Signal calculation
    SignalResult CalculateSignalAtPoint(
        const ignition::math::Vector3d &rxPos,
        const std::vector<GnbConfig> &gnbs,
        const std::vector<Obstacle> &obstacles,
        const PropagationConfig &config,
        IPropagationModel *model);
    
    // Helper to get next available gNB ID
    int GetNextGnbId();

    // Transport
    ignition::transport::Node node;
    ignition::transport::Node::Publisher heatmapPub;
    ignition::transport::Node::Publisher statusPub;
    ignition::transport::Node::Publisher clickInfoPub;
    ignition::transport::Node::Publisher queryResultPub;
    ignition::transport::Node::Publisher viewInfoPub;
    ignition::transport::Node::Publisher gnbListPub;
    ignition::transport::Node::Publisher bestServerPub;  // Publishes dominant gNB map

    // Grid parameters
    int gridW{256};
    int gridH{256};
    double cellSize{0.5};

    // Multi-gNB support
    std::vector<GnbConfig> gnbConfigs;
    mutable std::mutex gnbMutex;
    int nextGnbId{0};
    
    // Signal combination mode
    SignalCombineMode combineMode{SignalCombineMode::BEST_SERVER};

    // View state
    ViewState viewState;
    mutable std::mutex viewMutex;

    // Propagation configuration (global defaults)
    PropagationConfig propConfig;
    std::unique_ptr<IPropagationModel> propModel;
    mutable std::mutex modelMutex;

    // Heatmap buffers
    std::vector<float> pixels;           // Signal strength (dBm)
    std::vector<int> bestServerMap;      // Best serving gNB ID per pixel
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
    
    // Color palette for gNB identification (up to 8 distinct colors)
    static constexpr int MAX_GNB_COLORS = 8;
};

}  // namespace heatmap_plugin