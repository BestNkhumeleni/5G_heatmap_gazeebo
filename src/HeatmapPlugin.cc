#include "heatmap_plugin/HeatmapPlugin.hh"

#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Geometry.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Static.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/msgs/image.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/msgs/vector3d.pb.h>
#include <ignition/msgs/vector2d.pb.h>
#include <ignition/msgs/double.pb.h>
#include <ignition/msgs/int32.pb.h>
#include <ignition/msgs/empty.pb.h>
#include <ignition/msgs/pose.pb.h>
#include <ignition/common/Console.hh>
#include <sdf/Geometry.hh>
#include <sdf/Box.hh>
#include <sdf/Cylinder.hh>
#include <cmath>
#include <algorithm>
#include <sstream>

using namespace heatmap_plugin;
using namespace ignition;
using namespace ignition::gazebo;

HeatmapPlugin::~HeatmapPlugin()
{
    running = false;
    if (worker.joinable())
        worker.join();
}

void HeatmapPlugin::Configure(
    const Entity &/*_entity*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
    auto sdf = const_cast<sdf::Element*>(_sdf.get());
    
    // Grid parameters
    if (sdf->HasElement("grid_w"))
        gridW = sdf->Get<int>("grid_w");
    if (sdf->HasElement("grid_h"))
        gridH = sdf->Get<int>("grid_h");
    if (sdf->HasElement("cell_size"))
        cellSize = sdf->Get<double>("cell_size");
    
    viewState.baseWorldWidth = gridW * cellSize;
    viewState.baseWorldHeight = gridH * cellSize;
    
    // View parameters
    if (sdf->HasElement("view_center_x"))
        viewState.centerX = sdf->Get<double>("view_center_x");
    if (sdf->HasElement("view_center_y"))
        viewState.centerY = sdf->Get<double>("view_center_y");
    if (sdf->HasElement("initial_zoom"))
        viewState.zoomLevel = sdf->Get<double>("initial_zoom");
    if (sdf->HasElement("min_zoom"))
        viewState.minZoom = sdf->Get<double>("min_zoom");
    if (sdf->HasElement("max_zoom"))
        viewState.maxZoom = sdf->Get<double>("max_zoom");
    if (sdf->HasElement("world_width"))
        viewState.baseWorldWidth = sdf->Get<double>("world_width");
    if (sdf->HasElement("world_height"))
        viewState.baseWorldHeight = sdf->Get<double>("world_height");
    
    // Global RF parameters (defaults for all gNBs)
    if (sdf->HasElement("frequency_hz"))
        propConfig.frequencyHz = sdf->Get<double>("frequency_hz");
    if (sdf->HasElement("tx_dbm"))
        propConfig.txPowerDbm = sdf->Get<double>("tx_dbm");
    if (sdf->HasElement("tx_height"))
        propConfig.txHeightM = sdf->Get<double>("tx_height");
    if (sdf->HasElement("rx_height"))
        propConfig.rxHeightM = sdf->Get<double>("rx_height");
    if (sdf->HasElement("tx_gain_dbi"))
        propConfig.txAntennaGainDbi = sdf->Get<double>("tx_gain_dbi");
    if (sdf->HasElement("rx_gain_dbi"))
        propConfig.rxAntennaGainDbi = sdf->Get<double>("rx_gain_dbi");
    
    // Material properties
    if (sdf->HasElement("wall_loss_db"))
        propConfig.defaultMaterial.penetrationLoss_dB = sdf->Get<double>("wall_loss_db");
    if (sdf->HasElement("reflection_coeff"))
        propConfig.defaultMaterial.reflectionCoeff = sdf->Get<double>("reflection_coeff");
    
    // Propagation model selection
    std::string modelName = "3gpp_umi";
    if (sdf->HasElement("propagation_model"))
        modelName = sdf->Get<std::string>("propagation_model");
    
    if (modelName == "free_space" || modelName == "fspl")
        propConfig.model = PropagationModel::FREE_SPACE;
    else if (modelName == "3gpp_umi" || modelName == "umi")
        propConfig.model = PropagationModel::THREE_GPP_UMI;
    else if (modelName == "3gpp_uma" || modelName == "uma")
        propConfig.model = PropagationModel::THREE_GPP_UMA;
    else if (modelName == "ray_tracing" || modelName == "raytracing")
        propConfig.model = PropagationModel::RAY_TRACING;
    else if (modelName == "hybrid")
        propConfig.model = PropagationModel::HYBRID;
    
    // Shadowing
    if (sdf->HasElement("include_shadowing"))
        propConfig.includeShadowing = sdf->Get<bool>("include_shadowing");
    if (sdf->HasElement("shadowing_std_los"))
        propConfig.shadowingStdLos_dB = sdf->Get<double>("shadowing_std_los");
    if (sdf->HasElement("shadowing_std_nlos"))
        propConfig.shadowingStdNlos_dB = sdf->Get<double>("shadowing_std_nlos");
    
    // Ray tracing specific
    if (sdf->HasElement("max_reflections"))
        propConfig.maxReflections = sdf->Get<int>("max_reflections");
    if (sdf->HasElement("max_diffractions"))
        propConfig.maxDiffractions = sdf->Get<int>("max_diffractions");
    
    // Signal combination mode
    if (sdf->HasElement("combine_mode"))
    {
        std::string mode = sdf->Get<std::string>("combine_mode");
        if (mode == "best_server" || mode == "best")
            combineMode = SignalCombineMode::BEST_SERVER;
        else if (mode == "sum_power" || mode == "sum")
            combineMode = SignalCombineMode::SUM_POWER;
        else if (mode == "interference" || mode == "sinr")
            combineMode = SignalCombineMode::INTERFERENCE;
    }
    
    // Interactive features
    if (sdf->HasElement("enable_click_query"))
        enableClickQuery = sdf->Get<bool>("enable_click_query");

    // Parse gNB configurations
    // Legacy single gNB support
    if (sdf->HasElement("gnb_pose"))
    {
        GnbConfig gnb;
        gnb.id = GetNextGnbId();
        gnb.name = "gNB_" + std::to_string(gnb.id);
        auto pose = sdf->Get<math::Pose3d>("gnb_pose");
        gnb.position = pose.Pos();
        gnb.txPowerDbm = propConfig.txPowerDbm;
        gnb.txGainDbi = propConfig.txAntennaGainDbi;
        gnb.frequencyHz = propConfig.frequencyHz;
        gnbConfigs.push_back(gnb);
    }
    
    // Multi-gNB configuration
    auto gnbElem = sdf->GetElement("gnb");
    while (gnbElem)
    {
        GnbConfig gnb;
        gnb.id = GetNextGnbId();
        
        if (gnbElem->HasAttribute("name"))
            gnb.name = gnbElem->Get<std::string>("name");
        else
            gnb.name = "gNB_" + std::to_string(gnb.id);
        
        if (gnbElem->HasElement("pose"))
        {
            auto pose = gnbElem->Get<math::Pose3d>("pose");
            gnb.position = pose.Pos();
        }
        
        // Per-gNB overrides
        if (gnbElem->HasElement("tx_power"))
            gnb.txPowerDbm = gnbElem->Get<double>("tx_power");
        else
            gnb.txPowerDbm = propConfig.txPowerDbm;
            
        if (gnbElem->HasElement("tx_gain"))
            gnb.txGainDbi = gnbElem->Get<double>("tx_gain");
        else
            gnb.txGainDbi = propConfig.txAntennaGainDbi;
            
        if (gnbElem->HasElement("frequency"))
            gnb.frequencyHz = gnbElem->Get<double>("frequency");
        else
            gnb.frequencyHz = propConfig.frequencyHz;
        
        if (gnbElem->HasElement("enabled"))
            gnb.enabled = gnbElem->Get<bool>("enabled");
        
        gnbConfigs.push_back(gnb);
        gnbElem = gnbElem->GetNextElement("gnb");
    }
    
    // If no gNBs configured, add a default one
    if (gnbConfigs.empty())
    {
        GnbConfig defaultGnb;
        defaultGnb.id = GetNextGnbId();
        defaultGnb.name = "gNB_0";
        defaultGnb.position = math::Vector3d(0, 0, propConfig.txHeightM);
        defaultGnb.txPowerDbm = propConfig.txPowerDbm;
        defaultGnb.txGainDbi = propConfig.txAntennaGainDbi;
        defaultGnb.frequencyHz = propConfig.frequencyHz;
        gnbConfigs.push_back(defaultGnb);
    }

    // Initialize propagation model
    propModel = PropagationModelFactory::Create(propConfig.model);

    // Initialize pixel buffers
    pixels.assign(gridW * gridH, -200.0f);
    bestServerMap.assign(gridW * gridH, -1);

    // Setup publishers
    heatmapPub = node.Advertise<msgs::Image>("/gnb/heatmap");
    statusPub = node.Advertise<msgs::StringMsg>("/gnb/heatmap/status");
    clickInfoPub = node.Advertise<msgs::StringMsg>("/gnb/heatmap/click_info");
    queryResultPub = node.Advertise<msgs::StringMsg>("/gnb/heatmap/query_result");
    viewInfoPub = node.Advertise<msgs::StringMsg>("/gnb/heatmap/view_info");
    gnbListPub = node.Advertise<msgs::StringMsg>("/gnb/heatmap/gnb_list");
    bestServerPub = node.Advertise<msgs::Image>("/gnb/heatmap/best_server");

    // Setup subscribers - model/config
    node.Subscribe("/gnb/heatmap/set_model", &HeatmapPlugin::OnModelChangeRequest, this);
    node.Subscribe("/gnb/heatmap/config", &HeatmapPlugin::OnConfigUpdate, this);
    node.Subscribe("/gnb/heatmap/set_combine_mode", &HeatmapPlugin::OnCombineModeChange, this);
    
    // Legacy single gNB (moves gNB 0)
    node.Subscribe("/gnb/heatmap/set_position", &HeatmapPlugin::OnGnbPoseChange, this);
    node.Subscribe("/gnb/heatmap/query_position", &HeatmapPlugin::OnMouseClick, this);
    
    // Multi-gNB management
    node.Subscribe("/gnb/heatmap/add_gnb", &HeatmapPlugin::OnAddGnb, this);
    node.Subscribe("/gnb/heatmap/remove_gnb", &HeatmapPlugin::OnRemoveGnb, this);
    node.Subscribe("/gnb/heatmap/update_gnb", &HeatmapPlugin::OnUpdateGnb, this);
    node.Subscribe("/gnb/heatmap/move_gnb", &HeatmapPlugin::OnMoveGnb, this);
    node.Subscribe("/gnb/heatmap/enable_gnb", &HeatmapPlugin::OnEnableGnb, this);
    node.Subscribe("/gnb/heatmap/disable_gnb", &HeatmapPlugin::OnDisableGnb, this);
    node.Subscribe("/gnb/heatmap/list_gnbs", &HeatmapPlugin::OnListGnbs, this);
    
    // View control subscribers
    node.Subscribe("/gnb/heatmap/zoom", &HeatmapPlugin::OnZoom, this);
    node.Subscribe("/gnb/heatmap/pan", &HeatmapPlugin::OnPan, this);
    node.Subscribe("/gnb/heatmap/set_view", &HeatmapPlugin::OnSetView, this);
    node.Subscribe("/gnb/heatmap/reset_view", &HeatmapPlugin::OnResetView, this);
    node.Subscribe("/gnb/heatmap/center_on_gnb", &HeatmapPlugin::OnCenterOnGnb, this);

    ignmsg << "=== HeatmapPlugin Configuration (Multi-gNB) ===" << std::endl;
    ignmsg << "  Output Resolution: " << gridW << "x" << gridH << std::endl;
    ignmsg << "  Base World Size: " << viewState.baseWorldWidth << "x" 
           << viewState.baseWorldHeight << " m" << std::endl;
    ignmsg << "  Propagation Model: " << propModel->GetName() << std::endl;
    ignmsg << "  Signal Combine Mode: " << 
        (combineMode == SignalCombineMode::BEST_SERVER ? "Best Server" :
         combineMode == SignalCombineMode::SUM_POWER ? "Sum Power" : "SINR") << std::endl;
    ignmsg << "  Number of gNBs: " << gnbConfigs.size() << std::endl;
    for (const auto& gnb : gnbConfigs)
    {
        ignmsg << "    [" << gnb.id << "] " << gnb.name << " at (" 
               << gnb.position.X() << ", " << gnb.position.Y() << ", " 
               << gnb.position.Z() << ") " << gnb.txPowerDbm << " dBm"
               << (gnb.enabled ? "" : " [disabled]") << std::endl;
    }
    ignmsg << "===============================" << std::endl;
    ignmsg << "Multi-gNB control topics:" << std::endl;
    ignmsg << "  /gnb/heatmap/add_gnb - Add gNB (Pose: x,y,z position)" << std::endl;
    ignmsg << "  /gnb/heatmap/remove_gnb - Remove gNB by ID (Int32)" << std::endl;
    ignmsg << "  /gnb/heatmap/move_gnb - Move gNB (Pose with name=ID)" << std::endl;
    ignmsg << "  /gnb/heatmap/update_gnb - Update gNB params (StringMsg)" << std::endl;
    ignmsg << "  /gnb/heatmap/enable_gnb - Enable gNB by ID" << std::endl;
    ignmsg << "  /gnb/heatmap/disable_gnb - Disable gNB by ID" << std::endl;
    ignmsg << "  /gnb/heatmap/list_gnbs - List all gNBs" << std::endl;
    ignmsg << "  /gnb/heatmap/set_combine_mode - best_server/sum_power/sinr" << std::endl;

    UpdateObstacles(_ecm);

    running = true;
    worker = std::thread(&HeatmapPlugin::WorkerLoop, this);
}

int HeatmapPlugin::GetNextGnbId()
{
    return nextGnbId++;
}

// ============================================================================
// Multi-gNB Management
// ============================================================================

void HeatmapPlugin::OnAddGnb(const msgs::Pose &_msg)
{
    GnbConfig gnb;
    {
        std::lock_guard<std::mutex> lock(gnbMutex);
        gnb.id = GetNextGnbId();
    }
    
    gnb.name = _msg.name().empty() ? "gNB_" + std::to_string(gnb.id) : _msg.name();
    gnb.position = math::Vector3d(_msg.position().x(), _msg.position().y(), _msg.position().z());
    
    // Use defaults from propConfig
    gnb.txPowerDbm = propConfig.txPowerDbm;
    gnb.txGainDbi = propConfig.txAntennaGainDbi;
    gnb.frequencyHz = propConfig.frequencyHz;
    gnb.enabled = true;
    
    {
        std::lock_guard<std::mutex> lock(gnbMutex);
        gnbConfigs.push_back(gnb);
    }
    
    needsRecalculation = true;
    ignmsg << "Added gNB [" << gnb.id << "] '" << gnb.name << "' at (" 
           << gnb.position.X() << ", " << gnb.position.Y() << ", " 
           << gnb.position.Z() << ")" << std::endl;
    
    PublishGnbList();
}

void HeatmapPlugin::OnRemoveGnb(const msgs::Int32 &_msg)
{
    int idToRemove = _msg.data();
    
    std::lock_guard<std::mutex> lock(gnbMutex);
    
    auto it = std::find_if(gnbConfigs.begin(), gnbConfigs.end(),
        [idToRemove](const GnbConfig& g) { return g.id == idToRemove; });
    
    if (it != gnbConfigs.end())
    {
        std::string name = it->name;
        gnbConfigs.erase(it);
        needsRecalculation = true;
        ignmsg << "Removed gNB [" << idToRemove << "] '" << name << "'" << std::endl;
        PublishGnbList();
    }
    else
    {
        ignwarn << "gNB with ID " << idToRemove << " not found" << std::endl;
    }
}

void HeatmapPlugin::OnMoveGnb(const msgs::Pose &_msg)
{
    // Use pose.name as string ID, or position.w as numeric ID
    int targetId = -1;
    
    if (!_msg.name().empty())
    {
        try { targetId = std::stoi(_msg.name()); }
        catch (...) { targetId = 0; }  // Default to first gNB
    }
    else
    {
        targetId = static_cast<int>(_msg.orientation().w());
    }
    
    std::lock_guard<std::mutex> lock(gnbMutex);
    
    for (auto& gnb : gnbConfigs)
    {
        if (gnb.id == targetId)
        {
            gnb.position = math::Vector3d(
                _msg.position().x(), _msg.position().y(), _msg.position().z());
            needsRecalculation = true;
            ignmsg << "Moved gNB [" << gnb.id << "] to (" << gnb.position.X() 
                   << ", " << gnb.position.Y() << ", " << gnb.position.Z() << ")" << std::endl;
            return;
        }
    }
    
    ignwarn << "gNB with ID " << targetId << " not found for move" << std::endl;
}

void HeatmapPlugin::OnUpdateGnb(const msgs::StringMsg &_msg)
{
    // Format: "id=0;tx_power=35;tx_gain=10;frequency=3.5e9;name=NewName"
    std::istringstream iss(_msg.data());
    std::string token;
    
    int targetId = -1;
    std::map<std::string, std::string> params;
    
    while (std::getline(iss, token, ';'))
    {
        auto pos = token.find('=');
        if (pos == std::string::npos) continue;
        
        std::string key = token.substr(0, pos);
        std::string value = token.substr(pos + 1);
        
        if (key == "id")
            targetId = std::stoi(value);
        else
            params[key] = value;
    }
    
    if (targetId < 0)
    {
        ignwarn << "No gNB ID specified in update" << std::endl;
        return;
    }
    
    std::lock_guard<std::mutex> lock(gnbMutex);
    
    for (auto& gnb : gnbConfigs)
    {
        if (gnb.id == targetId)
        {
            for (const auto& [key, value] : params)
            {
                try
                {
                    if (key == "tx_power")
                        gnb.txPowerDbm = std::stod(value);
                    else if (key == "tx_gain")
                        gnb.txGainDbi = std::stod(value);
                    else if (key == "frequency")
                        gnb.frequencyHz = std::stod(value);
                    else if (key == "name")
                        gnb.name = value;
                    else if (key == "enabled")
                        gnb.enabled = (value == "true" || value == "1");
                    
                    ignmsg << "Updated gNB [" << targetId << "] " << key << " = " << value << std::endl;
                }
                catch (const std::exception& e)
                {
                    ignwarn << "Failed to parse " << key << ": " << e.what() << std::endl;
                }
            }
            needsRecalculation = true;
            return;
        }
    }
    
    ignwarn << "gNB with ID " << targetId << " not found" << std::endl;
}

void HeatmapPlugin::OnEnableGnb(const msgs::Int32 &_msg)
{
    int targetId = _msg.data();
    std::lock_guard<std::mutex> lock(gnbMutex);
    
    for (auto& gnb : gnbConfigs)
    {
        if (gnb.id == targetId)
        {
            gnb.enabled = true;
            needsRecalculation = true;
            ignmsg << "Enabled gNB [" << targetId << "]" << std::endl;
            return;
        }
    }
    ignwarn << "gNB with ID " << targetId << " not found" << std::endl;
}

void HeatmapPlugin::OnDisableGnb(const msgs::Int32 &_msg)
{
    int targetId = _msg.data();
    std::lock_guard<std::mutex> lock(gnbMutex);
    
    for (auto& gnb : gnbConfigs)
    {
        if (gnb.id == targetId)
        {
            gnb.enabled = false;
            needsRecalculation = true;
            ignmsg << "Disabled gNB [" << targetId << "]" << std::endl;
            return;
        }
    }
    ignwarn << "gNB with ID " << targetId << " not found" << std::endl;
}

void HeatmapPlugin::OnListGnbs(const msgs::Empty &/*_msg*/)
{
    PublishGnbList();
}

void HeatmapPlugin::PublishGnbList()
{
    std::ostringstream oss;
    
    std::lock_guard<std::mutex> lock(gnbMutex);
    
    oss << "gnb_count=" << gnbConfigs.size() << ";";
    
    for (const auto& gnb : gnbConfigs)
    {
        oss << "gnb[" << gnb.id << "]={name=" << gnb.name
            << ",x=" << gnb.position.X()
            << ",y=" << gnb.position.Y()
            << ",z=" << gnb.position.Z()
            << ",power=" << gnb.txPowerDbm
            << ",gain=" << gnb.txGainDbi
            << ",freq=" << gnb.frequencyHz
            << ",enabled=" << (gnb.enabled ? "true" : "false")
            << "};";
    }
    
    msgs::StringMsg msg;
    msg.set_data(oss.str());
    gnbListPub.Publish(msg);
}

void HeatmapPlugin::OnCombineModeChange(const msgs::StringMsg &_msg)
{
    std::string mode = _msg.data();
    
    if (mode == "best_server" || mode == "best")
        combineMode = SignalCombineMode::BEST_SERVER;
    else if (mode == "sum_power" || mode == "sum")
        combineMode = SignalCombineMode::SUM_POWER;
    else if (mode == "interference" || mode == "sinr")
        combineMode = SignalCombineMode::INTERFERENCE;
    else
    {
        ignwarn << "Unknown combine mode: " << mode << std::endl;
        return;
    }
    
    needsRecalculation = true;
    ignmsg << "Signal combine mode changed to: " << mode << std::endl;
}

// ============================================================================
// Legacy Single-gNB Handlers (operate on gNB 0)
// ============================================================================

void HeatmapPlugin::OnGnbPoseChange(const msgs::Pose &_msg)
{
    math::Vector3d newPos(_msg.position().x(), _msg.position().y(), _msg.position().z());
    
    std::lock_guard<std::mutex> lock(gnbMutex);
    
    if (!gnbConfigs.empty())
    {
        gnbConfigs[0].position = newPos;
        needsRecalculation = true;
        ignmsg << "gNB [0] position updated to: (" << newPos.X() << ", " 
               << newPos.Y() << ", " << newPos.Z() << ")" << std::endl;
    }
}

// ============================================================================
// Signal Calculation
// ============================================================================

SignalResult HeatmapPlugin::CalculateSignalAtPoint(
    const ignition::math::Vector3d &rxPos,
    const std::vector<GnbConfig> &gnbs,
    const std::vector<Obstacle> &obstacles,
    const PropagationConfig &config,
    IPropagationModel *model)
{
    SignalResult result;
    result.bestSignalDbm = -200.0;
    result.bestGnbId = -1;
    
    double totalLinearPower = 0.0;
    double noiseFloor = -100.0;  // dBm
    
    for (const auto& gnb : gnbs)
    {
        if (!gnb.enabled) continue;
        
        // Create per-gNB config
        PropagationConfig gnbConfig = config;
        gnbConfig.txPowerDbm = gnb.txPowerDbm;
        gnbConfig.txAntennaGainDbi = gnb.txGainDbi;
        gnbConfig.frequencyHz = gnb.frequencyHz;
        gnbConfig.txHeightM = gnb.position.Z();
        
        double signalDbm = model->CalculateReceivedPower(
            gnb.position, rxPos, obstacles, gnbConfig);
        
        result.allSignals.push_back({gnb.id, signalDbm});
        
        // Track best server
        if (signalDbm > result.bestSignalDbm)
        {
            result.bestSignalDbm = signalDbm;
            result.bestGnbId = gnb.id;
        }
        
        // Accumulate linear power
        totalLinearPower += std::pow(10.0, signalDbm / 10.0);
    }
    
    // Total power (sum)
    if (totalLinearPower > 0)
        result.totalPowerDbm = 10.0 * std::log10(totalLinearPower);
    else
        result.totalPowerDbm = -200.0;
    
    // Calculate SINR (best signal vs interference + noise)
    if (result.bestGnbId >= 0)
    {
        double bestLinear = std::pow(10.0, result.bestSignalDbm / 10.0);
        double interferenceLinear = totalLinearPower - bestLinear;
        double noiseLinear = std::pow(10.0, noiseFloor / 10.0);
        
        if (interferenceLinear + noiseLinear > 0)
            result.sinrDb = 10.0 * std::log10(bestLinear / (interferenceLinear + noiseLinear));
        else
            result.sinrDb = 50.0;  // Very high SINR
    }
    
    return result;
}

// ============================================================================
// View Controls
// ============================================================================

void HeatmapPlugin::OnZoom(const msgs::Double &_msg)
{
    std::lock_guard<std::mutex> lock(viewMutex);
    double zoomDelta = _msg.data();
    double newZoom = viewState.zoomLevel * (1.0 + zoomDelta * 0.1);
    viewState.zoomLevel = std::clamp(newZoom, viewState.minZoom, viewState.maxZoom);
    needsRecalculation = true;
}

void HeatmapPlugin::OnPan(const msgs::Vector2d &_msg)
{
    std::lock_guard<std::mutex> lock(viewMutex);
    double panScale = 1.0 / viewState.zoomLevel;
    viewState.centerX += _msg.x() * panScale;
    viewState.centerY += _msg.y() * panScale;
    needsRecalculation = true;
}

void HeatmapPlugin::OnSetView(const msgs::Pose &_msg)
{
    std::lock_guard<std::mutex> lock(viewMutex);
    viewState.centerX = _msg.position().x();
    viewState.centerY = _msg.position().y();
    if (_msg.position().z() > 0)
        viewState.zoomLevel = std::clamp(_msg.position().z(), viewState.minZoom, viewState.maxZoom);
    needsRecalculation = true;
}

void HeatmapPlugin::OnResetView(const msgs::Empty &/*_msg*/)
{
    std::lock_guard<std::mutex> lock(viewMutex);
    viewState.centerX = 0.0;
    viewState.centerY = 0.0;
    viewState.zoomLevel = 1.0;
    needsRecalculation = true;
}

void HeatmapPlugin::OnCenterOnGnb(const msgs::Int32 &_msg)
{
    int targetId = _msg.data();
    
    std::lock_guard<std::mutex> lock1(gnbMutex);
    std::lock_guard<std::mutex> lock2(viewMutex);
    
    for (const auto& gnb : gnbConfigs)
    {
        if (gnb.id == targetId)
        {
            viewState.centerX = gnb.position.X();
            viewState.centerY = gnb.position.Y();
            needsRecalculation = true;
            ignmsg << "View centered on gNB [" << targetId << "]" << std::endl;
            return;
        }
    }
    
    // Default to first gNB if ID not found
    if (!gnbConfigs.empty())
    {
        viewState.centerX = gnbConfigs[0].position.X();
        viewState.centerY = gnbConfigs[0].position.Y();
        needsRecalculation = true;
    }
}

void HeatmapPlugin::PublishViewInfo()
{
    ViewState localView;
    {
        std::lock_guard<std::mutex> lock(viewMutex);
        localView = viewState;
    }
    
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "center_x=" << localView.centerX << ";"
        << "center_y=" << localView.centerY << ";"
        << "zoom=" << localView.zoomLevel << ";"
        << "visible_w=" << localView.GetVisibleWidth() << ";"
        << "visible_h=" << localView.GetVisibleHeight();
    
    msgs::StringMsg msg;
    msg.set_data(oss.str());
    viewInfoPub.Publish(msg);
}

// ============================================================================
// Model/Config Updates
// ============================================================================

void HeatmapPlugin::OnModelChangeRequest(const msgs::StringMsg &_msg)
{
    std::string modelName = _msg.data();
    PropagationModel newModel = propConfig.model;
    
    if (modelName == "free_space" || modelName == "fspl")
        newModel = PropagationModel::FREE_SPACE;
    else if (modelName == "3gpp_umi" || modelName == "umi")
        newModel = PropagationModel::THREE_GPP_UMI;
    else if (modelName == "3gpp_uma" || modelName == "uma")
        newModel = PropagationModel::THREE_GPP_UMA;
    else if (modelName == "ray_tracing" || modelName == "raytracing")
        newModel = PropagationModel::RAY_TRACING;
    else if (modelName == "hybrid")
        newModel = PropagationModel::HYBRID;
    else
    {
        ignwarn << "Unknown propagation model: " << modelName << std::endl;
        return;
    }
    
    {
        std::lock_guard<std::mutex> lock(modelMutex);
        propConfig.model = newModel;
        propModel = PropagationModelFactory::Create(newModel);
    }
    
    needsRecalculation = true;
    ignmsg << "Propagation model changed to: " << propModel->GetName() << std::endl;
}

void HeatmapPlugin::OnConfigUpdate(const msgs::StringMsg &_msg)
{
    std::istringstream iss(_msg.data());
    std::string token;
    
    std::lock_guard<std::mutex> lock(modelMutex);
    
    while (std::getline(iss, token, ';'))
    {
        auto pos = token.find('=');
        if (pos == std::string::npos) continue;
        
        std::string key = token.substr(0, pos);
        std::string value = token.substr(pos + 1);
        
        try
        {
            if (key == "tx_power")
                propConfig.txPowerDbm = std::stod(value);
            else if (key == "frequency")
                propConfig.frequencyHz = std::stod(value);
            else if (key == "tx_height")
                propConfig.txHeightM = std::stod(value);
            else if (key == "rx_height")
                propConfig.rxHeightM = std::stod(value);
            else if (key == "wall_loss")
                propConfig.defaultMaterial.penetrationLoss_dB = std::stod(value);
            else if (key == "reflection_coeff")
                propConfig.defaultMaterial.reflectionCoeff = std::stod(value);
            else if (key == "shadowing")
                propConfig.includeShadowing = (value == "true" || value == "1");
            else if (key == "max_reflections")
                propConfig.maxReflections = std::stoi(value);
            
            ignmsg << "Config updated: " << key << " = " << value << std::endl;
        }
        catch (const std::exception& e)
        {
            ignwarn << "Failed to parse config " << key << ": " << e.what() << std::endl;
        }
    }
    
    needsRecalculation = true;
}

// ============================================================================
// Interactive Query
// ============================================================================

void HeatmapPlugin::OnMouseClick(const msgs::Vector3d &_msg)
{
    if (!enableClickQuery) return;
    math::Vector3d clickPos(_msg.x(), _msg.y(), _msg.z());
    QuerySignalAtPosition(clickPos);
}

void HeatmapPlugin::QuerySignalAtPosition(const math::Vector3d &_pos)
{
    std::vector<GnbConfig> localGnbs;
    std::vector<Obstacle> localObstacles;
    PropagationConfig localConfig;
    std::unique_ptr<IPropagationModel> localModel;
    
    {
        std::lock_guard<std::mutex> lock1(gnbMutex);
        std::lock_guard<std::mutex> lock2(obstacleMutex);
        std::lock_guard<std::mutex> lock3(modelMutex);
        localGnbs = gnbConfigs;
        localObstacles = obstacles;
        localConfig = propConfig;
        localModel = PropagationModelFactory::Create(localConfig.model);
    }
    
    math::Vector3d queryPos(_pos.X(), _pos.Y(), localConfig.rxHeightM);
    
    auto result = CalculateSignalAtPoint(queryPos, localGnbs, localObstacles, 
                                         localConfig, localModel.get());
    
    std::ostringstream detailOss;
    detailOss << std::fixed << std::setprecision(2);
    detailOss << "=== Signal Query Result ===" << std::endl;
    detailOss << "Position: (" << _pos.X() << ", " << _pos.Y() << ")" << std::endl;
    detailOss << "Best Server: gNB [" << result.bestGnbId << "]" << std::endl;
    detailOss << "Best Signal: " << result.bestSignalDbm << " dBm" << std::endl;
    detailOss << "Total Power: " << result.totalPowerDbm << " dBm" << std::endl;
    detailOss << "SINR: " << result.sinrDb << " dB" << std::endl;
    detailOss << "--- Per-gNB Signals ---" << std::endl;
    
    for (const auto& [id, signal] : result.allSignals)
    {
        auto it = std::find_if(localGnbs.begin(), localGnbs.end(),
            [id](const GnbConfig& g) { return g.id == id; });
        std::string name = (it != localGnbs.end()) ? it->name : "Unknown";
        double dist = (it != localGnbs.end()) ? it->position.Distance(queryPos) : 0;
        detailOss << "  [" << id << "] " << name << ": " << signal << " dBm"
                  << " (d=" << dist << " m)" << std::endl;
    }
    
    ignmsg << detailOss.str();
    
    msgs::StringMsg clickMsg;
    std::ostringstream clickOss;
    clickOss << std::fixed << std::setprecision(1);
    clickOss << "Pos: (" << _pos.X() << ", " << _pos.Y() << ") | "
             << "Best: gNB[" << result.bestGnbId << "] " << result.bestSignalDbm << " dBm | "
             << "SINR: " << result.sinrDb << " dB";
    clickMsg.set_data(clickOss.str());
    clickInfoPub.Publish(clickMsg);
    
    msgs::StringMsg resultMsg;
    resultMsg.set_data(detailOss.str());
    queryResultPub.Publish(resultMsg);
}

void HeatmapPlugin::PublishStatus()
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    
    std::lock_guard<std::mutex> lock1(gnbMutex);
    std::lock_guard<std::mutex> lock2(modelMutex);
    
    oss << "model=" << propModel->GetName() << ";"
        << "combine_mode=" << (combineMode == SignalCombineMode::BEST_SERVER ? "best_server" :
                               combineMode == SignalCombineMode::SUM_POWER ? "sum_power" : "sinr") << ";"
        << "gnb_count=" << gnbConfigs.size() << ";"
        << "frequency=" << propConfig.frequencyHz << ";"
        << "rx_height=" << propConfig.rxHeightM << ";"
        << "wall_loss=" << propConfig.defaultMaterial.penetrationLoss_dB << ";"
        << "obstacles=" << obstacles.size() << ";"
        << "shadowing=" << (propConfig.includeShadowing ? "true" : "false");
    
    msgs::StringMsg msg;
    msg.set_data(oss.str());
    statusPub.Publish(msg);
}

// ============================================================================
// PreUpdate / Obstacle Update
// ============================================================================

void HeatmapPlugin::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
    if (_info.paused) return;

    uint64_t currentIter = _info.iterations;
    
    static uint64_t lastObstacleUpdate = 0;
    if (currentIter - lastObstacleUpdate >= 50)
    {
        UpdateObstacles(_ecm);
        lastObstacleUpdate = currentIter;
    }

    static uint64_t lastPublish = 0;
    if (currentIter - lastPublish >= 10)
    {
        PublishHeatmap();
        lastPublish = currentIter;
    }
    
    if (currentIter - lastStatusPublish >= 100)
    {
        PublishStatus();
        lastStatusPublish = currentIter;
    }
    
    if (currentIter - lastViewInfoPublish >= 50)
    {
        PublishViewInfo();
        lastViewInfoPublish = currentIter;
    }
}

void HeatmapPlugin::UpdateObstacles(EntityComponentManager &_ecm)
{
    std::vector<Obstacle> newObstacles;

    _ecm.Each<components::Model, components::Name, components::Pose>(
        [&](const Entity &modelEntity, const components::Model *,
            const components::Name *nameComp, const components::Pose *poseComp) -> bool
        {
            std::string modelName = nameComp->Data();
            std::string lowerName = modelName;
            std::transform(lowerName.begin(), lowerName.end(), lowerName.begin(), ::tolower);
            
            if (lowerName.find("ground") != std::string::npos ||
                lowerName.find("gnb") != std::string::npos ||
                lowerName.find("sun") != std::string::npos)
                return true;

            math::Pose3d modelPose = poseComp->Data();

            _ecm.Each<components::Link, components::ParentEntity, components::Pose>(
                [&](const Entity &linkEntity, const components::Link *,
                    const components::ParentEntity *parentComp,
                    const components::Pose *linkPoseComp) -> bool
                {
                    if (parentComp->Data() != modelEntity) return true;
                    math::Pose3d linkPose = linkPoseComp->Data();

                    _ecm.Each<components::Collision, components::ParentEntity, 
                             components::Geometry, components::Pose>(
                        [&](const Entity &, const components::Collision *,
                            const components::ParentEntity *collParentComp,
                            const components::Geometry *geomComp,
                            const components::Pose *collPoseComp) -> bool
                        {
                            if (collParentComp->Data() != linkEntity) return true;

                            const sdf::Geometry &geom = geomComp->Data();
                            math::Pose3d collPose = collPoseComp->Data();
                            math::Pose3d worldPose = modelPose * linkPose * collPose;

                            Obstacle obs;
                            obs.name = modelName;
                            obs.position = worldPose.Pos();
                            obs.material = propConfig.defaultMaterial;

                            bool valid = false;
                            if (geom.Type() == sdf::GeometryType::BOX && geom.BoxShape())
                            {
                                obs.size = geom.BoxShape()->Size();
                                valid = true;
                            }
                            else if (geom.Type() == sdf::GeometryType::CYLINDER && geom.CylinderShape())
                            {
                                double r = geom.CylinderShape()->Radius();
                                double h = geom.CylinderShape()->Length();
                                obs.size = math::Vector3d(r * 2, r * 2, h);
                                valid = true;
                            }

                            if (valid)
                            {
                                math::Vector3d halfSize = obs.size / 2.0;
                                obs.bbox = math::AxisAlignedBox(
                                    obs.position - halfSize, obs.position + halfSize);
                                newObstacles.push_back(obs);
                            }
                            return true;
                        });
                    return true;
                });
            return true;
        });

    bool changed = false;
    {
        std::lock_guard<std::mutex> lock(obstacleMutex);
        if (newObstacles.size() != obstacles.size())
            changed = true;
        else
        {
            for (size_t i = 0; i < newObstacles.size(); ++i)
            {
                if (newObstacles[i].position.Distance(obstacles[i].position) > 0.01)
                {
                    changed = true;
                    break;
                }
            }
        }
        
        if (changed)
        {
            obstacles = std::move(newObstacles);
            needsRecalculation = true;
        }
    }
}

// ============================================================================
// Worker Loop / Heatmap Generation
// ============================================================================

void HeatmapPlugin::WorkerLoop()
{
    while (running)
    {
        if (!needsRecalculation)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }
        
        needsRecalculation = false;

        ViewState localView;
        {
            std::lock_guard<std::mutex> lock(viewMutex);
            localView = viewState;
        }

        double minX = localView.GetMinX();
        double minY = localView.GetMinY();
        double visibleWidth = localView.GetVisibleWidth();
        double visibleHeight = localView.GetVisibleHeight();

        std::vector<float> newPixels(gridW * gridH);
        std::vector<int> newBestServer(gridW * gridH);

        std::vector<GnbConfig> localGnbs;
        std::vector<Obstacle> localObstacles;
        PropagationConfig localConfig;
        std::unique_ptr<IPropagationModel> localModel;
        SignalCombineMode localCombineMode;
        
        {
            std::lock_guard<std::mutex> lock1(gnbMutex);
            std::lock_guard<std::mutex> lock2(obstacleMutex);
            std::lock_guard<std::mutex> lock3(modelMutex);
            localGnbs = gnbConfigs;
            localObstacles = obstacles;
            localConfig = propConfig;
            localModel = PropagationModelFactory::Create(localConfig.model);
            localCombineMode = combineMode;
        }

        for (int y = 0; y < gridH && running; ++y)
        {
            for (int x = 0; x < gridW; ++x)
            {
                double nx = static_cast<double>(x) / (gridW - 1);
                double ny = static_cast<double>(y) / (gridH - 1);
                
                double worldX = minX + nx * visibleWidth;
                double worldY = minY + ny * visibleHeight;

                math::Vector3d rxPos(worldX, worldY, localConfig.rxHeightM);

                auto result = CalculateSignalAtPoint(rxPos, localGnbs, localObstacles,
                                                     localConfig, localModel.get());

                float displayValue;
                switch (localCombineMode)
                {
                    case SignalCombineMode::BEST_SERVER:
                        displayValue = static_cast<float>(result.bestSignalDbm);
                        break;
                    case SignalCombineMode::SUM_POWER:
                        displayValue = static_cast<float>(result.totalPowerDbm);
                        break;
                    case SignalCombineMode::INTERFERENCE:
                        displayValue = static_cast<float>(result.sinrDb);
                        break;
                }

                newPixels[y * gridW + x] = displayValue;
                newBestServer[y * gridW + x] = result.bestGnbId;
            }
        }

        {
            std::lock_guard<std::mutex> lock(bufMutex);
            pixels = std::move(newPixels);
            bestServerMap = std::move(newBestServer);
        }

        igndbg << "Heatmap recalculated with " << localGnbs.size() << " gNBs" << std::endl;
    }
}

void HeatmapPlugin::PublishHeatmap()
{
    msgs::Image msg;
    msg.set_width(gridW);
    msg.set_height(gridH);
    msg.set_pixel_format_type(msgs::PixelFormatType::RGB_INT8);
    msg.set_step(gridW * 3);

    std::vector<uint8_t> rgbData(gridW * gridH * 3);

    // Color ranges depend on combine mode
    float minVal, maxVal;
    {
        std::lock_guard<std::mutex> lock(modelMutex);
        if (combineMode == SignalCombineMode::INTERFERENCE)
        {
            minVal = -10.0f;  // SINR range
            maxVal = 30.0f;
        }
        else
        {
            minVal = -120.0f;  // dBm range
            maxVal = -30.0f;
        }
    }
    
    const float range = maxVal - minVal;

    {
        std::lock_guard<std::mutex> lock(bufMutex);
        
        for (int i = 0; i < gridW * gridH; ++i)
        {
            float val = pixels[i];
            
            if (val <= minVal)
            {
                rgbData[i * 3 + 0] = 30;
                rgbData[i * 3 + 1] = 30;
                rgbData[i * 3 + 2] = 50;
                continue;
            }
            
            float normalized = (val - minVal) / range;
            normalized = std::clamp(normalized, 0.0f, 1.0f);

            uint8_t r, g, b;
            
            if (normalized < 0.25f)
            {
                float t = normalized / 0.25f;
                r = 0; g = static_cast<uint8_t>(255 * t); b = 255;
            }
            else if (normalized < 0.5f)
            {
                float t = (normalized - 0.25f) / 0.25f;
                r = 0; g = 255; b = static_cast<uint8_t>(255 * (1 - t));
            }
            else if (normalized < 0.75f)
            {
                float t = (normalized - 0.5f) / 0.25f;
                r = static_cast<uint8_t>(255 * t); g = 255; b = 0;
            }
            else
            {
                float t = (normalized - 0.75f) / 0.25f;
                r = 255; g = static_cast<uint8_t>(255 * (1 - t)); b = 0;
            }

            rgbData[i * 3 + 0] = r;
            rgbData[i * 3 + 1] = g;
            rgbData[i * 3 + 2] = b;
        }
    }

    msg.set_data(rgbData.data(), rgbData.size());
    heatmapPub.Publish(msg);
}

IGNITION_ADD_PLUGIN(
    heatmap_plugin::HeatmapPlugin,
    System,
    HeatmapPlugin::ISystemConfigure,
    HeatmapPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
    heatmap_plugin::HeatmapPlugin,
    "heatmap_plugin::HeatmapPlugin")