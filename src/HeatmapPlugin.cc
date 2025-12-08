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
    
    // Grid parameters (output resolution)
    if (sdf->HasElement("grid_w"))
        gridW = sdf->Get<int>("grid_w");
    if (sdf->HasElement("grid_h"))
        gridH = sdf->Get<int>("grid_h");
    if (sdf->HasElement("cell_size"))
        cellSize = sdf->Get<double>("cell_size");
    
    // Calculate base world extents from grid size and cell size
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
    
    // RF parameters
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
    
    // gNB position
    if (sdf->HasElement("gnb_pose"))
    {
        auto pose = sdf->Get<math::Pose3d>("gnb_pose");
        gnbPos = pose.Pos();
        propConfig.txHeightM = gnbPos.Z();
    }
    
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
    
    // Interactive features
    if (sdf->HasElement("enable_click_query"))
        enableClickQuery = sdf->Get<bool>("enable_click_query");

    // Initialize propagation model
    propModel = PropagationModelFactory::Create(propConfig.model);

    // Initialize pixel buffer
    pixels.assign(gridW * gridH, -200.0f);

    // Setup publishers
    heatmapPub = node.Advertise<msgs::Image>("/gnb/heatmap");
    statusPub = node.Advertise<msgs::StringMsg>("/gnb/heatmap/status");
    clickInfoPub = node.Advertise<msgs::StringMsg>("/gnb/heatmap/click_info");
    queryResultPub = node.Advertise<msgs::StringMsg>("/gnb/heatmap/query_result");
    viewInfoPub = node.Advertise<msgs::StringMsg>("/gnb/heatmap/view_info");

    // Setup subscribers for runtime configuration
    node.Subscribe("/gnb/heatmap/set_model", &HeatmapPlugin::OnModelChangeRequest, this);
    node.Subscribe("/gnb/heatmap/config", &HeatmapPlugin::OnConfigUpdate, this);
    node.Subscribe("/gnb/heatmap/set_position", &HeatmapPlugin::OnGnbPoseChange, this);
    node.Subscribe("/gnb/heatmap/query_position", &HeatmapPlugin::OnMouseClick, this);
    
    // View control subscribers
    node.Subscribe("/gnb/heatmap/zoom", &HeatmapPlugin::OnZoom, this);
    node.Subscribe("/gnb/heatmap/pan", &HeatmapPlugin::OnPan, this);
    node.Subscribe("/gnb/heatmap/set_view", &HeatmapPlugin::OnSetView, this);
    node.Subscribe("/gnb/heatmap/reset_view", &HeatmapPlugin::OnResetView, this);
    node.Subscribe("/gnb/heatmap/center_on_gnb", &HeatmapPlugin::OnCenterOnGnb, this);

    ignmsg << "=== HeatmapPlugin Configuration ===" << std::endl;
    ignmsg << "  Output Resolution: " << gridW << "x" << gridH << std::endl;
    ignmsg << "  Base World Size: " << viewState.baseWorldWidth << "x" 
           << viewState.baseWorldHeight << " m" << std::endl;
    ignmsg << "  Initial View Center: (" << viewState.centerX << ", " 
           << viewState.centerY << ")" << std::endl;
    ignmsg << "  Zoom Range: " << viewState.minZoom << "x - " 
           << viewState.maxZoom << "x" << std::endl;
    ignmsg << "  Frequency: " << propConfig.frequencyHz / 1e9 << " GHz" << std::endl;
    ignmsg << "  Tx Power: " << propConfig.txPowerDbm << " dBm" << std::endl;
    ignmsg << "  Propagation Model: " << propModel->GetName() << std::endl;
    ignmsg << "===============================" << std::endl;
    ignmsg << "View control topics:" << std::endl;
    ignmsg << "  /gnb/heatmap/zoom - Zoom in/out (positive=in, negative=out)" << std::endl;
    ignmsg << "  /gnb/heatmap/pan - Pan view (dx, dy in world units)" << std::endl;
    ignmsg << "  /gnb/heatmap/set_view - Set view (x, y, zoom in pose msg)" << std::endl;
    ignmsg << "  /gnb/heatmap/reset_view - Reset to default view" << std::endl;
    ignmsg << "  /gnb/heatmap/center_on_gnb - Center view on gNB" << std::endl;
    ignmsg << "  /gnb/heatmap/view_info - Current view info (published)" << std::endl;

    // Initial obstacle scan
    UpdateObstacles(_ecm);

    running = true;
    worker = std::thread(&HeatmapPlugin::WorkerLoop, this);
}

void HeatmapPlugin::OnZoom(const msgs::Double &_msg)
{
    std::lock_guard<std::mutex> lock(viewMutex);
    
    double zoomDelta = _msg.data();
    double newZoom = viewState.zoomLevel * (1.0 + zoomDelta * 0.1);
    viewState.zoomLevel = std::clamp(newZoom, viewState.minZoom, viewState.maxZoom);
    
    needsRecalculation = true;
    ignmsg << "Zoom level: " << viewState.zoomLevel << "x (visible area: " 
           << viewState.GetVisibleWidth() << "x" << viewState.GetVisibleHeight() 
           << " m)" << std::endl;
}

void HeatmapPlugin::OnPan(const msgs::Vector2d &_msg)
{
    std::lock_guard<std::mutex> lock(viewMutex);
    
    // Pan in world coordinates (scaled by zoom for consistent feel)
    double panScale = 1.0 / viewState.zoomLevel;
    viewState.centerX += _msg.x() * panScale;
    viewState.centerY += _msg.y() * panScale;
    
    needsRecalculation = true;
    ignmsg << "View center: (" << viewState.centerX << ", " << viewState.centerY 
           << ")" << std::endl;
}

void HeatmapPlugin::OnSetView(const msgs::Pose &_msg)
{
    std::lock_guard<std::mutex> lock(viewMutex);
    
    viewState.centerX = _msg.position().x();
    viewState.centerY = _msg.position().y();
    
    // Use z for zoom level if provided and positive
    if (_msg.position().z() > 0)
    {
        viewState.zoomLevel = std::clamp(_msg.position().z(), 
                                          viewState.minZoom, viewState.maxZoom);
    }
    
    needsRecalculation = true;
    ignmsg << "View set to: center=(" << viewState.centerX << ", " 
           << viewState.centerY << "), zoom=" << viewState.zoomLevel << "x" << std::endl;
}

void HeatmapPlugin::OnResetView(const msgs::Empty &/*_msg*/)
{
    std::lock_guard<std::mutex> lock(viewMutex);
    
    viewState.centerX = 0.0;
    viewState.centerY = 0.0;
    viewState.zoomLevel = 1.0;
    
    needsRecalculation = true;
    ignmsg << "View reset to default" << std::endl;
}

void HeatmapPlugin::OnCenterOnGnb(const msgs::Empty &/*_msg*/)
{
    std::lock_guard<std::mutex> lock(viewMutex);
    
    viewState.centerX = gnbPos.X();
    viewState.centerY = gnbPos.Y();
    
    needsRecalculation = true;
    ignmsg << "View centered on gNB at (" << viewState.centerX << ", " 
           << viewState.centerY << ")" << std::endl;
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
        << "visible_h=" << localView.GetVisibleHeight() << ";"
        << "min_x=" << localView.GetMinX() << ";"
        << "max_x=" << localView.GetMaxX() << ";"
        << "min_y=" << localView.GetMinY() << ";"
        << "max_y=" << localView.GetMaxY();
    
    msgs::StringMsg msg;
    msg.set_data(oss.str());
    viewInfoPub.Publish(msg);
}

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

void HeatmapPlugin::OnGnbPoseChange(const msgs::Pose &_msg)
{
    math::Vector3d newPos(
        _msg.position().x(),
        _msg.position().y(),
        _msg.position().z()
    );
    
    {
        std::lock_guard<std::mutex> lock(modelMutex);
        gnbPos = newPos;
        propConfig.txHeightM = gnbPos.Z();
    }
    
    needsRecalculation = true;
    ignmsg << "gNB position updated to: (" << gnbPos.X() << ", " 
           << gnbPos.Y() << ", " << gnbPos.Z() << ")" << std::endl;
}

void HeatmapPlugin::OnMouseClick(const msgs::Vector3d &_msg)
{
    if (!enableClickQuery)
        return;
    
    math::Vector3d clickPos(_msg.x(), _msg.y(), _msg.z());
    QuerySignalAtPosition(clickPos);
}

void HeatmapPlugin::QuerySignalAtPosition(const math::Vector3d &_pos)
{
    std::vector<Obstacle> localObstacles;
    PropagationConfig localConfig;
    std::unique_ptr<IPropagationModel> localModel;
    math::Vector3d localGnbPos;
    
    {
        std::lock_guard<std::mutex> lock1(obstacleMutex);
        std::lock_guard<std::mutex> lock2(modelMutex);
        localObstacles = obstacles;
        localConfig = propConfig;
        localModel = PropagationModelFactory::Create(localConfig.model);
        localGnbPos = gnbPos;
    }
    
    math::Vector3d queryPos(_pos.X(), _pos.Y(), localConfig.rxHeightM);
    double signalDbm = localModel->CalculateReceivedPower(
        localGnbPos, queryPos, localObstacles, localConfig);
    
    double distance = localGnbPos.Distance(queryPos);
    
    std::ostringstream detailOss;
    detailOss << std::fixed << std::setprecision(2);
    detailOss << "=== Signal Query Result ===" << std::endl;
    detailOss << "Position: (" << _pos.X() << ", " << _pos.Y() << ", " << _pos.Z() << ")" << std::endl;
    detailOss << "Distance from gNB: " << distance << " m" << std::endl;
    detailOss << "Signal Strength: " << signalDbm << " dBm" << std::endl;
    detailOss << "Model: " << localModel->GetName() << std::endl;
    
    if (signalDbm >= -70)
        detailOss << "Quality: Excellent" << std::endl;
    else if (signalDbm >= -85)
        detailOss << "Quality: Good" << std::endl;
    else if (signalDbm >= -100)
        detailOss << "Quality: Fair" << std::endl;
    else if (signalDbm >= -115)
        detailOss << "Quality: Poor" << std::endl;
    else
        detailOss << "Quality: Very Poor/No Service" << std::endl;
    
    std::string detailStr = detailOss.str();
    ignmsg << detailStr;
    
    msgs::StringMsg clickMsg;
    std::ostringstream clickOss;
    clickOss << std::fixed << std::setprecision(1);
    clickOss << "Pos: (" << _pos.X() << ", " << _pos.Y() << ") | "
             << "Signal: " << signalDbm << " dBm | "
             << "Dist: " << distance << " m";
    clickMsg.set_data(clickOss.str());
    clickInfoPub.Publish(clickMsg);
    
    msgs::StringMsg resultMsg;
    resultMsg.set_data(detailStr);
    queryResultPub.Publish(resultMsg);
}

void HeatmapPlugin::PublishStatus()
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "model=" << propModel->GetName() << ";"
        << "frequency=" << propConfig.frequencyHz << ";"
        << "tx_power=" << propConfig.txPowerDbm << ";"
        << "tx_pos=(" << gnbPos.X() << "," << gnbPos.Y() << "," << gnbPos.Z() << ");"
        << "tx_height=" << propConfig.txHeightM << ";"
        << "rx_height=" << propConfig.rxHeightM << ";"
        << "wall_loss=" << propConfig.defaultMaterial.penetrationLoss_dB << ";"
        << "obstacles=" << obstacles.size() << ";"
        << "shadowing=" << (propConfig.includeShadowing ? "true" : "false");
    
    msgs::StringMsg msg;
    msg.set_data(oss.str());
    statusPub.Publish(msg);
}

void HeatmapPlugin::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
    if (_info.paused)
        return;

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
        [&](const Entity &modelEntity,
            const components::Model *,
            const components::Name *nameComp,
            const components::Pose *poseComp) -> bool
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
                [&](const Entity &linkEntity,
                    const components::Link *,
                    const components::ParentEntity *parentComp,
                    const components::Pose *linkPoseComp) -> bool
                {
                    if (parentComp->Data() != modelEntity)
                        return true;

                    math::Pose3d linkPose = linkPoseComp->Data();

                    _ecm.Each<components::Collision, components::ParentEntity, 
                             components::Geometry, components::Pose>(
                        [&](const Entity &,
                            const components::Collision *,
                            const components::ParentEntity *collParentComp,
                            const components::Geometry *geomComp,
                            const components::Pose *collPoseComp) -> bool
                        {
                            if (collParentComp->Data() != linkEntity)
                                return true;

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
                                    obs.position - halfSize,
                                    obs.position + halfSize);
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
            igndbg << "Obstacles updated: " << obstacles.size() << " found" << std::endl;
        }
    }
}

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

        // Get current view state
        ViewState localView;
        {
            std::lock_guard<std::mutex> lock(viewMutex);
            localView = viewState;
        }

        // Calculate world bounds for current view
        double minX = localView.GetMinX();
        double maxX = localView.GetMaxX();
        double minY = localView.GetMinY();
        double maxY = localView.GetMaxY();
        double visibleWidth = localView.GetVisibleWidth();
        double visibleHeight = localView.GetVisibleHeight();

        std::vector<float> newPixels(gridW * gridH);

        // Copy data for thread safety
        std::vector<Obstacle> localObstacles;
        PropagationConfig localConfig;
        std::unique_ptr<IPropagationModel> localModel;
        ignition::math::Vector3d txPos;
        
        {
            std::lock_guard<std::mutex> lock1(obstacleMutex);
            std::lock_guard<std::mutex> lock2(modelMutex);
            localObstacles = obstacles;
            localConfig = propConfig;
            localModel = PropagationModelFactory::Create(localConfig.model);
            txPos = gnbPos;
        }

        for (int y = 0; y < gridH && running; ++y)
        {
            for (int x = 0; x < gridW; ++x)
            {
                // Map pixel to world coordinates based on current view
                double nx = static_cast<double>(x) / (gridW - 1);
                double ny = static_cast<double>(y) / (gridH - 1);
                
                double worldX = minX + nx * visibleWidth;
                double worldY = minY + ny * visibleHeight;

                math::Vector3d rxPos(worldX, worldY, localConfig.rxHeightM);

                double rxPower = localModel->CalculateReceivedPower(
                    txPos, rxPos, localObstacles, localConfig);

                newPixels[y * gridW + x] = static_cast<float>(rxPower);
            }
        }

        {
            std::lock_guard<std::mutex> lock(bufMutex);
            pixels = std::move(newPixels);
        }

        igndbg << "Heatmap recalculated for view center=(" << localView.centerX 
               << ", " << localView.centerY << "), zoom=" << localView.zoomLevel 
               << "x" << std::endl;
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

    const float MIN_DBM = -120.0f;
    const float MAX_DBM = -30.0f;
    const float RANGE_DBM = MAX_DBM - MIN_DBM;

    {
        std::lock_guard<std::mutex> lock(bufMutex);
        
        for (int i = 0; i < gridW * gridH; ++i)
        {
            float dbm = pixels[i];
            
            if (dbm <= MIN_DBM)
            {
                rgbData[i * 3 + 0] = 30;
                rgbData[i * 3 + 1] = 30;
                rgbData[i * 3 + 2] = 50;
                continue;
            }
            
            float normalized = (dbm - MIN_DBM) / RANGE_DBM;
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