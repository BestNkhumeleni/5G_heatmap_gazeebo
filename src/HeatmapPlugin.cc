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

    // Setup subscribers for runtime configuration
    node.Subscribe("/gnb/heatmap/set_model", &HeatmapPlugin::OnModelChangeRequest, this);
    node.Subscribe("/gnb/heatmap/config", &HeatmapPlugin::OnConfigUpdate, this);
    node.Subscribe("/gnb/heatmap/set_position", &HeatmapPlugin::OnGnbPoseChange, this);
    node.Subscribe("/gnb/heatmap/query_position", &HeatmapPlugin::OnMouseClick, this);

    ignmsg << "=== HeatmapPlugin Configuration ===" << std::endl;
    ignmsg << "  Grid: " << gridW << "x" << gridH << " (" << (gridW * cellSize) << "x" << (gridH * cellSize) << " m)" << std::endl;
    ignmsg << "  Frequency: " << propConfig.frequencyHz / 1e9 << " GHz" << std::endl;
    ignmsg << "  Tx Power: " << propConfig.txPowerDbm << " dBm" << std::endl;
    ignmsg << "  Tx Height: " << propConfig.txHeightM << " m" << std::endl;
    ignmsg << "  Rx Height: " << propConfig.rxHeightM << " m" << std::endl;
    ignmsg << "  Propagation Model: " << propModel->GetName() << std::endl;
    ignmsg << "  Wall Penetration Loss: " << propConfig.defaultMaterial.penetrationLoss_dB << " dB" << std::endl;
    ignmsg << "  Shadowing: " << (propConfig.includeShadowing ? "enabled" : "disabled") << std::endl;
    ignmsg << "  Click Query: " << (enableClickQuery ? "enabled" : "disabled") << std::endl;
    ignmsg << "===============================" << std::endl;
    ignmsg << "Runtime control topics:" << std::endl;
    ignmsg << "  /gnb/heatmap/set_model - Change propagation model" << std::endl;
    ignmsg << "  /gnb/heatmap/config - Update configuration" << std::endl;
    ignmsg << "  /gnb/heatmap/set_position - Move gNB transmitter" << std::endl;
    ignmsg << "  /gnb/heatmap/query_position - Query signal at position" << std::endl;
    ignmsg << "  /gnb/heatmap/status - Current status (published)" << std::endl;
    ignmsg << "  /gnb/heatmap/click_info - Click query results" << std::endl;

    // Initial obstacle scan
    UpdateObstacles(_ecm);

    running = true;
    worker = std::thread(&HeatmapPlugin::WorkerLoop, this);
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
        ignwarn << "Available: free_space, 3gpp_umi, 3gpp_uma, ray_tracing, hybrid" << std::endl;
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
    // Parse simple key=value format
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
    // Copy data for thread safety
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
    
    // Calculate signal strength at queried position
    math::Vector3d queryPos(_pos.X(), _pos.Y(), localConfig.rxHeightM);
    double signalDbm = localModel->CalculateReceivedPower(
        localGnbPos, queryPos, localObstacles, localConfig);
    
    // Calculate distance
    double distance = localGnbPos.Distance(queryPos);
    
    // Publish detailed result
    std::ostringstream detailOss;
    detailOss << std::fixed << std::setprecision(2);
    detailOss << "=== Signal Query Result ===" << std::endl;
    detailOss << "Position: (" << _pos.X() << ", " << _pos.Y() << ", " << _pos.Z() << ")" << std::endl;
    detailOss << "Distance from gNB: " << distance << " m" << std::endl;
    detailOss << "Signal Strength: " << signalDbm << " dBm" << std::endl;
    detailOss << "Model: " << localModel->GetName() << std::endl;
    
    // Signal quality interpretation
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
    
    // Publish to terminal
    ignmsg << detailStr;
    
    // Publish to topic for GUI display
    msgs::StringMsg clickMsg;
    std::ostringstream clickOss;
    clickOss << std::fixed << std::setprecision(1);
    clickOss << "Pos: (" << _pos.X() << ", " << _pos.Y() << ") | "
             << "Signal: " << signalDbm << " dBm | "
             << "Dist: " << distance << " m";
    clickMsg.set_data(clickOss.str());
    clickInfoPub.Publish(clickMsg);
    
    // Publish detailed result
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
    
    // Update obstacles periodically
    static uint64_t lastObstacleUpdate = 0;
    if (currentIter - lastObstacleUpdate >= 50)
    {
        UpdateObstacles(_ecm);
        lastObstacleUpdate = currentIter;
    }

    // Publish heatmap
    static uint64_t lastPublish = 0;
    if (currentIter - lastPublish >= 10)
    {
        PublishHeatmap();
        lastPublish = currentIter;
    }
    
    // Publish status periodically
    if (currentIter - lastStatusPublish >= 100)
    {
        PublishStatus();
        lastStatusPublish = currentIter;
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

        double halfW = (gridW * cellSize) / 2.0;
        double halfH = (gridH * cellSize) / 2.0;

        std::vector<float> newPixels(gridW * gridH);

        // Copy data for thread safety
        std::vector<Obstacle> localObstacles;
        PropagationConfig localConfig;
        std::unique_ptr<IPropagationModel> localModel;
        
        {
            std::lock_guard<std::mutex> lock1(obstacleMutex);
            std::lock_guard<std::mutex> lock2(modelMutex);
            localObstacles = obstacles;
            localConfig = propConfig;
            localModel = PropagationModelFactory::Create(localConfig.model);
        }

        ignition::math::Vector3d txPos = gnbPos;

        for (int y = 0; y < gridH && running; ++y)
        {
            for (int x = 0; x < gridW; ++x)
            {
                double nx = static_cast<double>(x) / (gridW - 1);
                double ny = static_cast<double>(y) / (gridH - 1);
                double worldX = gnbPos.X() - halfW + nx * (2.0 * halfW);
                double worldY = gnbPos.Y() - halfH + ny * (2.0 * halfH);

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

        igndbg << "Heatmap recalculated using " << localModel->GetName() << std::endl;
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

    {
        std::lock_guard<std::mutex> lock(bufMutex);
        
        for (int i = 0; i < gridW * gridH; ++i)
        {
            float dbm = pixels[i];
            
            if (dbm <= -140.0f)
            {
                rgbData[i * 3 + 0] = 50;
                rgbData[i * 3 + 1] = 50;
                rgbData[i * 3 + 2] = 50;
                continue;
            }
            
            float normalized = (dbm + 120.0f) / 90.0f;
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