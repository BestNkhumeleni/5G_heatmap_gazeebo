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
#include <ignition/common/Console.hh>
#include <sdf/Geometry.hh>
#include <sdf/Box.hh>
#include <sdf/Cylinder.hh>
#include <cmath>
#include <algorithm>

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
    
    if (sdf->HasElement("grid_w"))
        gridW = sdf->Get<int>("grid_w");
    if (sdf->HasElement("grid_h"))
        gridH = sdf->Get<int>("grid_h");
    if (sdf->HasElement("cell_size"))
        cellSize = sdf->Get<double>("cell_size");
    if (sdf->HasElement("frequency_hz"))
        freqHz = sdf->Get<double>("frequency_hz");
    if (sdf->HasElement("tx_dbm"))
        ptxDbm = sdf->Get<double>("tx_dbm");
    if (sdf->HasElement("wall_loss_db"))
        wallLossDb = sdf->Get<double>("wall_loss_db");
    if (sdf->HasElement("gnb_pose"))
    {
        auto pose = sdf->Get<math::Pose3d>("gnb_pose");
        gnbPos = pose.Pos();
    }

    pixels.assign(gridW * gridH, -200.0f);
    heatmapPub = node.Advertise<msgs::Image>("/gnb/heatmap");

    ignmsg << "HeatmapPlugin configured:\n"
           << "  Grid: " << gridW << "x" << gridH << "\n"
           << "  Cell size: " << cellSize << " m\n"
           << "  Coverage area: " << (gridW * cellSize) << "x" << (gridH * cellSize) << " m\n"
           << "  Frequency: " << freqHz / 1e9 << " GHz\n"
           << "  Tx power: " << ptxDbm << " dBm\n"
           << "  Wall loss: " << wallLossDb << " dB per wall\n"
           << "  gNB position: " << gnbPos << "\n";

    // Initial obstacle scan
    UpdateObstacles(_ecm);

    running = true;
    worker = std::thread(&HeatmapPlugin::WorkerLoop, this);
}

void HeatmapPlugin::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
    if (_info.paused)
        return;

    // Update obstacles every frame to catch new/moved buildings
    static uint64_t lastObstacleUpdate = 0;
    uint64_t currentIter = _info.iterations;
    
    // Update obstacles every 50 iterations (~50ms at 1000Hz)
    if (currentIter - lastObstacleUpdate >= 50)
    {
        UpdateObstacles(_ecm);
        lastObstacleUpdate = currentIter;
    }

    // Publish heatmap at reduced rate
    static uint64_t lastPublish = 0;
    if (currentIter - lastPublish >= 10)
    {
        PublishHeatmap();
        lastPublish = currentIter;
    }
}

void HeatmapPlugin::UpdateObstacles(EntityComponentManager &_ecm)
{
    std::vector<Obstacle> newObstacles;

    // Iterate through all models
    _ecm.Each<components::Model, components::Name, components::Pose>(
        [&](const Entity &modelEntity,
            const components::Model *,
            const components::Name *nameComp,
            const components::Pose *poseComp) -> bool
        {
            std::string modelName = nameComp->Data();
            
            // Skip ground plane and gNB tower
            std::string lowerName = modelName;
            std::transform(lowerName.begin(), lowerName.end(), lowerName.begin(), ::tolower);
            
            if (lowerName.find("ground") != std::string::npos ||
                lowerName.find("gnb") != std::string::npos ||
                lowerName.find("sun") != std::string::npos)
            {
                return true;  // Continue iteration
            }

            // Check if model is static (optional - you might want dynamic obstacles too)
            auto staticComp = _ecm.Component<components::Static>(modelEntity);
            
            math::Pose3d modelPose = poseComp->Data();

            // Find all links in this model
            _ecm.Each<components::Link, components::ParentEntity, components::Pose>(
                [&](const Entity &linkEntity,
                    const components::Link *,
                    const components::ParentEntity *parentComp,
                    const components::Pose *linkPoseComp) -> bool
                {
                    // Check if this link belongs to our model
                    if (parentComp->Data() != modelEntity)
                        return true;

                    math::Pose3d linkPose = linkPoseComp->Data();

                    // Find collisions in this link
                    _ecm.Each<components::Collision, components::ParentEntity, 
                             components::Geometry, components::Pose>(
                        [&](const Entity &/*collEntity*/,
                            const components::Collision *,
                            const components::ParentEntity *collParentComp,
                            const components::Geometry *geomComp,
                            const components::Pose *collPoseComp) -> bool
                        {
                            if (collParentComp->Data() != linkEntity)
                                return true;

                            const sdf::Geometry &geom = geomComp->Data();
                            math::Pose3d collPose = collPoseComp->Data();

                            // Calculate world pose
                            math::Pose3d worldPose = modelPose * linkPose * collPose;

                            Obstacle obs;
                            obs.name = modelName;
                            obs.position = worldPose.Pos();

                            bool validObstacle = false;

                            if (geom.Type() == sdf::GeometryType::BOX && geom.BoxShape())
                            {
                                obs.size = geom.BoxShape()->Size();
                                validObstacle = true;
                            }
                            else if (geom.Type() == sdf::GeometryType::CYLINDER && geom.CylinderShape())
                            {
                                // Approximate cylinder as box
                                double r = geom.CylinderShape()->Radius();
                                double h = geom.CylinderShape()->Length();
                                obs.size = math::Vector3d(r * 2, r * 2, h);
                                validObstacle = true;
                            }

                            if (validObstacle)
                            {
                                // Create AABB (axis-aligned, ignoring rotation for simplicity)
                                // For rotated boxes, you'd need OBB intersection
                                math::Vector3d halfSize = obs.size / 2.0;
                                obs.bbox = math::AxisAlignedBox(
                                    obs.position - halfSize,
                                    obs.position + halfSize
                                );
                                
                                newObstacles.push_back(obs);
                            }

                            return true;
                        });

                    return true;
                });

            return true;
        });

    // Check if obstacles changed
    bool changed = false;
    {
        std::lock_guard<std::mutex> lock(obstacleMutex);
        if (newObstacles.size() != obstacles.size())
        {
            changed = true;
        }
        else
        {
            for (size_t i = 0; i < newObstacles.size(); ++i)
            {
                if (newObstacles[i].position.Distance(obstacles[i].position) > 0.01 ||
                    newObstacles[i].size.Distance(obstacles[i].size) > 0.01)
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
            
            ignmsg << "Obstacles updated: " << obstacles.size() << " found" << std::endl;
            for (const auto &obs : obstacles)
            {
                ignmsg << "  - " << obs.name << " at " << obs.position 
                       << " size " << obs.size << std::endl;
            }
        }
    }
}

bool HeatmapPlugin::RayIntersectsAABB(
    const math::Vector3d &rayOrigin,
    const math::Vector3d &rayDir,
    double rayLength,
    const math::AxisAlignedBox &box)
{
    double tmin = 0.0;
    double tmax = rayLength;

    for (int i = 0; i < 3; ++i)
    {
        double origin, dir, bmin, bmax;
        
        switch (i)
        {
            case 0:
                origin = rayOrigin.X();
                dir = rayDir.X();
                bmin = box.Min().X();
                bmax = box.Max().X();
                break;
            case 1:
                origin = rayOrigin.Y();
                dir = rayDir.Y();
                bmin = box.Min().Y();
                bmax = box.Max().Y();
                break;
            default:
                origin = rayOrigin.Z();
                dir = rayDir.Z();
                bmin = box.Min().Z();
                bmax = box.Max().Z();
                break;
        }

        if (std::abs(dir) < 1e-8)
        {
            if (origin < bmin || origin > bmax)
                return false;
        }
        else
        {
            double invDir = 1.0 / dir;
            double t1 = (bmin - origin) * invDir;
            double t2 = (bmax - origin) * invDir;

            if (t1 > t2)
                std::swap(t1, t2);

            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);

            if (tmin > tmax)
                return false;
        }
    }

    return tmin <= tmax && tmax > 0;
}

int HeatmapPlugin::CountOcclusions(
    const math::Vector3d &from,
    const math::Vector3d &to) const
{
    math::Vector3d dir = to - from;
    double length = dir.Length();
    
    if (length < 0.01)
        return 0;
    
    dir /= length;  // Normalize

    int count = 0;
    
    std::lock_guard<std::mutex> lock(obstacleMutex);
    for (const auto &obs : obstacles)
    {
        if (RayIntersectsAABB(from, dir, length, obs.bbox))
        {
            ++count;
        }
    }

    return count;
}

bool HeatmapPlugin::IsInsideObstacle(const math::Vector3d &point) const
{
    std::lock_guard<std::mutex> lock(obstacleMutex);
    for (const auto &obs : obstacles)
    {
        if (obs.bbox.Contains(point))
        {
            return true;
        }
    }
    return false;
}

double HeatmapPlugin::FSPL_dB(double d, double f)
{
    if (d < 0.1)
        d = 0.1;
    return 20.0 * std::log10(d) + 20.0 * std::log10(f) - 147.55;
}

void HeatmapPlugin::WorkerLoop()
{
    while (running)
    {
        // Only recalculate if needed
        if (!needsRecalculation)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }
        
        needsRecalculation = false;

        double halfW = (gridW * cellSize) / 2.0;
        double halfH = (gridH * cellSize) / 2.0;
        double sampleZ = gnbPos.Z();

        std::vector<float> newPixels(gridW * gridH);

        // Copy obstacles for thread safety
        std::vector<Obstacle> localObstacles;
        {
            std::lock_guard<std::mutex> lock(obstacleMutex);
            localObstacles = obstacles;
        }

        for (int y = 0; y < gridH && running; ++y)
        {
            for (int x = 0; x < gridW; ++x)
            {
                double nx = static_cast<double>(x) / (gridW - 1);
                double ny = static_cast<double>(y) / (gridH - 1);
                double worldX = gnbPos.X() - halfW + nx * (2.0 * halfW);
                double worldY = gnbPos.Y() - halfH + ny * (2.0 * halfH);

                math::Vector3d samplePos(worldX, worldY, sampleZ);
                double d = samplePos.Distance(gnbPos);

                // Check if inside an obstacle
                bool insideObstacle = false;
                for (const auto &obs : localObstacles)
                {
                    if (obs.bbox.Contains(samplePos))
                    {
                        insideObstacle = true;
                        break;
                    }
                }

                double prDbm;
                if (insideObstacle)
                {
                    prDbm = -150.0;  // No signal inside buildings
                }
                else
                {
                    // Compute FSPL
                    prDbm = ptxDbm - FSPL_dB(d, freqHz);

                    // Count wall penetrations and apply loss
                    math::Vector3d dir = samplePos - gnbPos;
                    double length = dir.Length();
                    if (length > 0.01)
                    {
                        dir /= length;
                        
                        for (const auto &obs : localObstacles)
                        {
                            if (RayIntersectsAABB(gnbPos, dir, length, obs.bbox))
                            {
                                prDbm -= wallLossDb;
                            }
                        }
                    }
                }

                prDbm = std::clamp(prDbm, -120.0, ptxDbm);
                newPixels[y * gridW + x] = static_cast<float>(prDbm);
            }
        }

        {
            std::lock_guard<std::mutex> lock(bufMutex);
            pixels = std::move(newPixels);
        }

        igndbg << "Heatmap recalculated with " << localObstacles.size() << " obstacles" << std::endl;
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
            
            // Inside buildings - dark gray
            if (dbm <= -140.0f)
            {
                rgbData[i * 3 + 0] = 50;
                rgbData[i * 3 + 1] = 50;
                rgbData[i * 3 + 2] = 50;
                continue;
            }
            
            // Map dBm to [0,1]: -120dBm -> 0, -30dBm -> 1
            float normalized = (dbm + 120.0f) / 90.0f;
            normalized = std::clamp(normalized, 0.0f, 1.0f);

            // Color gradient: blue -> cyan -> green -> yellow -> red
            uint8_t r, g, b;
            if (normalized < 0.25f)
            {
                float t = normalized / 0.25f;
                r = 0;
                g = static_cast<uint8_t>(255 * t);
                b = 255;
            }
            else if (normalized < 0.5f)
            {
                float t = (normalized - 0.25f) / 0.25f;
                r = 0;
                g = 255;
                b = static_cast<uint8_t>(255 * (1 - t));
            }
            else if (normalized < 0.75f)
            {
                float t = (normalized - 0.5f) / 0.25f;
                r = static_cast<uint8_t>(255 * t);
                g = 255;
                b = 0;
            }
            else
            {
                float t = (normalized - 0.75f) / 0.25f;
                r = 255;
                g = static_cast<uint8_t>(255 * (1 - t));
                b = 0;
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