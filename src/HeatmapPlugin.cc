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
#include <ignition/gazebo/Util.hh>
#include <ignition/msgs/image.pb.h>
#include <ignition/common/Console.hh>
#include <sdf/Geometry.hh>
#include <sdf/Box.hh>
#include <cmath>
#include <algorithm>
#include <limits>

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
    EntityComponentManager &/*_ecm*/,
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
           << "  Frequency: " << freqHz / 1e9 << " GHz\n"
           << "  Tx power: " << ptxDbm << " dBm\n"
           << "  Wall loss: " << wallLossDb << " dB\n"
           << "  gNB position: " << gnbPos << "\n";

    running = true;
    worker = std::thread(&HeatmapPlugin::WorkerLoop, this);
}

void HeatmapPlugin::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
    if (_info.paused)
        return;

    // Update obstacles periodically (every 100 iterations)
    static int updateCounter = 0;
    if (++updateCounter % 100 == 0)
    {
        UpdateObstacles(_ecm);
    }

    // Publish heatmap at reduced rate
    static int frameCount = 0;
    if (++frameCount % 10 == 0)
    {
        PublishHeatmap();
    }
}

void HeatmapPlugin::UpdateObstacles(EntityComponentManager &_ecm)
{
    std::vector<Obstacle> newObstacles;

    // Find all static models with collision geometry
    _ecm.Each<components::Model, components::Name, components::Static>(
        [&](const Entity &entity,
            const components::Model *,
            const components::Name *name,
            const components::Static *isStatic) -> bool
        {
            // Skip non-static models and ground plane
            if (!isStatic->Data())
                return true;
            if (name->Data().find("ground") != std::string::npos)
                return true;
            if (name->Data().find("gnb") != std::string::npos)
                return true;

            // Get model pose
            auto poseComp = _ecm.Component<components::Pose>(entity);
            if (!poseComp)
                return true;

            math::Pose3d modelPose = poseComp->Data();

            // Find collision children to get geometry
            auto links = _ecm.ChildrenByComponents(entity, components::Link());
            for (const auto &link : links)
            {
                auto collisions = _ecm.ChildrenByComponents(link, components::Collision());
                for (const auto &collision : collisions)
                {
                    auto geomComp = _ecm.Component<components::Geometry>(collision);
                    auto collPoseComp = _ecm.Component<components::Pose>(collision);
                    
                    if (!geomComp)
                        continue;

                    const sdf::Geometry &geom = geomComp->Data();
                    
                    // Handle box geometry
                    if (geom.Type() == sdf::GeometryType::BOX && geom.BoxShape())
                    {
                        math::Vector3d size = geom.BoxShape()->Size();
                        
                        // Get collision pose relative to model
                        math::Pose3d collPose;
                        if (collPoseComp)
                            collPose = collPoseComp->Data();
                        
                        // Calculate world position
                        math::Vector3d worldPos = modelPose.Pos() + 
                            modelPose.Rot().RotateVector(collPose.Pos());

                        Obstacle obs;
                        obs.name = name->Data();
                        obs.position = worldPos;
                        obs.size = size;
                        
                        // Create axis-aligned bounding box
                        math::Vector3d halfSize = size / 2.0;
                        obs.bbox = math::AxisAlignedBox(
                            worldPos - halfSize,
                            worldPos + halfSize
                        );
                        
                        newObstacles.push_back(obs);
                        
                        igndbg << "Found obstacle: " << obs.name 
                               << " at " << worldPos 
                               << " size " << size << std::endl;
                    }
                }
            }
            return true;
        });

    // Update obstacle list thread-safely
    {
        std::lock_guard<std::mutex> lock(obstacleMutex);
        obstacles = std::move(newObstacles);
        obstaclesUpdated = true;
    }
    
    igndbg << "Updated obstacles: " << obstacles.size() << " found" << std::endl;
}

bool HeatmapPlugin::RayIntersectsAABB(
    const math::Vector3d &rayOrigin,
    const math::Vector3d &rayDir,
    double rayLength,
    const math::AxisAlignedBox &box)
{
    // Slab method for ray-AABB intersection
    double tmin = 0.0;
    double tmax = rayLength;

    for (int i = 0; i < 3; ++i)
    {
        double origin = (i == 0) ? rayOrigin.X() : (i == 1) ? rayOrigin.Y() : rayOrigin.Z();
        double dir = (i == 0) ? rayDir.X() : (i == 1) ? rayDir.Y() : rayDir.Z();
        double bmin = (i == 0) ? box.Min().X() : (i == 1) ? box.Min().Y() : box.Min().Z();
        double bmax = (i == 0) ? box.Max().X() : (i == 1) ? box.Max().Y() : box.Max().Z();

        if (std::abs(dir) < 1e-8)
        {
            // Ray is parallel to slab
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

    return true;
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
    
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(obstacleMutex));
    for (const auto &obs : obstacles)
    {
        if (RayIntersectsAABB(from, dir, length, obs.bbox))
        {
            ++count;
        }
    }

    return count;
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
        double halfW = (gridW * cellSize) / 2.0;
        double halfH = (gridH * cellSize) / 2.0;
        double sampleZ = gnbPos.Z();

        std::vector<float> newPixels(gridW * gridH);

        // Copy obstacles for this iteration
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

                // Check if sample point is inside an obstacle (building)
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
                    // Inside building - no signal (or very weak)
                    prDbm = -150.0;
                }
                else
                {
                    // Compute FSPL
                    prDbm = ptxDbm - FSPL_dB(d, freqHz);

                    // Check for occlusions and apply wall loss
                    int numWalls = CountOcclusions(gnbPos, samplePos);
                    if (numWalls > 0)
                    {
                        prDbm -= numWalls * wallLossDb;
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

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
            
            // Special case for inside buildings
            if (dbm <= -140.0f)
            {
                // Dark gray for building interior
                rgbData[i * 3 + 0] = 40;
                rgbData[i * 3 + 1] = 40;
                rgbData[i * 3 + 2] = 40;
                continue;
            }
            
            float normalized = (dbm + 120.0f) / 90.0f;
            normalized = std::clamp(normalized, 0.0f, 1.0f);

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