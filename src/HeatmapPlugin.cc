#include "heatmap_plugin/HeatmapPlugin.hh"

#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Geometry.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/msgs/image.pb.h>
#include <ignition/common/Console.hh>
#include <cmath>
#include <algorithm>

using namespace heatmap_plugin;
using namespace ignition;
using namespace ignition::gazebo;

// Speed of light in m/s
constexpr double C = 299792458.0;

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
    // Parse SDF parameters
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
    if (sdf->HasElement("block_loss_db"))
        blockLossDb = sdf->Get<double>("block_loss_db");
    if (sdf->HasElement("gnb_pose"))
    {
        auto pose = sdf->Get<math::Pose3d>("gnb_pose");
        gnbPos = pose.Pos();
    }

    // Initialize pixel buffer with very low signal
    pixels.assign(gridW * gridH, -200.0f);

    // Advertise heatmap topic
    heatmapPub = node.Advertise<msgs::Image>("/gnb/heatmap");

    ignmsg << "HeatmapPlugin configured:\n"
           << "  Grid: " << gridW << "x" << gridH << "\n"
           << "  Cell size: " << cellSize << " m\n"
           << "  Frequency: " << freqHz / 1e9 << " GHz\n"
           << "  Tx power: " << ptxDbm << " dBm\n"
           << "  gNB position: " << gnbPos << "\n";

    // Start the background computation thread
    running = true;
    worker = std::thread(&HeatmapPlugin::WorkerLoop, this);
}

void HeatmapPlugin::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
    // Skip if paused
    if (_info.paused)
        return;

    // Provide ECM pointer to worker thread
    {
        std::lock_guard<std::mutex> lock(ecmMutex);
        ecmPtr = &_ecm;
    }

    // Publish heatmap at a reduced rate (every 10th step or ~100ms)
    static int frameCount = 0;
    if (++frameCount % 10 == 0)
    {
        PublishHeatmap();
    }
}

double HeatmapPlugin::FSPL_dB(double d, double f)
{
    // Free-Space Path Loss: FSPL(dB) = 20*log10(d) + 20*log10(f) + 20*log10(4*pi/c)
    // Simplified: FSPL = 20*log10(d) + 20*log10(f) - 147.55
    if (d < 0.1)
        d = 0.1;  // Minimum distance to avoid singularity
    
    return 20.0 * std::log10(d) + 20.0 * std::log10(f) - 147.55;
}

bool HeatmapPlugin::IsOccluded(
    const math::Vector3d &from,
    const math::Vector3d &to,
    EntityComponentManager &ecm)
{
    // Simple occlusion check: iterate through entities with geometry
    // and check if any bounding box intersects the ray from->to
    //
    // NOTE: This is a simplified approach. For production use, consider
    // using ignition::rendering::RayQuery with a proper rendering scene,
    // or implement proper ray-box intersection tests.
    
    math::Vector3d dir = to - from;
    double dist = dir.Length();
    if (dist < 0.01)
        return false;
    dir.Normalize();

    bool occluded = false;

    // Query all entities with Visual and Pose components
    ecm.Each<components::Visual, components::Pose>(
        [&](const Entity &/*entity*/,
            const components::Visual */*visual*/,
            const components::Pose *pose) -> bool
        {
            // Get the visual's world position
            auto visualPos = pose->Data().Pos();
            
            // Simple proximity check - if any visual is close to the line
            // between from and to, consider it blocked
            // This is a rough approximation
            math::Vector3d toVisual = visualPos - from;
            double proj = toVisual.Dot(dir);
            
            if (proj > 0 && proj < dist)
            {
                math::Vector3d closest = from + dir * proj;
                double perpDist = (visualPos - closest).Length();
                
                // If within 0.5m of the ray, consider blocked
                // Adjust this threshold based on your scene
                if (perpDist < 0.5)
                {
                    occluded = true;
                    return false;  // Stop iteration, found occlusion
                }
            }
            return true;  // Continue iteration
        });

    return occluded;
}

void HeatmapPlugin::WorkerLoop()
{
    while (running)
    {
        // Compute grid bounds centered on gNB
        double halfW = (gridW * cellSize) / 2.0;
        double halfH = (gridH * cellSize) / 2.0;
        double sampleZ = gnbPos.Z();

        std::vector<float> newPixels(gridW * gridH);

        for (int y = 0; y < gridH && running; ++y)
        {
            for (int x = 0; x < gridW; ++x)
            {
                // Map grid cell to world coordinates
                double nx = static_cast<double>(x) / (gridW - 1);
                double ny = static_cast<double>(y) / (gridH - 1);
                double worldX = gnbPos.X() - halfW + nx * (2.0 * halfW);
                double worldY = gnbPos.Y() - halfH + ny * (2.0 * halfH);

                math::Vector3d samplePos(worldX, worldY, sampleZ);
                double d = samplePos.Distance(gnbPos);

                // Compute received power using FSPL
                double prDbm = ptxDbm - FSPL_dB(d, freqHz);

                // Check for occlusion (simplified - would need ECM access)
                // For now, we skip occlusion in the worker thread
                // In a real implementation, you'd cache geometry or use
                // a thread-safe scene query

                // Clamp to reasonable range
                prDbm = std::clamp(prDbm, -120.0, ptxDbm);

                newPixels[y * gridW + x] = static_cast<float>(prDbm);
            }
        }

        // Swap buffers
        {
            std::lock_guard<std::mutex> lock(bufMutex);
            pixels = std::move(newPixels);
        }

        // Sleep to avoid spinning too fast (update ~10 Hz)
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
            // Map dBm to color (gradient from blue to red)
            // Typical range: -120 dBm (weak) to -30 dBm (strong)
            float dbm = pixels[i];
            float normalized = (dbm + 120.0f) / 90.0f;  // Map [-120, -30] to [0, 1]
            normalized = std::clamp(normalized, 0.0f, 1.0f);

            // Heatmap colormap: blue -> cyan -> green -> yellow -> red
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

// Register the plugin with Ignition Gazebo
IGNITION_ADD_PLUGIN(
    heatmap_plugin::HeatmapPlugin,
    System,
    HeatmapPlugin::ISystemConfigure,
    HeatmapPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
    heatmap_plugin::HeatmapPlugin,
    "heatmap_plugin::HeatmapPlugin")