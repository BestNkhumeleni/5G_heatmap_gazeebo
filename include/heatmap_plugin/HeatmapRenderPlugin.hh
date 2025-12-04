#pragma once


#include <ignition/rendering/RenderPlugin.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/RayQuery.hh>
#include <ignition/transport/Node.hh>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>


namespace heatmap_plugin
{


class HeatmapRenderPlugin : public ignition::rendering::RenderPlugin
{
public:
HeatmapRenderPlugin() = default;
virtual ~HeatmapRenderPlugin();


void Load(ignition::rendering::ScenePtr _scene, sdf::ElementPtr _sdf) override;


private:
void WorkerLoop();
void OnPreRender();
static double FSPL_db(double d, double f);


// scene and ray query
ignition::rendering::ScenePtr scene{};
ignition::rendering::RayQueryPtr rayQuery{};


// transport node
std::unique_ptr<ignition::transport::Node> node{};


std::vector<float> pixels; // dBm values
int gridW{128}, gridH{128};
double freqHz{3.5e9};
double ptxDbm{30.0};
double blockLossDb{30.0};
ignition::math::Vector3d gnbPos{0,0,1.5};


std::thread worker;
std::atomic<bool> running{false};
std::mutex bufMutex;
};


} // namespace