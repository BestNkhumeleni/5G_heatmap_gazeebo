#include "HeatmapRenderPlugin.hh"

#include <ignition/plugin/Register.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/RenderEvents.hh>
#include <ignition/common/Console.hh>
#include <sdf/Param.hh>
#include <cmath>


using namespace heatmap_plugin;
using namespace ignition;
using namespace ignition::rendering;
using namespace ignition::transport;


HeatmapRenderPlugin::~HeatmapRenderPlugin()
{
this->running = false;
if (this->worker.joinable())
this->worker.join();
}


void HeatmapRenderPlugin::Load(ScenePtr _scene, sdf::ElementPtr _sdf)
{
this->scene = _scene;


// Read SDF params (with safe checks)
if (_sdf->HasElement("grid_w")) this->gridW = _sdf->Get<int>("grid_w");
if (_sdf->HasElement("grid_h")) this->gridH = _sdf->Get<int>("grid_h");
if (_sdf->HasElement("frequency_hz")) this->freqHz = _sdf->Get<double>("frequency_hz");
if (_sdf->HasElement("tx_dbm")) this->ptxDbm = _sdf->Get<double>("tx_dbm");
if (_sdf->HasElement("block_loss_db")) this->blockLossDb = _sdf->Get<double>("block_loss_db");
if (_sdf->HasElement("gnb_pose"))
{
auto pose = _sdf->Get<ignition::math::Vector3d>("gnb_pose");
this->gnbPos = ignition::math::Vector3d(pose.X(), pose.Y(), pose.Z());
}


this->pixels.assign(this->gridW * this->gridH, -200.0f);
this->node = std::make_unique<Node>();
this->node->Advertise("/gnb/heatmap", "ignition.msgs.Image");


// Create a RayQuery from the scene for occlusion tests
this->rayQuery = this->scene->CreateRayQuery();


// Start worker thread
this->running = true;
this->worker = std::thread(&HeatmapRenderPlugin::WorkerLoop, this);


// Connect to pre-render to publish latest buffer
this->scene->AddRenderEventCallback([this]() { this->OnPreRender(); });


ignmsg << "HeatmapRenderPlugin loaded (grid=" << this->gridW << "x" << this->gridH << ")" << std::endl;
}


void HeatmapRenderPlugin::WorkerLoop()
{
// Simple bounding box around gnb for sampling (adjust as desired)
double half_width = 10.0; // meters
double half_height = 10.0;
double sampleZ = this->gnbPos.Z();


while (this->running)
{
std::vector<float> newGrid(this->gridW * this->gridH);


for (int y = 0; y < this->gridH; ++y)
{
for (int x = 0; x < this->gridW; ++x)
{
double nx = (double)x / (this->gridW - 1);
double ny = (double)y / (this->gridH - 1);
double worldX = this->gnbPos.X() - half_width + nx * (2.0 * half_width);
double worldY = this->gnbPos.Y() - half_height + ny * (2.0 * half_height);


ignition::math::Vector3d samplePos(worldX, worldY, sampleZ);
double d = samplePos.Distance(this->gnbPos);
double prDbm = this->ptxDbm - FSPL_db(d, this->freqHz);


// Ray query occlusion: QueryClosestIntersection returns a RayQueryResult
auto result = this->rayQuery->ClosestIntersection(this->gnbPos, samplePos);
if (result && result->visual)
{
IGNITION_ADD_PLUGIN(heatmap_plugin::HeatmapRenderPlugin, ignition::rendering::RenderPlugin)