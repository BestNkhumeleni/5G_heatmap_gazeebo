#pragma once

#include <ignition/math/Vector3.hh>
#include <ignition/math/AxisAlignedBox.hh>
#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include <random>

namespace heatmap_plugin
{

/// Forward declaration
struct Obstacle;

/// Ray hit information for ray tracing
struct RayHit
{
    bool hit{false};
    double distance{0.0};
    ignition::math::Vector3d point;
    ignition::math::Vector3d normal;
    const Obstacle* obstacle{nullptr};
};

/// Represents a traced ray path (for multipath)
struct RayPath
{
    enum class Type { LOS, REFLECTED, DIFFRACTED };
    
    Type type{Type::LOS};
    double totalDistance{0.0};
    double pathLoss_dB{0.0};
    int numReflections{0};
    int numDiffractions{0};
    std::vector<ignition::math::Vector3d> waypoints;
};

/// Material properties for buildings
struct MaterialProperties
{
    std::string name{"concrete"};
    double penetrationLoss_dB{15.0};      // Loss per wall penetration
    double reflectionCoeff{0.7};           // Reflection coefficient (0-1)
    double absorptionCoeff{0.3};           // Absorption coefficient (0-1)
};

/// Environment type for 3GPP models
enum class EnvironmentType
{
    URBAN_MICRO,      // Street-level small cells
    URBAN_MACRO,      // Elevated macro cells
    RURAL_MACRO,      // Rural areas
    INDOOR_OFFICE,    // Indoor office environment
    INDOOR_FACTORY    // Industrial indoor
};

/// Propagation model type
enum class PropagationModel
{
    FREE_SPACE,           // Simple FSPL
    THREE_GPP_UMI,        // 3GPP TR 38.901 Urban Micro
    THREE_GPP_UMA,        // 3GPP TR 38.901 Urban Macro
    RAY_TRACING,          // Ray tracing with reflections
    HYBRID                // 3GPP + Ray tracing for multipath
};

/// Configuration for propagation models
struct PropagationConfig
{
    PropagationModel model{PropagationModel::THREE_GPP_UMI};
    double frequencyHz{3.5e9};
    double txPowerDbm{30.0};
    double txHeightM{10.0};
    double rxHeightM{1.5};
    double txAntennaGainDbi{0.0};
    double rxAntennaGainDbi{0.0};
    
    // 3GPP specific
    EnvironmentType environment{EnvironmentType::URBAN_MICRO};
    bool includeShadowing{true};
    double shadowingStdLos_dB{4.0};
    double shadowingStdNlos_dB{7.82};
    
    // Ray tracing specific
    int maxReflections{2};
    int maxDiffractions{1};
    int numRaysPerPoint{8};      // For stochastic ray tracing
    double maxRayDistance{500.0};
    
    // Material
    MaterialProperties defaultMaterial;
};

/// Base class for propagation models
class IPropagationModel
{
public:
    virtual ~IPropagationModel() = default;
    
    /// Calculate path loss between two points
    /// @param txPos Transmitter position
    /// @param rxPos Receiver position  
    /// @param obstacles List of obstacles in the environment
    /// @param config Propagation configuration
    /// @return Received power in dBm
    virtual double CalculateReceivedPower(
        const ignition::math::Vector3d& txPos,
        const ignition::math::Vector3d& rxPos,
        const std::vector<Obstacle>& obstacles,
        const PropagationConfig& config) = 0;
    
    /// Get model name
    virtual std::string GetName() const = 0;
    
    /// Check if position is in LOS
    static bool IsLineOfSight(
        const ignition::math::Vector3d& from,
        const ignition::math::Vector3d& to,
        const std::vector<Obstacle>& obstacles);
    
    /// Count wall penetrations
    static int CountWallPenetrations(
        const ignition::math::Vector3d& from,
        const ignition::math::Vector3d& to,
        const std::vector<Obstacle>& obstacles);
};

/// Free Space Path Loss model
class FreeSpaceModel : public IPropagationModel
{
public:
    double CalculateReceivedPower(
        const ignition::math::Vector3d& txPos,
        const ignition::math::Vector3d& rxPos,
        const std::vector<Obstacle>& obstacles,
        const PropagationConfig& config) override;
    
    std::string GetName() const override { return "Free Space (FSPL)"; }
    
    /// Calculate FSPL in dB
    static double FSPL_dB(double distanceM, double frequencyHz);
};

/// 3GPP TR 38.901 Urban Micro model
class ThreeGPPUMiModel : public IPropagationModel
{
public:
    double CalculateReceivedPower(
        const ignition::math::Vector3d& txPos,
        const ignition::math::Vector3d& rxPos,
        const std::vector<Obstacle>& obstacles,
        const PropagationConfig& config) override;
    
    std::string GetName() const override { return "3GPP UMi (TR 38.901)"; }
    
private:
    /// Calculate LOS path loss for UMi Street Canyon
    double CalculateLosPathLoss(
        double distance3D_m,
        double distance2D_m,
        double frequencyHz,
        double hBS_m,
        double hUT_m) const;
    
    /// Calculate NLOS path loss for UMi Street Canyon
    double CalculateNlosPathLoss(
        double distance3D_m,
        double distance2D_m,
        double frequencyHz,
        double hBS_m,
        double hUT_m) const;
    
    /// Calculate LOS probability
    double CalculateLosProbability(double distance2D_m) const;
    
    /// Calculate breakpoint distance
    double CalculateBreakpointDistance(
        double frequencyHz,
        double hBS_m,
        double hUT_m) const;
    
    std::mt19937 rng_{std::random_device{}()};
};

/// 3GPP TR 38.901 Urban Macro model
class ThreeGPPUMaModel : public IPropagationModel
{
public:
    double CalculateReceivedPower(
        const ignition::math::Vector3d& txPos,
        const ignition::math::Vector3d& rxPos,
        const std::vector<Obstacle>& obstacles,
        const PropagationConfig& config) override;
    
    std::string GetName() const override { return "3GPP UMa (TR 38.901)"; }
    
private:
    double CalculateLosPathLoss(
        double distance3D_m,
        double distance2D_m,
        double frequencyHz,
        double hBS_m,
        double hUT_m) const;
    
    double CalculateNlosPathLoss(
        double distance3D_m,
        double distance2D_m,
        double frequencyHz,
        double hBS_m,
        double hUT_m) const;
    
    double CalculateLosProbability(double distance2D_m, double hUT_m) const;
    
    std::mt19937 rng_{std::random_device{}()};
};

/// Ray Tracing propagation model
class RayTracingModel : public IPropagationModel
{
public:
    double CalculateReceivedPower(
        const ignition::math::Vector3d& txPos,
        const ignition::math::Vector3d& rxPos,
        const std::vector<Obstacle>& obstacles,
        const PropagationConfig& config) override;
    
    std::string GetName() const override { return "Ray Tracing"; }
    
private:
    /// Trace a single ray and find intersection
    RayHit TraceRay(
        const ignition::math::Vector3d& origin,
        const ignition::math::Vector3d& direction,
        double maxDistance,
        const std::vector<Obstacle>& obstacles) const;
    
    /// Find all valid paths between tx and rx
    std::vector<RayPath> FindPaths(
        const ignition::math::Vector3d& txPos,
        const ignition::math::Vector3d& rxPos,
        const std::vector<Obstacle>& obstacles,
        const PropagationConfig& config) const;
    
    /// Calculate path loss for a single ray path
    double CalculatePathLoss(
        const RayPath& path,
        double frequencyHz,
        const PropagationConfig& config) const;
    
    /// Calculate reflection point on a surface
    ignition::math::Vector3d CalculateReflectionPoint(
        const ignition::math::Vector3d& source,
        const ignition::math::Vector3d& target,
        const ignition::math::Vector3d& planePoint,
        const ignition::math::Vector3d& planeNormal) const;
    
    /// Reflect a direction vector off a surface
    ignition::math::Vector3d ReflectDirection(
        const ignition::math::Vector3d& incident,
        const ignition::math::Vector3d& normal) const;
    
    /// Calculate diffraction loss (knife-edge)
    double CalculateDiffractionLoss(
        const ignition::math::Vector3d& txPos,
        const ignition::math::Vector3d& rxPos,
        const ignition::math::Vector3d& edgePos,
        double frequencyHz) const;
    
    /// Combine multiple paths (power summation)
    double CombinePaths(
        const std::vector<RayPath>& paths,
        double frequencyHz) const;
};

/// Hybrid model combining 3GPP with ray tracing
class HybridModel : public IPropagationModel
{
public:
    double CalculateReceivedPower(
        const ignition::math::Vector3d& txPos,
        const ignition::math::Vector3d& rxPos,
        const std::vector<Obstacle>& obstacles,
        const PropagationConfig& config) override;
    
    std::string GetName() const override { return "Hybrid (3GPP + Ray Tracing)"; }
    
private:
    ThreeGPPUMiModel umiModel_;
    RayTracingModel rayModel_;
};

/// Factory for creating propagation models
class PropagationModelFactory
{
public:
    static std::unique_ptr<IPropagationModel> Create(PropagationModel type);
    static std::string GetModelName(PropagationModel type);
    static std::vector<PropagationModel> GetAvailableModels();
};

}  // namespace heatmap_plugin