#include "heatmap_plugin/PropagationModels.hh"
#include "heatmap_plugin/HeatmapPlugin.hh"
#include <algorithm>
#include <cmath>
#include <limits>

namespace heatmap_plugin
{

// Speed of light
constexpr double C = 299792458.0;

// ============================================================================
// Utility Functions
// ============================================================================

bool RayIntersectsAABB(
    const ignition::math::Vector3d& origin,
    const ignition::math::Vector3d& dir,
    double maxDist,
    const ignition::math::AxisAlignedBox& box,
    double& tMin,
    double& tMax)
{
    tMin = 0.0;
    tMax = maxDist;

    for (int i = 0; i < 3; ++i)
    {
        double o = (i == 0) ? origin.X() : (i == 1) ? origin.Y() : origin.Z();
        double d = (i == 0) ? dir.X() : (i == 1) ? dir.Y() : dir.Z();
        double bmin = (i == 0) ? box.Min().X() : (i == 1) ? box.Min().Y() : box.Min().Z();
        double bmax = (i == 0) ? box.Max().X() : (i == 1) ? box.Max().Y() : box.Max().Z();

        if (std::abs(d) < 1e-8)
        {
            if (o < bmin || o > bmax) return false;
        }
        else
        {
            double invD = 1.0 / d;
            double t1 = (bmin - o) * invD;
            double t2 = (bmax - o) * invD;
            if (t1 > t2) std::swap(t1, t2);
            tMin = std::max(tMin, t1);
            tMax = std::min(tMax, t2);
            if (tMin > tMax) return false;
        }
    }
    return tMax > 0 && tMin < maxDist;
}

ignition::math::Vector3d GetBoxNormal(
    const ignition::math::Vector3d& hitPoint,
    const ignition::math::AxisAlignedBox& box)
{
    const double eps = 0.001;
    if (std::abs(hitPoint.X() - box.Min().X()) < eps) return {-1, 0, 0};
    if (std::abs(hitPoint.X() - box.Max().X()) < eps) return {1, 0, 0};
    if (std::abs(hitPoint.Y() - box.Min().Y()) < eps) return {0, -1, 0};
    if (std::abs(hitPoint.Y() - box.Max().Y()) < eps) return {0, 1, 0};
    if (std::abs(hitPoint.Z() - box.Min().Z()) < eps) return {0, 0, -1};
    if (std::abs(hitPoint.Z() - box.Max().Z()) < eps) return {0, 0, 1};
    return {0, 0, 1};
}

// ============================================================================
// IPropagationModel Base Class
// ============================================================================

bool IPropagationModel::IsLineOfSight(
    const ignition::math::Vector3d& from,
    const ignition::math::Vector3d& to,
    const std::vector<Obstacle>& obstacles)
{
    auto dir = to - from;
    double length = dir.Length();
    if (length < 0.01) return true;
    dir /= length;

    for (const auto& obs : obstacles)
    {
        double tMin, tMax;
        if (RayIntersectsAABB(from, dir, length, obs.bbox, tMin, tMax))
        {
            if (tMin > 0.01 && tMin < length - 0.01)
                return false;
        }
    }
    return true;
}

int IPropagationModel::CountWallPenetrations(
    const ignition::math::Vector3d& from,
    const ignition::math::Vector3d& to,
    const std::vector<Obstacle>& obstacles)
{
    auto dir = to - from;
    double length = dir.Length();
    if (length < 0.01) return 0;
    dir /= length;

    int count = 0;
    for (const auto& obs : obstacles)
    {
        double tMin, tMax;
        if (RayIntersectsAABB(from, dir, length, obs.bbox, tMin, tMax))
        {
            if (tMin > 0.01 && tMin < length - 0.01)
                ++count;
        }
    }
    return count;
}

// ============================================================================
// Free Space Model
// ============================================================================

double FreeSpaceModel::FSPL_dB(double distanceM, double frequencyHz)
{
    if (distanceM < 0.1) distanceM = 0.1;
    // FSPL = 20*log10(d) + 20*log10(f) + 20*log10(4*pi/c)
    // Simplified: 20*log10(d) + 20*log10(f) - 147.55
    return 20.0 * std::log10(distanceM) + 20.0 * std::log10(frequencyHz) - 147.55;
}

double FreeSpaceModel::CalculateReceivedPower(
    const ignition::math::Vector3d& txPos,
    const ignition::math::Vector3d& rxPos,
    const std::vector<Obstacle>& obstacles,
    const PropagationConfig& config)
{
    double distance = txPos.Distance(rxPos);
    double pathLoss = FSPL_dB(distance, config.frequencyHz);
    
    // Add wall penetration loss
    int walls = CountWallPenetrations(txPos, rxPos, obstacles);
    pathLoss += walls * config.defaultMaterial.penetrationLoss_dB;
    
    // Check if inside obstacle
    for (const auto& obs : obstacles)
    {
        if (obs.bbox.Contains(rxPos))
            return -150.0;  // No signal inside
    }
    
    double rxPower = config.txPowerDbm + config.txAntennaGainDbi + 
                     config.rxAntennaGainDbi - pathLoss;
    
    return std::clamp(rxPower, -150.0, config.txPowerDbm);
}

// ============================================================================
// 3GPP UMi Model (TR 38.901)
// ============================================================================

double ThreeGPPUMiModel::CalculateBreakpointDistance(
    double frequencyHz,
    double hBS_m,
    double hUT_m) const
{
    // Effective antenna heights for UMi Street Canyon
    double hE = 1.0;  // Effective environment height
    double hBS_eff = hBS_m - hE;
    double hUT_eff = hUT_m - hE;
    
    // Breakpoint distance
    return 4.0 * hBS_eff * hUT_eff * frequencyHz / C;
}

double ThreeGPPUMiModel::CalculateLosPathLoss(
    double distance3D_m,
    double distance2D_m,
    double frequencyHz,
    double hBS_m,
    double hUT_m) const
{
    double fc_GHz = frequencyHz / 1e9;
    double dBP = CalculateBreakpointDistance(frequencyHz, hBS_m, hUT_m);
    
    double PL;
    if (distance2D_m <= dBP)
    {
        // PL1 for d2D <= d'BP
        PL = 32.4 + 21.0 * std::log10(distance3D_m) + 20.0 * std::log10(fc_GHz);
    }
    else
    {
        // PL2 for d2D > d'BP
        PL = 32.4 + 40.0 * std::log10(distance3D_m) + 20.0 * std::log10(fc_GHz)
             - 9.5 * std::log10(dBP * dBP + (hBS_m - hUT_m) * (hBS_m - hUT_m));
    }
    
    return PL;
}

double ThreeGPPUMiModel::CalculateNlosPathLoss(
    double distance3D_m,
    double distance2D_m,
    double frequencyHz,
    double hBS_m,
    double hUT_m) const
{
    double fc_GHz = frequencyHz / 1e9;
    
    // NLOS path loss for UMi Street Canyon
    double PL_NLOS = 35.3 * std::log10(distance3D_m) + 22.4 
                    + 21.3 * std::log10(fc_GHz)
                    - 0.3 * (hUT_m - 1.5);
    
    // LOS path loss (for max comparison)
    double PL_LOS = CalculateLosPathLoss(distance3D_m, distance2D_m, 
                                         frequencyHz, hBS_m, hUT_m);
    
    // NLOS path loss is max of NLOS formula and LOS path loss
    return std::max(PL_NLOS, PL_LOS);
}

double ThreeGPPUMiModel::CalculateLosProbability(double distance2D_m) const
{
    // LOS probability for UMi Street Canyon
    if (distance2D_m <= 18.0)
        return 1.0;
    else
        return (18.0 / distance2D_m) + std::exp(-distance2D_m / 36.0) * (1.0 - 18.0 / distance2D_m);
}

double ThreeGPPUMiModel::CalculateReceivedPower(
    const ignition::math::Vector3d& txPos,
    const ignition::math::Vector3d& rxPos,
    const std::vector<Obstacle>& obstacles,
    const PropagationConfig& config)
{
    // Check if inside obstacle
    for (const auto& obs : obstacles)
    {
        if (obs.bbox.Contains(rxPos))
            return -150.0;
    }
    
    double dx = rxPos.X() - txPos.X();
    double dy = rxPos.Y() - txPos.Y();
    double dz = rxPos.Z() - txPos.Z();
    
    double distance2D = std::sqrt(dx * dx + dy * dy);
    double distance3D = std::sqrt(dx * dx + dy * dy + dz * dz);
    
    if (distance3D < 1.0) distance3D = 1.0;
    if (distance2D < 1.0) distance2D = 1.0;
    
    // Determine LOS/NLOS based on actual geometry
    bool isLOS = IsLineOfSight(txPos, rxPos, obstacles);
    
    double pathLoss;
    double shadowingStd;
    
    if (isLOS)
    {
        pathLoss = CalculateLosPathLoss(distance3D, distance2D, 
                                        config.frequencyHz,
                                        config.txHeightM, config.rxHeightM);
        shadowingStd = config.shadowingStdLos_dB;
    }
    else
    {
        pathLoss = CalculateNlosPathLoss(distance3D, distance2D,
                                         config.frequencyHz,
                                         config.txHeightM, config.rxHeightM);
        shadowingStd = config.shadowingStdNlos_dB;
        
        // Additional penetration loss for walls
        int walls = CountWallPenetrations(txPos, rxPos, obstacles);
        if (walls > 0)
        {
            pathLoss += walls * config.defaultMaterial.penetrationLoss_dB;
        }
    }
    
    // Add shadowing (log-normal)
    double shadowing = 0.0;
    if (config.includeShadowing)
    {
        std::normal_distribution<double> dist(0.0, shadowingStd);
        shadowing = dist(rng_);
    }
    
    double rxPower = config.txPowerDbm + config.txAntennaGainDbi +
                     config.rxAntennaGainDbi - pathLoss + shadowing;
    
    return std::clamp(rxPower, -150.0, config.txPowerDbm);
}

// ============================================================================
// 3GPP UMa Model (TR 38.901)
// ============================================================================

double ThreeGPPUMaModel::CalculateLosPathLoss(
    double distance3D_m,
    double distance2D_m,
    double frequencyHz,
    double hBS_m,
    double hUT_m) const
{
    double fc_GHz = frequencyHz / 1e9;
    
    // Breakpoint distance for UMa
    double hE = 1.0;
    double hBS_eff = hBS_m - hE;
    double hUT_eff = hUT_m - hE;
    double dBP = 4.0 * hBS_eff * hUT_eff * frequencyHz / C;
    
    double PL;
    if (distance2D_m <= dBP)
    {
        PL = 28.0 + 22.0 * std::log10(distance3D_m) + 20.0 * std::log10(fc_GHz);
    }
    else
    {
        PL = 28.0 + 40.0 * std::log10(distance3D_m) + 20.0 * std::log10(fc_GHz)
             - 9.0 * std::log10(dBP * dBP + (hBS_m - hUT_m) * (hBS_m - hUT_m));
    }
    
    return PL;
}

double ThreeGPPUMaModel::CalculateNlosPathLoss(
    double distance3D_m,
    double distance2D_m,
    double frequencyHz,
    double hBS_m,
    double hUT_m) const
{
    double fc_GHz = frequencyHz / 1e9;
    
    double PL_NLOS = 13.54 + 39.08 * std::log10(distance3D_m)
                    + 20.0 * std::log10(fc_GHz)
                    - 0.6 * (hUT_m - 1.5);
    
    double PL_LOS = CalculateLosPathLoss(distance3D_m, distance2D_m,
                                         frequencyHz, hBS_m, hUT_m);
    
    return std::max(PL_NLOS, PL_LOS);
}

double ThreeGPPUMaModel::CalculateLosProbability(double distance2D_m, double hUT_m) const
{
    if (distance2D_m <= 18.0)
        return 1.0;
    
    double C_val = 0.0;
    if (hUT_m > 13.0)
    {
        C_val = std::pow((hUT_m - 13.0) / 10.0, 1.5);
    }
    
    return (18.0 / distance2D_m + std::exp(-distance2D_m / 63.0) * (1.0 - 18.0 / distance2D_m))
           * (1.0 + C_val * 5.0 / 4.0 * std::pow(distance2D_m / 100.0, 3.0) 
              * std::exp(-distance2D_m / 150.0));
}

double ThreeGPPUMaModel::CalculateReceivedPower(
    const ignition::math::Vector3d& txPos,
    const ignition::math::Vector3d& rxPos,
    const std::vector<Obstacle>& obstacles,
    const PropagationConfig& config)
{
    for (const auto& obs : obstacles)
    {
        if (obs.bbox.Contains(rxPos))
            return -150.0;
    }
    
    double dx = rxPos.X() - txPos.X();
    double dy = rxPos.Y() - txPos.Y();
    double dz = rxPos.Z() - txPos.Z();
    
    double distance2D = std::sqrt(dx * dx + dy * dy);
    double distance3D = std::sqrt(dx * dx + dy * dy + dz * dz);
    
    if (distance3D < 1.0) distance3D = 1.0;
    if (distance2D < 1.0) distance2D = 1.0;
    
    bool isLOS = IsLineOfSight(txPos, rxPos, obstacles);
    
    double pathLoss;
    double shadowingStd;
    
    if (isLOS)
    {
        pathLoss = CalculateLosPathLoss(distance3D, distance2D,
                                        config.frequencyHz,
                                        config.txHeightM, config.rxHeightM);
        shadowingStd = 4.0;  // UMa LOS
    }
    else
    {
        pathLoss = CalculateNlosPathLoss(distance3D, distance2D,
                                         config.frequencyHz,
                                         config.txHeightM, config.rxHeightM);
        shadowingStd = 6.0;  // UMa NLOS
        
        int walls = CountWallPenetrations(txPos, rxPos, obstacles);
        pathLoss += walls * config.defaultMaterial.penetrationLoss_dB;
    }
    
    double shadowing = 0.0;
    if (config.includeShadowing)
    {
        std::normal_distribution<double> dist(0.0, shadowingStd);
        shadowing = dist(rng_);
    }
    
    double rxPower = config.txPowerDbm + config.txAntennaGainDbi +
                     config.rxAntennaGainDbi - pathLoss + shadowing;
    
    return std::clamp(rxPower, -150.0, config.txPowerDbm);
}

// ============================================================================
// Ray Tracing Model
// ============================================================================

RayHit RayTracingModel::TraceRay(
    const ignition::math::Vector3d& origin,
    const ignition::math::Vector3d& direction,
    double maxDistance,
    const std::vector<Obstacle>& obstacles) const
{
    RayHit closest;
    closest.distance = maxDistance;
    
    for (const auto& obs : obstacles)
    {
        double tMin, tMax;
        if (RayIntersectsAABB(origin, direction, maxDistance, obs.bbox, tMin, tMax))
        {
            if (tMin > 0.001 && tMin < closest.distance)
            {
                closest.hit = true;
                closest.distance = tMin;
                closest.point = origin + direction * tMin;
                closest.normal = GetBoxNormal(closest.point, obs.bbox);
                closest.obstacle = &obs;
            }
        }
    }
    
    return closest;
}

ignition::math::Vector3d RayTracingModel::ReflectDirection(
    const ignition::math::Vector3d& incident,
    const ignition::math::Vector3d& normal) const
{
    return incident - normal * 2.0 * incident.Dot(normal);
}

ignition::math::Vector3d RayTracingModel::CalculateReflectionPoint(
    const ignition::math::Vector3d& source,
    const ignition::math::Vector3d& target,
    const ignition::math::Vector3d& planePoint,
    const ignition::math::Vector3d& planeNormal) const
{
    // Mirror the source across the plane
    double d = (source - planePoint).Dot(planeNormal);
    auto mirroredSource = source - planeNormal * 2.0 * d;
    
    // Find intersection of line from mirrored source to target with plane
    auto dir = target - mirroredSource;
    double denom = dir.Dot(planeNormal);
    
    if (std::abs(denom) < 1e-8)
        return planePoint;  // Parallel to plane
    
    double t = (planePoint - mirroredSource).Dot(planeNormal) / denom;
    return mirroredSource + dir * t;
}

double RayTracingModel::CalculateDiffractionLoss(
    const ignition::math::Vector3d& txPos,
    const ignition::math::Vector3d& rxPos,
    const ignition::math::Vector3d& edgePos,
    double frequencyHz) const
{
    // Knife-edge diffraction using Fresnel parameter
    double d1 = txPos.Distance(edgePos);
    double d2 = edgePos.Distance(rxPos);
    double dTotal = txPos.Distance(rxPos);
    
    // Height of obstruction above LOS
    auto losDir = (rxPos - txPos).Normalized();
    double tEdge = (edgePos - txPos).Dot(losDir);
    auto losPoint = txPos + losDir * tEdge;
    double h = (edgePos - losPoint).Length();
    
    // Fresnel parameter
    double lambda = C / frequencyHz;
    double v = h * std::sqrt(2.0 / lambda * (1.0 / d1 + 1.0 / d2));
    
    // Approximate diffraction loss (dB)
    double loss;
    if (v < -0.78)
        loss = 0.0;
    else if (v < 0)
        loss = 6.02 + 9.11 * v - 1.27 * v * v;
    else if (v < 2.4)
        loss = 6.02 + 9.0 * v + 1.65 * v * v;
    else
        loss = 13.0 + 20.0 * std::log10(v);
    
    return std::max(0.0, loss);
}

std::vector<RayPath> RayTracingModel::FindPaths(
    const ignition::math::Vector3d& txPos,
    const ignition::math::Vector3d& rxPos,
    const std::vector<Obstacle>& obstacles,
    const PropagationConfig& config) const
{
    std::vector<RayPath> paths;
    double directDist = txPos.Distance(rxPos);
    
    // 1. Direct path (LOS or through obstacles)
    {
        RayPath direct;
        direct.type = RayPath::Type::LOS;
        direct.totalDistance = directDist;
        direct.waypoints = {txPos, rxPos};
        direct.pathLoss_dB = FreeSpaceModel::FSPL_dB(directDist, config.frequencyHz);
        
        // Check for obstructions
        auto dir = (rxPos - txPos).Normalized();
        auto hit = TraceRay(txPos, dir, directDist, obstacles);
        
        if (hit.hit && hit.distance < directDist - 0.1)
        {
            // Add penetration loss
            int walls = IPropagationModel::CountWallPenetrations(txPos, rxPos, obstacles);
            direct.pathLoss_dB += walls * config.defaultMaterial.penetrationLoss_dB;
        }
        
        paths.push_back(direct);
    }
    
    // 2. Single-bounce reflections
    if (config.maxReflections >= 1)
    {
        for (const auto& obs : obstacles)
        {
            // Try each face of the box as a potential reflector
            std::vector<std::pair<ignition::math::Vector3d, ignition::math::Vector3d>> faces = {
                {obs.position + ignition::math::Vector3d(obs.size.X()/2, 0, 0), {1, 0, 0}},
                {obs.position - ignition::math::Vector3d(obs.size.X()/2, 0, 0), {-1, 0, 0}},
                {obs.position + ignition::math::Vector3d(0, obs.size.Y()/2, 0), {0, 1, 0}},
                {obs.position - ignition::math::Vector3d(0, obs.size.Y()/2, 0), {0, -1, 0}},
            };
            
            for (const auto& [planePoint, planeNormal] : faces)
            {
                auto reflPoint = CalculateReflectionPoint(txPos, rxPos, planePoint, planeNormal);
                
                // Check if reflection point is on the building face
                bool onFace = true;
                double margin = 0.1;
                if (std::abs(planeNormal.X()) > 0.5)
                {
                    onFace = reflPoint.Y() >= obs.bbox.Min().Y() - margin &&
                             reflPoint.Y() <= obs.bbox.Max().Y() + margin &&
                             reflPoint.Z() >= obs.bbox.Min().Z() - margin &&
                             reflPoint.Z() <= obs.bbox.Max().Z() + margin;
                }
                else if (std::abs(planeNormal.Y()) > 0.5)
                {
                    onFace = reflPoint.X() >= obs.bbox.Min().X() - margin &&
                             reflPoint.X() <= obs.bbox.Max().X() + margin &&
                             reflPoint.Z() >= obs.bbox.Min().Z() - margin &&
                             reflPoint.Z() <= obs.bbox.Max().Z() + margin;
                }
                
                if (!onFace) continue;
                
                // Check if both segments are unobstructed (or at least reach the surface)
                double d1 = txPos.Distance(reflPoint);
                double d2 = reflPoint.Distance(rxPos);
                
                if (d1 < 0.5 || d2 < 0.5) continue;
                
                RayPath reflected;
                reflected.type = RayPath::Type::REFLECTED;
                reflected.totalDistance = d1 + d2;
                reflected.numReflections = 1;
                reflected.waypoints = {txPos, reflPoint, rxPos};
                
                // Path loss = FSPL for total distance + reflection loss
                reflected.pathLoss_dB = FreeSpaceModel::FSPL_dB(reflected.totalDistance, config.frequencyHz);
                reflected.pathLoss_dB += 10.0 * std::log10(1.0 / (config.defaultMaterial.reflectionCoeff * config.defaultMaterial.reflectionCoeff));
                
                // Only add if path loss is reasonable
                if (reflected.pathLoss_dB < 200.0)
                    paths.push_back(reflected);
            }
        }
    }
    
    return paths;
}

double RayTracingModel::CombinePaths(
    const std::vector<RayPath>& paths,
    double frequencyHz) const
{
    if (paths.empty()) return -150.0;
    
    // Convert path losses to linear power and sum (incoherent addition)
    double totalPowerLinear = 0.0;
    
    for (const auto& path : paths)
    {
        double powerLinear = std::pow(10.0, -path.pathLoss_dB / 10.0);
        totalPowerLinear += powerLinear;
    }
    
    // Convert back to dB
    if (totalPowerLinear > 0)
        return -10.0 * std::log10(totalPowerLinear);
    else
        return 200.0;  // Very high loss
}

double RayTracingModel::CalculateReceivedPower(
    const ignition::math::Vector3d& txPos,
    const ignition::math::Vector3d& rxPos,
    const std::vector<Obstacle>& obstacles,
    const PropagationConfig& config)
{
    // Check if inside obstacle
    for (const auto& obs : obstacles)
    {
        if (obs.bbox.Contains(rxPos))
            return -150.0;
    }
    
    // Find all paths
    auto paths = FindPaths(txPos, rxPos, obstacles, config);
    
    // Combine paths
    double combinedPathLoss = CombinePaths(paths, config.frequencyHz);
    
    double rxPower = config.txPowerDbm + config.txAntennaGainDbi +
                     config.rxAntennaGainDbi - combinedPathLoss;
    
    return std::clamp(rxPower, -150.0, config.txPowerDbm);
}

// ============================================================================
// Hybrid Model
// ============================================================================

double HybridModel::CalculateReceivedPower(
    const ignition::math::Vector3d& txPos,
    const ignition::math::Vector3d& rxPos,
    const std::vector<Obstacle>& obstacles,
    const PropagationConfig& config)
{
    // Use 3GPP for base path loss, ray tracing for multipath enhancement
    double umiPower = umiModel_.CalculateReceivedPower(txPos, rxPos, obstacles, config);
    
    // Only use ray tracing enhancement in NLOS conditions
    if (!IPropagationModel::IsLineOfSight(txPos, rxPos, obstacles))
    {
        double rayPower = rayModel_.CalculateReceivedPower(txPos, rxPos, obstacles, config);
        
        // Take the better of the two (ray tracing might find reflection paths)
        return std::max(umiPower, rayPower);
    }
    
    return umiPower;
}

// ============================================================================
// Factory
// ============================================================================

std::unique_ptr<IPropagationModel> PropagationModelFactory::Create(PropagationModel type)
{
    switch (type)
    {
        case PropagationModel::FREE_SPACE:
            return std::make_unique<FreeSpaceModel>();
        case PropagationModel::THREE_GPP_UMI:
            return std::make_unique<ThreeGPPUMiModel>();
        case PropagationModel::THREE_GPP_UMA:
            return std::make_unique<ThreeGPPUMaModel>();
        case PropagationModel::RAY_TRACING:
            return std::make_unique<RayTracingModel>();
        case PropagationModel::HYBRID:
            return std::make_unique<HybridModel>();
        default:
            return std::make_unique<FreeSpaceModel>();
    }
}

std::string PropagationModelFactory::GetModelName(PropagationModel type)
{
    switch (type)
    {
        case PropagationModel::FREE_SPACE: return "Free Space (FSPL)";
        case PropagationModel::THREE_GPP_UMI: return "3GPP UMi (TR 38.901)";
        case PropagationModel::THREE_GPP_UMA: return "3GPP UMa (TR 38.901)";
        case PropagationModel::RAY_TRACING: return "Ray Tracing";
        case PropagationModel::HYBRID: return "Hybrid (3GPP + Ray Tracing)";
        default: return "Unknown";
    }
}

std::vector<PropagationModel> PropagationModelFactory::GetAvailableModels()
{
    return {
        PropagationModel::FREE_SPACE,
        PropagationModel::THREE_GPP_UMI,
        PropagationModel::THREE_GPP_UMA,
        PropagationModel::RAY_TRACING,
        PropagationModel::HYBRID
    };
}

}  // namespace heatmap_plugin