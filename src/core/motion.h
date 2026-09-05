#pragma once

#include "core/trajectory.h"

#include <cstddef>
#include <filesystem>
#include <vector>

namespace camstudio {

struct RotationSample {
    double camPhase{};
    double pathPhase{};
    double alphaRad{};
    double betaRad{};
};

struct MotionProgram {
    std::vector<RotationSample> samples;
    double followerRadiusMm{};
    Vec3 pivotMm{};
};

struct CycleDiagnostics {
    double seamSegmentMm{};
    double medianSegmentMm{};
    double seamLengthRatio{};
    double seamTangentJumpDeg{};
    double maximumSpeedMmPerCycle{};
    double maximumAccelerationMmPerCycleSquared{};
};

[[nodiscard]] double mapCamPhaseToPathPhase(
    double camPhase,
    const std::vector<TimingKey>& timingKeys);
[[nodiscard]] Trajectory applyTimingMap(const Trajectory& trajectory);
[[nodiscard]] MotionProgram solveRotationRotation(
    const Trajectory& spatial,
    Vec3 pivotMm = {},
    double radialToleranceMm = 0.1);
[[nodiscard]] Trajectory makeRotationRotationPitchCurve(
    const MotionProgram& motion,
    Vec3 jointAtRestMm);
[[nodiscard]] CycleDiagnostics analyzeCycle(const Trajectory& trajectory);
void writeMotionCsv(const MotionProgram& motion, const std::filesystem::path& path);

}  // namespace camstudio
