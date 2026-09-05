#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace camstudio {

struct Vec3 {
    double x{};
    double y{};
    double z{};
};

enum class FollowerMotion {
    Rotation = 0,
    Translation = 1,
    RotationRotation = 2,
    TranslationTranslation = 3,
    RotationTranslation = 4,
};

struct TimingKey {
    double camPhase{};
    double pathPhase{};
};

struct Bounds {
    Vec3 minimum{};
    Vec3 maximum{};
};

struct Trajectory {
    std::vector<Vec3> points;
    FollowerMotion motion{FollowerMotion::RotationRotation};
    std::vector<TimingKey> timingKeys;
    std::string sourceName;

    [[nodiscard]] Bounds bounds() const;
    [[nodiscard]] double boundsDiagonal() const;
    [[nodiscard]] double closureGap() const;
    [[nodiscard]] Trajectory centeredAndScaled(double targetDiagonalMm) const;
    void validate() const;
};

struct SphericalFollowerAnalysis {
    double meanRadiusMm{};
    double minimumRadiusMm{};
    double maximumRadiusMm{};
    double maximumRadialErrorMm{};
    double rmsRadialErrorMm{};
    std::vector<std::pair<double, double>> anglesRad;

    [[nodiscard]] bool feasible(double toleranceMm) const;
};

[[nodiscard]] Trajectory readLegacyTrajectory(const std::filesystem::path& path);
[[nodiscard]] Trajectory readXyzCsv(const std::filesystem::path& path);
[[nodiscard]] Trajectory projectPlanarToSpherical(
    const Trajectory& planar,
    double followerRadiusMm,
    Vec3 pivotMm = {});
[[nodiscard]] SphericalFollowerAnalysis analyzeSphericalFollower(
    const Trajectory& trajectory,
    Vec3 pivotMm = {});
void writeCsv(const Trajectory& trajectory, const std::filesystem::path& path);
[[nodiscard]] std::string motionName(FollowerMotion motion);

}  // namespace camstudio
