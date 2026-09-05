#include "core/trajectory.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace camstudio {
namespace {

bool finite(const Vec3& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

double distance(const Vec3& a, const Vec3& b) {
    const auto dx = a.x - b.x;
    const auto dy = a.y - b.y;
    const auto dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::vector<std::string> splitCsvLine(const std::string& line) {
    std::vector<std::string> columns;
    std::stringstream stream(line);
    std::string column;
    while (std::getline(stream, column, ',')) {
        columns.push_back(column);
    }
    return columns;
}

double parseNumber(const std::string& text, std::size_t lineNumber) {
    std::size_t consumed{};
    try {
        const auto value = std::stod(text, &consumed);
        if (consumed != text.size() || !std::isfinite(value)) {
            throw std::runtime_error("invalid");
        }
        return value;
    } catch (...) {
        throw std::runtime_error("Invalid number on CSV line " + std::to_string(lineNumber));
    }
}

FollowerMotion decodeMotion(int value) {
    if (value < 0 || value > 4) {
        throw std::runtime_error("Unsupported follower motion type: " + std::to_string(value));
    }
    return static_cast<FollowerMotion>(value);
}

}  // namespace

bool SphericalFollowerAnalysis::feasible(double toleranceMm) const {
    return std::isfinite(toleranceMm) && toleranceMm >= 0.0 &&
           maximumRadialErrorMm <= toleranceMm;
}

Bounds Trajectory::bounds() const {
    if (points.empty()) {
        throw std::runtime_error("Cannot calculate bounds for an empty trajectory");
    }

    Bounds result{points.front(), points.front()};
    for (const auto& point : points) {
        result.minimum.x = std::min(result.minimum.x, point.x);
        result.minimum.y = std::min(result.minimum.y, point.y);
        result.minimum.z = std::min(result.minimum.z, point.z);
        result.maximum.x = std::max(result.maximum.x, point.x);
        result.maximum.y = std::max(result.maximum.y, point.y);
        result.maximum.z = std::max(result.maximum.z, point.z);
    }
    return result;
}

double Trajectory::boundsDiagonal() const {
    const auto box = bounds();
    return distance(box.minimum, box.maximum);
}

double Trajectory::closureGap() const {
    return points.size() < 2 ? 0.0 : distance(points.front(), points.back());
}

Trajectory Trajectory::centeredAndScaled(double targetDiagonalMm) const {
    validate();
    if (!std::isfinite(targetDiagonalMm) || targetDiagonalMm <= 0.0) {
        throw std::runtime_error("Target size must be a positive number of millimetres");
    }

    const auto box = bounds();
    const Vec3 center{
        (box.minimum.x + box.maximum.x) * 0.5,
        (box.minimum.y + box.maximum.y) * 0.5,
        (box.minimum.z + box.maximum.z) * 0.5,
    };
    const auto diagonal = boundsDiagonal();
    if (diagonal <= std::numeric_limits<double>::epsilon()) {
        throw std::runtime_error("Trajectory has no measurable size");
    }

    const auto scale = targetDiagonalMm / diagonal;
    auto result = *this;
    for (auto& point : result.points) {
        point.x = (point.x - center.x) * scale;
        point.y = (point.y - center.y) * scale;
        point.z = (point.z - center.z) * scale;
    }
    return result;
}

void Trajectory::validate() const {
    if (points.size() < 3) {
        throw std::runtime_error("A trajectory needs at least three points");
    }
    if (!std::all_of(points.begin(), points.end(), finite)) {
        throw std::runtime_error("Trajectory contains a non-finite coordinate");
    }

    double previousCamPhase = -1.0;
    double previousPathPhase = -1.0;
    for (const auto& key : timingKeys) {
        if (!std::isfinite(key.camPhase) || !std::isfinite(key.pathPhase) ||
            key.camPhase < 0.0 || key.camPhase > 1.0 ||
            key.pathPhase < 0.0 || key.pathPhase > 1.0) {
            throw std::runtime_error("Timing keys must use finite phases between 0 and 1");
        }
        if (key.camPhase <= previousCamPhase || key.pathPhase <= previousPathPhase) {
            throw std::runtime_error("Timing keys must increase monotonically");
        }
        previousCamPhase = key.camPhase;
        previousPathPhase = key.pathPhase;
    }
}

Trajectory readLegacyTrajectory(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Cannot open trajectory: " + path.string());
    }

    std::size_t pointCount{};
    if (!(input >> pointCount) || pointCount < 3 || pointCount > 1'000'000) {
        throw std::runtime_error("Invalid legacy point count in: " + path.string());
    }

    Trajectory result;
    result.sourceName = path.filename().string();
    result.points.reserve(pointCount);
    for (std::size_t index = 0; index < pointCount; ++index) {
        Vec3 point{};
        if (!(input >> point.x >> point.y)) {
            throw std::runtime_error("Missing point " + std::to_string(index) + " in: " + path.string());
        }
        result.points.push_back(point);
    }

    int motion{};
    if (!(input >> motion)) {
        throw std::runtime_error("Missing follower motion type in: " + path.string());
    }
    result.motion = decodeMotion(motion);

    std::size_t timingCount{};
    if (!(input >> timingCount)) {
        throw std::runtime_error("Missing timing-key count in: " + path.string());
    }
    result.timingKeys.reserve(timingCount);
    for (std::size_t index = 0; index < timingCount; ++index) {
        TimingKey key{};
        if (!(input >> key.camPhase >> key.pathPhase)) {
            throw std::runtime_error("Missing timing key " + std::to_string(index) + " in: " + path.string());
        }
        result.timingKeys.push_back(key);
    }

    std::string trailing;
    if (input >> trailing) {
        throw std::runtime_error("Unexpected trailing data in: " + path.string());
    }

    result.validate();
    return result;
}

Trajectory readXyzCsv(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Cannot open XYZ trajectory: " + path.string());
    }

    Trajectory result;
    result.sourceName = path.filename().string();
    result.motion = FollowerMotion::RotationRotation;

    std::string line;
    std::size_t lineNumber{};
    bool firstDataLine = true;
    while (std::getline(input, line)) {
        ++lineNumber;
        if (line.empty() || line.front() == '#') {
            continue;
        }

        const auto columns = splitCsvLine(line);
        if (firstDataLine) {
            firstDataLine = false;
            const bool header = line.find("x_mm") != std::string::npos ||
                                line.find("y_mm") != std::string::npos ||
                                line.find("z_mm") != std::string::npos;
            if (header) {
                continue;
            }
        }

        if (columns.size() != 3 && columns.size() != 4) {
            throw std::runtime_error("CSV line " + std::to_string(lineNumber) +
                                     " must contain x,y,z or index,x,y,z");
        }
        const auto offset = columns.size() == 4 ? 1U : 0U;
        result.points.push_back({
            parseNumber(columns[offset], lineNumber),
            parseNumber(columns[offset + 1], lineNumber),
            parseNumber(columns[offset + 2], lineNumber),
        });
    }

    result.validate();
    return result;
}

Trajectory projectPlanarToSpherical(
    const Trajectory& planar,
    double followerRadiusMm,
    Vec3 pivotMm) {
    planar.validate();
    if (!std::isfinite(followerRadiusMm) || followerRadiusMm <= 0.0) {
        throw std::runtime_error("Follower radius must be positive");
    }

    auto result = planar;
    result.motion = FollowerMotion::RotationRotation;
    const auto radiusSquared = followerRadiusMm * followerRadiusMm;
    for (auto& point : result.points) {
        const auto transverseSquared = point.x * point.x + point.y * point.y;
        if (transverseSquared > radiusSquared) {
            throw std::runtime_error("Planar trajectory exceeds the follower radius");
        }
        const auto legacyX = -std::sqrt(std::max(0.0, radiusSquared - transverseSquared));
        const auto legacyY = point.y;
        const auto legacyZ = point.x;
        point = {pivotMm.x + legacyX, pivotMm.y + legacyY, pivotMm.z + legacyZ};
    }
    return result;
}

SphericalFollowerAnalysis analyzeSphericalFollower(
    const Trajectory& trajectory,
    Vec3 pivotMm) {
    trajectory.validate();
    SphericalFollowerAnalysis result;
    result.minimumRadiusMm = std::numeric_limits<double>::infinity();
    result.maximumRadiusMm = 0.0;

    std::vector<double> radii;
    radii.reserve(trajectory.points.size());
    result.anglesRad.reserve(trajectory.points.size());
    for (const auto& point : trajectory.points) {
        const Vec3 relative{point.x - pivotMm.x, point.y - pivotMm.y, point.z - pivotMm.z};
        const auto radius = distance(relative, {});
        if (radius <= std::numeric_limits<double>::epsilon()) {
            throw std::runtime_error("A target point coincides with the follower pivot");
        }
        radii.push_back(radius);
        result.meanRadiusMm += radius;
        result.minimumRadiusMm = std::min(result.minimumRadiusMm, radius);
        result.maximumRadiusMm = std::max(result.maximumRadiusMm, radius);

        const auto beta = -std::asin(std::clamp(relative.y / radius, -1.0, 1.0));
        const auto alpha = std::atan2(relative.z, -relative.x);
        result.anglesRad.emplace_back(alpha, beta);
    }
    result.meanRadiusMm /= static_cast<double>(radii.size());

    double squaredError{};
    for (const auto radius : radii) {
        const auto error = std::abs(radius - result.meanRadiusMm);
        result.maximumRadialErrorMm = std::max(result.maximumRadialErrorMm, error);
        squaredError += error * error;
    }
    result.rmsRadialErrorMm = std::sqrt(squaredError / static_cast<double>(radii.size()));
    return result;
}

void writeCsv(const Trajectory& trajectory, const std::filesystem::path& path) {
    trajectory.validate();
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("Cannot write CSV: " + path.string());
    }
    output << "index,x_mm,y_mm,z_mm\n" << std::setprecision(12);
    for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
        const auto& point = trajectory.points[index];
        output << index << ',' << point.x << ',' << point.y << ',' << point.z << '\n';
    }
}

std::string motionName(FollowerMotion motion) {
    switch (motion) {
        case FollowerMotion::Rotation: return "rotation";
        case FollowerMotion::Translation: return "translation";
        case FollowerMotion::RotationRotation: return "rotation-rotation";
        case FollowerMotion::TranslationTranslation: return "translation-translation";
        case FollowerMotion::RotationTranslation: return "rotation-translation";
    }
    return "unknown";
}

}  // namespace camstudio
