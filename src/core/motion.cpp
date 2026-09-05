#include "core/motion.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace camstudio {
namespace {

Vec3 add(const Vec3& a, const Vec3& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 subtract(const Vec3& a, const Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 multiply(const Vec3& value, double scale) {
    return {value.x * scale, value.y * scale, value.z * scale};
}

double dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

double length(const Vec3& value) {
    return std::sqrt(dot(value, value));
}

Vec3 rotateZ(const Vec3& point, double angle) {
    const auto cosine = std::cos(angle);
    const auto sine = std::sin(angle);
    return {
        cosine * point.x - sine * point.y,
        sine * point.x + cosine * point.y,
        point.z,
    };
}

Vec3 rotateY(const Vec3& point, double angle) {
    const auto cosine = std::cos(angle);
    const auto sine = std::sin(angle);
    return {
        cosine * point.x + sine * point.z,
        point.y,
        -sine * point.x + cosine * point.z,
    };
}

double unwrap(double previous, double current) {
    constexpr auto fullTurn = 2.0 * std::numbers::pi;
    while (current - previous > std::numbers::pi) {
        current -= fullTurn;
    }
    while (current - previous < -std::numbers::pi) {
        current += fullTurn;
    }
    return current;
}

Vec3 interpolateClosed(const std::vector<Vec3>& points, double phase) {
    const auto count = points.size();
    const auto wrapped = phase - std::floor(phase);
    const auto coordinate = wrapped * static_cast<double>(count);
    const auto lower = static_cast<std::size_t>(std::floor(coordinate)) % count;
    const auto upper = (lower + 1) % count;
    const auto fraction = coordinate - std::floor(coordinate);
    return add(multiply(points[lower], 1.0 - fraction), multiply(points[upper], fraction));
}

double angleBetweenDeg(const Vec3& a, const Vec3& b) {
    const auto denominator = length(a) * length(b);
    if (denominator <= std::numeric_limits<double>::epsilon()) {
        return 0.0;
    }
    const auto cosine = std::clamp(dot(a, b) / denominator, -1.0, 1.0);
    return std::acos(cosine) * 180.0 / std::numbers::pi;
}

}  // namespace

double mapCamPhaseToPathPhase(
    double camPhase,
    const std::vector<TimingKey>& timingKeys) {
    if (!std::isfinite(camPhase) || camPhase < 0.0 || camPhase > 1.0) {
        throw std::runtime_error("Cam phase must be between 0 and 1");
    }

    std::vector<TimingKey> keys;
    keys.reserve(timingKeys.size() + 2);
    keys.push_back({0.0, 0.0});
    for (const auto& key : timingKeys) {
        if (key.camPhase <= 0.0 || key.camPhase >= 1.0 ||
            key.pathPhase <= 0.0 || key.pathPhase >= 1.0) {
            throw std::runtime_error("Interior timing keys must lie strictly between 0 and 1");
        }
        keys.push_back(key);
    }
    keys.push_back({1.0, 1.0});

    for (std::size_t index = 1; index < keys.size(); ++index) {
        if (keys[index].camPhase <= keys[index - 1].camPhase ||
            keys[index].pathPhase <= keys[index - 1].pathPhase) {
            throw std::runtime_error("Timing keys must increase monotonically");
        }
        if (camPhase <= keys[index].camPhase) {
            const auto& left = keys[index - 1];
            const auto& right = keys[index];
            const auto fraction = (camPhase - left.camPhase) /
                                  (right.camPhase - left.camPhase);
            return left.pathPhase + fraction * (right.pathPhase - left.pathPhase);
        }
    }
    return 1.0;
}

Trajectory applyTimingMap(const Trajectory& trajectory) {
    trajectory.validate();
    auto result = trajectory;
    result.points.clear();
    result.points.reserve(trajectory.points.size());
    for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
        const auto camPhase = static_cast<double>(index) /
                              static_cast<double>(trajectory.points.size());
        const auto pathPhase = mapCamPhaseToPathPhase(camPhase, trajectory.timingKeys);
        result.points.push_back(interpolateClosed(trajectory.points, pathPhase));
    }
    result.timingKeys.clear();
    return result;
}

MotionProgram solveRotationRotation(
    const Trajectory& spatial,
    Vec3 pivotMm,
    double radialToleranceMm) {
    if (!std::isfinite(radialToleranceMm) || radialToleranceMm < 0.0) {
        throw std::runtime_error("Radial tolerance must be a finite non-negative number");
    }
    const auto timedSpatial = applyTimingMap(spatial);
    const auto analysis = analyzeSphericalFollower(timedSpatial, pivotMm);
    if (!analysis.feasible(radialToleranceMm)) {
        throw std::runtime_error(
            "XYZ path is not on one follower sphere; maximum radial error is " +
            std::to_string(analysis.maximumRadialErrorMm) + " mm");
    }

    MotionProgram result;
    result.followerRadiusMm = analysis.meanRadiusMm;
    result.pivotMm = pivotMm;
    result.samples.reserve(spatial.points.size());
    for (std::size_t index = 0; index < analysis.anglesRad.size(); ++index) {
        auto [alpha, beta] = analysis.anglesRad[index];
        if (!result.samples.empty()) {
            alpha = unwrap(result.samples.back().alphaRad, alpha);
            beta = unwrap(result.samples.back().betaRad, beta);
        }
        const auto phase = static_cast<double>(index) /
                           static_cast<double>(analysis.anglesRad.size());
        const auto pathPhase = mapCamPhaseToPathPhase(phase, spatial.timingKeys);
        result.samples.push_back({phase, pathPhase, alpha, beta});
    }
    return result;
}

Trajectory makeRotationRotationPitchCurve(
    const MotionProgram& motion,
    Vec3 jointAtRestMm) {
    if (motion.samples.size() < 3) {
        throw std::runtime_error("A motion program needs at least three samples");
    }

    Trajectory result;
    result.motion = FollowerMotion::RotationRotation;
    result.sourceName = "rotation-rotation pitch curve";
    result.points.reserve(motion.samples.size());
    const auto jointRelative = subtract(jointAtRestMm, motion.pivotMm);
    for (const auto& sample : motion.samples) {
        // This order matches the validated legacy model: Ry(alpha) * Rz(beta).
        auto joint = rotateY(rotateZ(jointRelative, sample.betaRad), sample.alphaRad);
        joint = add(motion.pivotMm, joint);
        const auto camAngle = -2.0 * std::numbers::pi * sample.camPhase;
        result.points.push_back(rotateZ(joint, camAngle));
    }
    result.validate();
    return result;
}

CycleDiagnostics analyzeCycle(const Trajectory& trajectory) {
    trajectory.validate();
    const auto count = trajectory.points.size();
    const auto cycleScale = static_cast<double>(count);
    std::vector<double> segments;
    segments.reserve(count);
    CycleDiagnostics result;

    for (std::size_t index = 0; index < count; ++index) {
        const auto next = (index + 1) % count;
        const auto segment = subtract(trajectory.points[next], trajectory.points[index]);
        segments.push_back(length(segment));
        result.maximumSpeedMmPerCycle =
            std::max(result.maximumSpeedMmPerCycle, length(segment) * cycleScale);

        const auto previous = (index + count - 1) % count;
        const auto acceleration = add(
            subtract(trajectory.points[next], multiply(trajectory.points[index], 2.0)),
            trajectory.points[previous]);
        result.maximumAccelerationMmPerCycleSquared = std::max(
            result.maximumAccelerationMmPerCycleSquared,
            length(acceleration) * cycleScale * cycleScale);
    }

    result.seamSegmentMm = segments.back();
    auto sorted = segments;
    std::sort(sorted.begin(), sorted.end());
    result.medianSegmentMm = sorted[sorted.size() / 2];
    result.seamLengthRatio = result.medianSegmentMm > std::numeric_limits<double>::epsilon()
                                 ? result.seamSegmentMm / result.medianSegmentMm
                                 : 0.0;
    const auto incoming = subtract(trajectory.points.front(), trajectory.points.back());
    const auto outgoing = subtract(trajectory.points[1], trajectory.points.front());
    result.seamTangentJumpDeg = angleBetweenDeg(incoming, outgoing);
    return result;
}

void writeMotionCsv(const MotionProgram& motion, const std::filesystem::path& path) {
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("Cannot write motion CSV: " + path.string());
    }
    output << "index,cam_phase,path_phase,alpha_deg,beta_deg\n" << std::setprecision(12);
    for (std::size_t index = 0; index < motion.samples.size(); ++index) {
        const auto& sample = motion.samples[index];
        output << index << ',' << sample.camPhase << ',' << sample.pathPhase << ','
               << sample.alphaRad * 180.0 / std::numbers::pi << ','
               << sample.betaRad * 180.0 / std::numbers::pi << '\n';
    }
}

}  // namespace camstudio
