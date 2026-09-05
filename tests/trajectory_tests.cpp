#include "core/motion.h"
#include "core/trajectory.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool closeTo(double actual, double expected, double tolerance = 1e-6) {
    return std::abs(actual - expected) <= tolerance;
}

template <typename Action>
void requireThrows(Action action, const char* message) {
    try {
        action();
    } catch (const std::exception&) {
        return;
    }
    throw std::runtime_error(message);
}

}  // namespace

int main() {
    try {
        const auto fixture = std::filesystem::path(CAMSTUDIO_FIXTURE_DIR) / "ginger.txt";
        const auto trajectory = camstudio::readLegacyTrajectory(fixture);

        require(trajectory.points.size() == 360, "ginger point count changed");
        require(trajectory.motion == camstudio::FollowerMotion::RotationRotation,
                "ginger follower motion changed");
        require(trajectory.timingKeys.empty(), "ginger should not contain timing keys");
        require(closeTo(trajectory.boundsDiagonal(), 0.660898, 1e-5),
                "ginger source bounds changed");

        const auto normalized = trajectory.centeredAndScaled(70.0);
        require(closeTo(normalized.boundsDiagonal(), 70.0),
                "normalization must match the legacy 70 mm rule");
        const auto box = normalized.bounds();
        require(closeTo(box.minimum.x + box.maximum.x, 0.0), "normalized X is not centered");
        require(closeTo(box.minimum.y + box.maximum.y, 0.0), "normalized Y is not centered");

        const auto spatial = camstudio::projectPlanarToSpherical(normalized, 150.0);
        const auto spherical = camstudio::analyzeSphericalFollower(spatial);
        require(closeTo(spherical.meanRadiusMm, 150.0), "legacy projection radius changed");
        require(spherical.maximumRadialErrorMm < 1e-9,
                "legacy projected path must remain on the follower sphere");
        require(spherical.feasible(0.01), "legacy projected path should be feasible");

        const auto motion = camstudio::solveRotationRotation(spatial, {}, 0.01);
        require(motion.samples.size() == spatial.points.size(),
                "motion solver changed the sample count");
        for (std::size_t index = 0; index < motion.samples.size(); ++index) {
            const auto& sample = motion.samples[index];
            const auto cosineAlpha = std::cos(sample.alphaRad);
            const auto sineAlpha = std::sin(sample.alphaRad);
            const auto cosineBeta = std::cos(sample.betaRad);
            const auto sineBeta = std::sin(sample.betaRad);
            const camstudio::Vec3 reconstructed{
                -motion.followerRadiusMm * cosineAlpha * cosineBeta,
                -motion.followerRadiusMm * sineBeta,
                motion.followerRadiusMm * sineAlpha * cosineBeta,
            };
            require(closeTo(reconstructed.x, spatial.points[index].x, 1e-8),
                    "alpha/beta reconstruction changed X");
            require(closeTo(reconstructed.y, spatial.points[index].y, 1e-8),
                    "alpha/beta reconstruction changed Y");
            require(closeTo(reconstructed.z, spatial.points[index].z, 1e-8),
                    "alpha/beta reconstruction changed Z");
        }

        const auto pitch = camstudio::makeRotationRotationPitchCurve(
            motion, {-40.0, 0.0, 0.0});
        require(pitch.points.size() == spatial.points.size(),
                "pitch curve changed the sample count");

        require(closeTo(camstudio::mapCamPhaseToPathPhase(0.25, {}), 0.25),
                "empty timing map must be linear");
        const std::vector<camstudio::TimingKey> timing{{0.25, 0.1}, {0.75, 0.9}};
        require(closeTo(camstudio::mapCamPhaseToPathPhase(0.25, timing), 0.1),
                "timing key interpolation missed a key");
        require(closeTo(camstudio::mapCamPhaseToPathPhase(0.5, timing), 0.5),
                "timing key interpolation changed the middle segment");

        requireThrows(
            [&] { static_cast<void>(camstudio::solveRotationRotation(spatial, {}, -0.01)); },
            "negative radial tolerance was accepted");

        const auto diagnostics = camstudio::analyzeCycle(spatial);
        require(std::isfinite(diagnostics.seamTangentJumpDeg),
                "cycle diagnostics produced a non-finite seam angle");
        require(diagnostics.medianSegmentMm > 0.0,
                "cycle diagnostics produced an empty median segment");

        std::cout << "All trajectory tests passed\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "Test failure: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
