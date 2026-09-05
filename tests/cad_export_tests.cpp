#include "cad/cad_export.h"
#include "core/motion.h"
#include "core/trajectory.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
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

camstudio::Trajectory makeSphericalGait() {
    constexpr std::size_t sampleCount = 72;
    constexpr double radiusMm = 20.0;
    const camstudio::Vec3 pivot{-42.0, 0.0, 0.0};
    camstudio::Trajectory gait;
    gait.sourceName = "synthetic release-test gait";
    gait.points.reserve(sampleCount);
    for (std::size_t index = 0; index < sampleCount; ++index) {
        const auto phase = 2.0 * std::numbers::pi *
                           static_cast<double>(index) / static_cast<double>(sampleCount);
        const auto y = 0.27 * std::sin(phase);
        const auto z = 0.20 * std::sin(2.0 * phase) + 0.05 * std::cos(phase);
        const auto x = -std::sqrt(1.0 - y * y - z * z);
        gait.points.push_back({
            pivot.x + radiusMm * x,
            pivot.y + radiusMm * y,
            pivot.z + radiusMm * z,
        });
    }
    return gait;
}

}  // namespace

int main() {
    const auto unique = std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count());
    const auto outputDirectory =
        std::filesystem::temp_directory_path() / ("camstudio-cad-test-" + unique);
    try {
        const auto gait = makeSphericalGait();
        const camstudio::Vec3 pivot{-42.0, 0.0, 0.0};
        const auto motion = camstudio::solveRotationRotation(gait, pivot, 1e-6);
        const auto pitch = camstudio::makeRotationRotationPitchCurve(
            motion, {-22.0, 0.0, 0.0});
        const auto step = outputDirectory / "release-test-cam.step";
        camstudio::exportCleanCamStep(pitch, step, 4.0, 0.25, 2.3);
        require(std::filesystem::is_regular_file(step), "STEP integration test wrote no file");
        require(std::filesystem::file_size(step) > 10'000,
                "STEP integration test produced an unexpectedly small file");
        require(std::filesystem::is_regular_file(outputDirectory / "cam-clean-disc-preview.stl"),
                "STEP integration test wrote no preview STL");

        requireThrows(
            [&] { camstudio::exportCleanCamStep(pitch, outputDirectory / "bad-radius.step", 0.0, 0.25, 2.3); },
            "zero contact-ball radius was accepted");
        requireThrows(
            [&] { camstudio::exportCleanCamStep(pitch, outputDirectory / "bad-clearance.step", 4.0, -0.1, 2.3); },
            "negative print clearance was accepted");

        auto undersampled = pitch;
        undersampled.points.resize(12);
        requireThrows(
            [&] { camstudio::exportCleanCamStep(undersampled, outputDirectory / "undersampled.step", 4.0, 0.25, 2.3); },
            "an undersampled cam revolution was accepted");

        camstudio::Trajectory tooClose;
        for (std::size_t index = 0; index < 36; ++index) {
            const auto phase = 2.0 * std::numbers::pi *
                               static_cast<double>(index) / 36.0;
            tooClose.points.push_back({std::cos(phase), std::sin(phase), 0.0});
        }
        requireThrows(
            [&] { camstudio::exportCleanCamStep(tooClose, outputDirectory / "shaft-collision.step", 2.5, 0.25, 2.3); },
            "a groove intersecting the shaft region was accepted");

        std::error_code cleanupError;
        std::filesystem::remove_all(outputDirectory, cleanupError);
        std::cout << "All CAD export tests passed\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::error_code cleanupError;
        std::filesystem::remove_all(outputDirectory, cleanupError);
        std::cerr << "CAD test failure: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
