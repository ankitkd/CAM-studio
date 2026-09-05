#pragma once

#include "core/trajectory.h"

#include <filesystem>

namespace camstudio {

struct NativePartsConfig {
    Vec3 pivotMm{-80.0, 0.0, 0.0};
    Vec3 jointMm{-40.0, 0.0, 0.0};
    Vec3 ankleMm{-60.0, 0.0, 0.0};
    Vec3 footTipMm{-230.0, 0.0, 0.0};
    double contactBallRadiusMm{2.5};
    double rodRadiusMm{1.8};
    double shaftRadiusMm{6.0};
};

void exportNativePartsStep(
    const std::filesystem::path& outputDirectory,
    const NativePartsConfig& config = {});
void exportCleanCamStep(
    const Trajectory& pitchCurve,
    const std::filesystem::path& outputStep,
    double followerBallRadiusMm = 2.5,
    double printClearanceMm = 0.25,
    double edgeMarginMm = 2.3);
void repairLegacyObjToStep(
    const std::filesystem::path& inputObj,
    const std::filesystem::path& outputStep);

}  // namespace camstudio
