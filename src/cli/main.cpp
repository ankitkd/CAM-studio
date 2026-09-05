#include "cad/cad_export.h"
#include "core/motion.h"
#include "core/trajectory.h"

#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void usage() {
    std::cout
        << "CamStudio trajectory tools\n\n"
        << "Usage:\n"
        << "  camstudio inspect <legacy-curve.txt>\n"
        << "  camstudio normalize <legacy-curve.txt> <output.csv> [size-mm]\n"
        << "  camstudio legacy-rr <legacy-curve.txt> <output.csv> [size-mm] [radius-mm]\n"
        << "  camstudio analyze-3d <trajectory.csv> [tolerance-mm]\n"
        << "  camstudio diagnose <trajectory.csv>\n"
        << "  camstudio export-parts-step <output-directory>"
           " [pivot-x-mm] [joint-x-mm] [ankle-x-mm] [foot-x-mm]\n"
        << "  camstudio export-clean-cam-step <pitch.csv> <output.step>"
           " [ball-radius-mm] [clearance-mm] [edge-margin-mm]\n"
        << "  camstudio cam-from-sphere <trajectory.csv> <output-directory>"
           " [pivot-x] [pivot-y] [pivot-z] [joint-x] [joint-y] [joint-z]"
           " [ball-radius-mm] [clearance-mm] [edge-margin-mm]\n"
        << "  camstudio repair-cam-step <legacy-cam.obj> <output.step>\n"
        << "  camstudio pitch-rr <legacy-curve.txt> <pitch.csv> <motion.csv>"
           " [size-mm] [radius-mm] [pivot-x-mm] [joint-x-mm]\n";
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc < 3) {
            usage();
            return EXIT_FAILURE;
        }

        const std::string command = argv[1];

        if (command == "cam-from-sphere") {
            if (argc < 4 || argc > 13) {
                usage();
                return EXIT_FAILURE;
            }
            camstudio::Vec3 pivot{-80.0, 0.0, 0.0};
            camstudio::Vec3 joint{-40.0, 0.0, 0.0};
            if (argc >= 5) pivot.x = std::stod(argv[4]);
            if (argc >= 6) pivot.y = std::stod(argv[5]);
            if (argc >= 7) pivot.z = std::stod(argv[6]);
            if (argc >= 8) joint.x = std::stod(argv[7]);
            if (argc >= 9) joint.y = std::stod(argv[8]);
            if (argc >= 10) joint.z = std::stod(argv[9]);
            const double ballRadiusMm = argc >= 11 ? std::stod(argv[10]) : 2.5;
            const double clearanceMm = argc >= 12 ? std::stod(argv[11]) : 0.25;
            const double edgeMarginMm = argc == 13 ? std::stod(argv[12]) : 2.3;

            const auto spatial = camstudio::readXyzCsv(argv[2]);
            const auto motion = camstudio::solveRotationRotation(spatial, pivot, 0.05);
            const auto pitch = camstudio::makeRotationRotationPitchCurve(motion, joint);
            const std::filesystem::path outputDirectory = argv[3];
            std::filesystem::create_directories(outputDirectory);
            camstudio::writeCsv(pitch, outputDirectory / "pitch-curve.csv");
            camstudio::writeMotionCsv(motion, outputDirectory / "follower-motion.csv");
            camstudio::exportCleanCamStep(
                pitch,
                outputDirectory / "cam-clean-disc.step",
                ballRadiusMm,
                clearanceMm,
                edgeMarginMm);
            const auto analysis = camstudio::analyzeSphericalFollower(spatial, pivot);
            std::cout << std::fixed << std::setprecision(6)
                      << "Sphere radius: " << analysis.meanRadiusMm << " mm\n"
                      << "Maximum sphere error: " << analysis.maximumRadialErrorMm << " mm\n"
                      << "Wrote complete cam design to " << outputDirectory << '\n';
            return EXIT_SUCCESS;
        }

        if (command == "export-clean-cam-step") {
            if (argc < 4 || argc > 7) {
                usage();
                return EXIT_FAILURE;
            }
            const auto pitchCurve = camstudio::readXyzCsv(argv[2]);
            const double ballRadiusMm = argc >= 5 ? std::stod(argv[4]) : 2.5;
            const double clearanceMm = argc >= 6 ? std::stod(argv[5]) : 0.25;
            const double edgeMarginMm = argc == 7 ? std::stod(argv[6]) : 2.3;
            camstudio::exportCleanCamStep(
                pitchCurve,
                argv[3],
                ballRadiusMm,
                clearanceMm,
                edgeMarginMm);
            std::cout << "Wrote and verified a smooth native STEP cam at " << argv[3] << '\n';
            return EXIT_SUCCESS;
        }

        if (command == "repair-cam-step") {
            if (argc != 4) {
                usage();
                return EXIT_FAILURE;
            }
            camstudio::repairLegacyObjToStep(argv[2], argv[3]);
            std::cout << "Rebuilt and verified one STEP solid at " << argv[3] << '\n';
            return EXIT_SUCCESS;
        }

        if (command == "export-parts-step") {
            if (argc < 3 || argc > 7) {
                usage();
                return EXIT_FAILURE;
            }
            camstudio::NativePartsConfig config;
            if (argc >= 4) config.pivotMm.x = std::stod(argv[3]);
            if (argc >= 5) config.jointMm.x = std::stod(argv[4]);
            if (argc >= 6) config.ankleMm.x = std::stod(argv[5]);
            if (argc == 7) config.footTipMm.x = std::stod(argv[6]);
            camstudio::exportNativePartsStep(argv[2], config);
            std::cout << "Wrote native STEP parts to " << argv[2] << '\n'
                      << "  follower.step (one joined solid)\n"
                      << "  shaft.step\n"
                      << "  supports.step\n";
            return EXIT_SUCCESS;
        }

        if (command == "analyze-3d") {
            const auto trajectory = camstudio::readXyzCsv(argv[2]);
            const double toleranceMm = argc == 4 ? std::stod(argv[3]) : 0.1;
            const auto analysis = camstudio::analyzeSphericalFollower(trajectory);
            std::cout << std::fixed << std::setprecision(6)
                      << "Points: " << trajectory.points.size() << '\n'
                      << "Mean follower radius: " << analysis.meanRadiusMm << " mm\n"
                      << "Radius range: " << analysis.minimumRadiusMm << " to "
                      << analysis.maximumRadiusMm << " mm\n"
                      << "Maximum radial error: " << analysis.maximumRadialErrorMm << " mm\n"
                      << "RMS radial error: " << analysis.rmsRadialErrorMm << " mm\n"
                      << "Two-rotation follower: "
                      << (analysis.feasible(toleranceMm) ? "FEASIBLE" : "NOT FEASIBLE")
                      << " at " << toleranceMm << " mm tolerance\n";
            return analysis.feasible(toleranceMm) ? EXIT_SUCCESS : 2;
        }

        if (command == "diagnose") {
            const auto trajectory = camstudio::readXyzCsv(argv[2]);
            const auto diagnostics = camstudio::analyzeCycle(trajectory);
            std::cout << std::fixed << std::setprecision(6)
                      << "Seam segment: " << diagnostics.seamSegmentMm << " mm\n"
                      << "Median segment: " << diagnostics.medianSegmentMm << " mm\n"
                      << "Seam/median ratio: " << diagnostics.seamLengthRatio << '\n'
                      << "Seam direction change: " << diagnostics.seamTangentJumpDeg << " degrees\n"
                      << "Maximum normalized speed: "
                      << diagnostics.maximumSpeedMmPerCycle << " mm/cycle\n"
                      << "Maximum normalized acceleration: "
                      << diagnostics.maximumAccelerationMmPerCycleSquared
                      << " mm/cycle^2\n";
            return EXIT_SUCCESS;
        }

        const auto trajectory = camstudio::readLegacyTrajectory(argv[2]);

        if (command == "inspect") {
            const auto bounds = trajectory.bounds();
            std::cout << std::fixed << std::setprecision(6)
                      << "Source: " << trajectory.sourceName << '\n'
                      << "Points: " << trajectory.points.size() << '\n'
                      << "Follower motion: " << camstudio::motionName(trajectory.motion) << '\n'
                      << "Timing keys: " << trajectory.timingKeys.size() << '\n'
                      << "Bounds diagonal: " << trajectory.boundsDiagonal() << '\n'
                      << "Closure gap: " << trajectory.closureGap() << '\n'
                      << "Minimum: " << bounds.minimum.x << ", " << bounds.minimum.y << ", " << bounds.minimum.z << '\n'
                      << "Maximum: " << bounds.maximum.x << ", " << bounds.maximum.y << ", " << bounds.maximum.z << '\n';
            return EXIT_SUCCESS;
        }

        if (command == "normalize") {
            if (argc < 4 || argc > 5) {
                usage();
                return EXIT_FAILURE;
            }
            const double targetSizeMm = argc == 5 ? std::stod(argv[4]) : 70.0;
            const auto normalized = trajectory.centeredAndScaled(targetSizeMm);
            camstudio::writeCsv(normalized, argv[3]);
            std::cout << "Wrote " << normalized.points.size() << " points at "
                      << targetSizeMm << " mm to " << argv[3] << '\n';
            return EXIT_SUCCESS;
        }

        if (command == "legacy-rr") {
            if (argc < 4 || argc > 6) {
                usage();
                return EXIT_FAILURE;
            }
            const double targetSizeMm = argc >= 5 ? std::stod(argv[4]) : 70.0;
            const double radiusMm = argc == 6 ? std::stod(argv[5]) : 150.0;
            const auto normalized = trajectory.centeredAndScaled(targetSizeMm);
            const auto spatial = camstudio::projectPlanarToSpherical(normalized, radiusMm);
            camstudio::writeCsv(spatial, argv[3]);
            const auto analysis = camstudio::analyzeSphericalFollower(spatial);
            std::cout << "Wrote the legacy-equivalent 3D path to " << argv[3] << '\n'
                      << "Follower radius: " << analysis.meanRadiusMm << " mm\n"
                      << "Maximum radial error: " << analysis.maximumRadialErrorMm << " mm\n";
            return EXIT_SUCCESS;
        }


        if (command == "pitch-rr") {
            if (argc < 5 || argc > 9) {
                usage();
                return EXIT_FAILURE;
            }
            const double targetSizeMm = argc >= 6 ? std::stod(argv[5]) : 70.0;
            const double radiusMm = argc >= 7 ? std::stod(argv[6]) : 150.0;
            const double pivotX = argc >= 8 ? std::stod(argv[7]) : -80.0;
            const double jointX = argc == 9 ? std::stod(argv[8]) : -40.0;
            const camstudio::Vec3 pivot{pivotX, 0.0, 0.0};
            const auto normalized = trajectory.centeredAndScaled(targetSizeMm);
            const auto spatial = camstudio::projectPlanarToSpherical(normalized, radiusMm, pivot);
            const auto motion = camstudio::solveRotationRotation(spatial, pivot);
            const auto pitch = camstudio::makeRotationRotationPitchCurve(
                motion, {jointX, 0.0, 0.0});
            camstudio::writeCsv(pitch, argv[3]);
            camstudio::writeMotionCsv(motion, argv[4]);
            const auto diagnostics = camstudio::analyzeCycle(spatial);
            std::cout << "Wrote " << pitch.points.size() << " pitch-curve points to "
                      << argv[3] << '\n'
                      << "Wrote follower angles to " << argv[4] << '\n'
                      << "Follower pivot X: " << pivotX << " mm\n"
                      << "Follower joint X: " << jointX << " mm\n"
                      << "Target-path seam direction change: "
                      << diagnostics.seamTangentJumpDeg << " degrees\n";
            return EXIT_SUCCESS;
        }

        usage();
        return EXIT_FAILURE;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
