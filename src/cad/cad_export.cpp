#include "cad/cad_export.h"

#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepLib.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <STEPControl_StepModelType.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <StlAPI_Writer.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <BRep_Builder.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>
#include <array>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace camstudio {
namespace {

gp_Pnt point(const Vec3& value) {
    return {value.x, value.y, value.z};
}

TopoDS_Shape cylinder(const Vec3& start, const Vec3& end, double radiusMm) {
    const gp_Vec vector(point(start), point(end));
    if (vector.Magnitude() <= 1e-9 || radiusMm <= 0.0) {
        throw std::runtime_error("A native cylinder has invalid dimensions");
    }
    return BRepPrimAPI_MakeCylinder(
               gp_Ax2(point(start), gp_Dir(vector)), radiusMm, vector.Magnitude())
        .Shape();
}

TopoDS_Shape unite(const TopoDS_Shape& target, const TopoDS_Shape& tool) {
    BRepAlgoAPI_Fuse operation(target, tool);
    operation.SetFuzzyValue(1e-6);
    operation.Build();
    if (!operation.IsDone() || operation.Shape().IsNull()) {
        throw std::runtime_error("Open CASCADE could not unite the follower primitives");
    }
    return operation.Shape();
}

void requireValid(const TopoDS_Shape& shape, const std::string& name) {
    if (shape.IsNull() || !BRepCheck_Analyzer(shape, true).IsValid()) {
        throw std::runtime_error(name + " is not a valid CAD boundary representation");
    }
}

std::size_t solidCount(const TopoDS_Shape& shape) {
    std::size_t count{};
    for (TopExp_Explorer explorer(shape, TopAbs_SOLID); explorer.More(); explorer.Next()) {
        ++count;
    }
    return count;
}

void writeStep(
    const TopoDS_Shape& shape,
    const std::filesystem::path& path,
    std::size_t expectedSolidCount) {
    STEPControl_Writer writer;
    if (writer.Transfer(shape, STEPControl_AsIs) != IFSelect_RetDone) {
        throw std::runtime_error("Could not transfer CAD geometry to STEP: " + path.string());
    }
    if (writer.Write(path.string().c_str()) != IFSelect_RetDone) {
        throw std::runtime_error("Could not write STEP file: " + path.string());
    }

    STEPControl_Reader reader;
    if (reader.ReadFile(path.string().c_str()) != IFSelect_RetDone ||
        reader.TransferRoots() <= 0) {
        throw std::runtime_error("Could not verify the written STEP file: " + path.string());
    }
    const auto imported = reader.OneShape();
    requireValid(imported, path.filename().string());
    if (solidCount(imported) != expectedSolidCount) {
        throw std::runtime_error(
            path.filename().string() + " re-imported with an unexpected number of solids");
    }
}

void writePreviewStl(const TopoDS_Shape& shape, const std::filesystem::path& path) {
    BRepMesh_IncrementalMesh mesher(shape, 0.12, false, 0.25, true);
    mesher.Perform();
    if (!mesher.IsDone()) {
        throw std::runtime_error("Could not mesh the cam preview");
    }
    StlAPI_Writer writer;
    if (!writer.Write(shape, path.string().c_str())) {
        throw std::runtime_error("Could not write the cam preview STL");
    }
}

TopoDS_Shape makeFollower(const NativePartsConfig& config) {
    auto follower = BRepPrimAPI_MakeSphere(point(config.jointMm), config.contactBallRadiusMm).Shape();
    follower = unite(follower, cylinder(config.jointMm, config.ankleMm, config.rodRadiusMm));
    follower = unite(follower, BRepPrimAPI_MakeSphere(point(config.ankleMm), config.rodRadiusMm).Shape());
    follower = unite(follower, cylinder(config.ankleMm, config.pivotMm, config.rodRadiusMm));
    follower = unite(follower, BRepPrimAPI_MakeSphere(point(config.pivotMm), config.rodRadiusMm).Shape());
    follower = unite(follower, cylinder(config.pivotMm, config.footTipMm, config.rodRadiusMm));
    return follower;
}

TopoDS_Shape makeSupports() {
    BRep_Builder builder;
    TopoDS_Compound supports;
    builder.MakeCompound(supports);
    builder.Add(supports, BRepPrimAPI_MakeBox(gp_Pnt(-10.0, -10.0, -34.0), 20.0, 20.0, 8.0).Shape());
    builder.Add(supports, BRepPrimAPI_MakeBox(gp_Pnt(-10.0, -10.0, 26.0), 20.0, 20.0, 8.0).Shape());
    return supports;
}

TopoDS_Wire camSectionWire(
    const Vec3& pitch,
    double channelRadius,
    double innerRadius,
    double outerRadius,
    double bottomZ,
    double topZ) {
    const auto pitchRadius = std::hypot(pitch.x, pitch.y);
    if (pitchRadius <= innerRadius + channelRadius) {
        throw std::runtime_error("Cam section does not leave enough material around the shaft");
    }
    const auto ux = pitch.x / pitchRadius;
    const auto uy = pitch.y / pitchRadius;
    const auto at = [&](double radius, double z) {
        return gp_Pnt(radius * ux, radius * uy, z);
    };

    const auto innerBottom = at(innerRadius, bottomZ);
    const auto outerBottom = at(outerRadius, bottomZ);
    const auto lowerOpening = at(outerRadius, pitch.z - channelRadius);
    const auto grooveBottom = at(pitchRadius, pitch.z - channelRadius);
    const auto grooveBack = at(pitchRadius - channelRadius, pitch.z);
    const auto grooveTop = at(pitchRadius, pitch.z + channelRadius);
    const auto upperOpening = at(outerRadius, pitch.z + channelRadius);
    const auto outerTop = at(outerRadius, topZ);
    const auto innerTop = at(innerRadius, topZ);

    BRepBuilderAPI_MakeWire wire;
    wire.Add(BRepBuilderAPI_MakeEdge(innerBottom, outerBottom).Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(outerBottom, lowerOpening).Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(lowerOpening, grooveBottom).Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(
                 GC_MakeArcOfCircle(grooveBottom, grooveBack, grooveTop).Value())
                 .Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(grooveTop, upperOpening).Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(upperOpening, outerTop).Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(outerTop, innerTop).Edge());
    wire.Add(BRepBuilderAPI_MakeEdge(innerTop, innerBottom).Edge());
    if (!wire.IsDone()) {
        throw std::runtime_error("Could not construct a clean cam cross-section");
    }
    return wire.Wire();
}

TopoDS_Shape loftClosedCam(
    const std::vector<Vec3>& pitchPoints,
    double channelRadius,
    double innerRadius,
    double outerRadius,
    double bottomZ,
    double topZ) {
    // Build the entire circumference as one periodic loft.  The previous
    // implementation made two solid half-lofts and boolean-fused their
    // coincident end faces.  Open CASCADE can leave invalid seam edges after
    // that fuse for otherwise valid, smooth user paths.
    BRepOffsetAPI_ThruSections loft(true, false, 1e-4);
    loft.CheckCompatibility(false);
    for (const auto& pitch : pitchPoints) {
        loft.AddWire(camSectionWire(
            pitch, channelRadius, innerRadius, outerRadius, bottomZ, topZ));
    }
    // Repeating the first section closes the periodic solid without a boolean
    // operation at the cam's circumferential seam.
    loft.AddWire(camSectionWire(
        pitchPoints.front(), channelRadius, innerRadius, outerRadius, bottomZ, topZ));
    loft.Build();
    if (!loft.IsDone() || loft.Shape().IsNull()) {
        throw std::runtime_error("Could not loft the closed clean cam surfaces");
    }
    return loft.Shape();
}

}  // namespace

void exportNativePartsStep(
    const std::filesystem::path& outputDirectory,
    const NativePartsConfig& config) {
    if (!std::isfinite(config.contactBallRadiusMm) || config.contactBallRadiusMm <= 0.0 ||
        !std::isfinite(config.rodRadiusMm) || config.rodRadiusMm <= 0.0 ||
        !std::isfinite(config.shaftRadiusMm) || config.shaftRadiusMm <= 0.0) {
        throw std::runtime_error("Part radii must be positive finite millimetre values");
    }
    std::filesystem::create_directories(outputDirectory);

    const auto follower = makeFollower(config);
    const auto shaft = cylinder({0.0, 0.0, -40.0}, {0.0, 0.0, 40.0}, config.shaftRadiusMm);
    const auto supports = makeSupports();
    requireValid(follower, "Follower");
    requireValid(shaft, "Cam shaft");
    requireValid(supports, "Supports");

    writeStep(follower, outputDirectory / "follower.step", 1);
    writeStep(shaft, outputDirectory / "shaft.step", 1);
    writeStep(supports, outputDirectory / "supports.step", 2);
}

void exportCleanCamStep(
    const Trajectory& pitchCurve,
    const std::filesystem::path& outputStep,
    double followerBallRadiusMm,
    double printClearanceMm,
    double edgeMarginMm) {
    pitchCurve.validate();
    if (pitchCurve.points.size() < 36) {
        throw std::runtime_error(
            "A clean cam needs at least 36 pitch-curve samples around one revolution");
    }
    if (!std::isfinite(followerBallRadiusMm) || followerBallRadiusMm <= 0.0 ||
        !std::isfinite(printClearanceMm) || printClearanceMm < 0.0 ||
        !std::isfinite(edgeMarginMm) || edgeMarginMm <= 0.0) {
        throw std::runtime_error("Clean-cam dimensions must be finite positive millimetre values");
    }

    const auto channelRadius = followerBallRadiusMm + printClearanceMm;
    double minimumZ = pitchCurve.points.front().z;
    double maximumZ = minimumZ;
    double maximumPitchRadius = 0.0;
    for (const auto& pitchPoint : pitchCurve.points) {
        const auto radius = std::hypot(pitchPoint.x, pitchPoint.y);
        if (radius <= channelRadius + 1.0) {
            throw std::runtime_error("Pitch curve passes too close to the cam shaft");
        }
        minimumZ = std::min(minimumZ, pitchPoint.z);
        maximumZ = std::max(maximumZ, pitchPoint.z);
        maximumPitchRadius = std::max(maximumPitchRadius, radius);
    }

    const auto bottomZ = minimumZ - channelRadius - edgeMarginMm;
    const auto topZ = maximumZ + channelRadius + edgeMarginMm;
    const auto outerRadius = maximumPitchRadius + channelRadius + edgeMarginMm;
    const auto innerRadius = 0.0;
    auto cam = loftClosedCam(
        pitchCurve.points,
        channelRadius,
        innerRadius,
        outerRadius,
        bottomZ,
        topZ);
    requireValid(cam, "Smooth closed circumferential cam loft");

    requireValid(cam, "Clean composite cam");
    const auto cleanSolidCount = solidCount(cam);
    if (cleanSolidCount != 1) {
        throw std::runtime_error(
            "Clean composite cam resolved to " + std::to_string(cleanSolidCount) +
            " solids instead of one");
    }

    if (!outputStep.parent_path().empty()) {
        std::filesystem::create_directories(outputStep.parent_path());
    }
    writeStep(cam, outputStep, 1);
    writePreviewStl(cam, outputStep.parent_path() / "cam-clean-disc-preview.stl");
}

void repairLegacyObjToStep(
    const std::filesystem::path& inputObj,
    const std::filesystem::path& outputStep) {
    std::ifstream input(inputObj);
    if (!input) {
        throw std::runtime_error("Cannot open legacy OBJ: " + inputObj.string());
    }

    std::vector<Vec3> vertices;
    std::vector<std::array<std::size_t, 3>> triangles;
    std::string line;
    while (std::getline(input, line)) {
        std::istringstream stream(line);
        std::string type;
        stream >> type;
        if (type == "v") {
            Vec3 vertex;
            if (!(stream >> vertex.x >> vertex.y >> vertex.z)) {
                throw std::runtime_error("Invalid vertex in legacy OBJ");
            }
            vertices.push_back(vertex);
        } else if (type == "f") {
            std::array<std::size_t, 3> triangle{};
            for (auto& index : triangle) {
                std::string token;
                if (!(stream >> token)) {
                    throw std::runtime_error("Only triangular OBJ faces are supported");
                }
                const auto slash = token.find('/');
                const auto raw = std::stoll(token.substr(0, slash));
                if (raw <= 0 || static_cast<std::size_t>(raw) > vertices.size()) {
                    throw std::runtime_error("Invalid face index in legacy OBJ");
                }
                index = static_cast<std::size_t>(raw - 1);
            }

            std::string extra;
            if (stream >> extra) {
                throw std::runtime_error("Only triangular OBJ faces are supported");
            }
            triangles.push_back(triangle);
        }
    }
    if (vertices.empty() || triangles.empty()) {
        throw std::runtime_error("Legacy OBJ contains no usable mesh");
    }

    BRepBuilderAPI_Sewing sewing(1e-4, true, true, true, false);
    for (const auto& triangle : triangles) {
        const auto a = point(vertices[triangle[0]]);
        const auto b = point(vertices[triangle[1]]);
        const auto c = point(vertices[triangle[2]]);
        const gp_Vec ab(a, b);
        const gp_Vec ac(a, c);
        if (ab.Crossed(ac).Magnitude() <= 1e-10) {
            continue;
        }
        const auto polygon = BRepBuilderAPI_MakePolygon(a, b, c, true).Wire();
        sewing.Add(BRepBuilderAPI_MakeFace(polygon, true).Face());
    }
    sewing.Perform();
    const auto sewed = sewing.SewedShape();
    if (sewed.IsNull()) {
        throw std::runtime_error("Legacy OBJ faces could not be sewn into CAD shells");
    }

    std::vector<TopoDS_Shape> solids;
    for (TopExp_Explorer explorer(sewed, TopAbs_SHELL); explorer.More(); explorer.Next()) {
        auto solid = BRepBuilderAPI_MakeSolid(TopoDS::Shell(explorer.Current())).Solid();
        BRepLib::OrientClosedSolid(solid);
        if (BRepCheck_Analyzer(solid, true).IsValid()) {
            solids.push_back(solid);
        }
    }
    if (solids.empty()) {
        throw std::runtime_error("Legacy OBJ did not produce a closed solid shell");
    }

    auto result = solids.front();
    for (std::size_t index = 1; index < solids.size(); ++index) {
        result = unite(result, solids[index]);
    }
    requireValid(result, "Repaired legacy cam");
    if (solidCount(result) != 1) {
        throw std::runtime_error(
            "Repaired legacy cam contains " + std::to_string(solidCount(result)) +
            " solids instead of one");
    }
    if (!outputStep.parent_path().empty()) {
        std::filesystem::create_directories(outputStep.parent_path());
    }
    writeStep(result, outputStep, 1);
    writePreviewStl(result, outputStep.parent_path() / "cam-reference-preview.stl");
}

}  // namespace camstudio
