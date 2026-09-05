# CamStudio delivery roadmap

## M1 — Verified trajectory foundation (complete)

- legacy trajectory fixture and parser
- millimetre-based internal geometry
- validation, centering, scaling, and regression tests
- dependency-free modern C++ build

## M2 — 3D path and kinematic feasibility (complete)

- native XYZ trajectory input
- closed-cycle and smoothness diagnostics
- spherical two-rotation follower solver
- unreachable-path and radial-error reporting
- reproduce legacy ginger follower angles

## M3 — Cam synthesis migration (in progress)

- migrate spline timing and optimization behind the core API
- reproduce legacy pitch curves and sampled motion (initial deterministic implementation complete)
- deterministic optimization tests
- configurable shaft, follower, roller, and clearances

## M4 — Manufacturable geometry (in progress)

- named components instead of concatenated shells (STEP baseline complete)
- watertight mesh output for 3MF/OBJ
- CAD-kernel solids for STEP (single-solid repaired cam and native follower baseline complete)
- replace repaired cam facets with compact smooth composite-cam surfaces
- collision, wall-thickness, and print-clearance checks

## M5 — Designer interface (initial release complete)

- sphere-constrained 3D path sketcher
- gait bounding-box dimensions
- full cam/follower/leg motion playback
- live mechanism guardrails
- automatic local draft recovery

Future work: gait timing controls, curve import, and desktop-insect presets.

## M6 — CAD integration

- direct, verified STEP export (complete)
- clean flat cam sides for downstream CAD structures (complete)
- future: reusable shaft/bore options and named follower-part export
