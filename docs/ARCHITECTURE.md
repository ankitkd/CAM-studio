# CamStudio architecture

CamStudio is a modern replacement around the validated 3DCamFollower research algorithm.
The legacy application remains the behavioral reference; it is not a runtime dependency.

## Product boundary

1. A designer supplies a closed 3D foot trajectory in millimetres.
2. The core validates geometry and gait timing.
3. Cam synthesis calculates follower motion, pitch curves, clearance, and manufacturable solids.
4. Exporters write a verified native STEP cam plus curve and motion data.
5. A local browser UI previews motion; CAD exchange uses standard STEP without a Fusion add-in.

## Components

- `camstudio_core`: platform-independent trajectory, timing, synthesis, and validation logic.
- `camstudio`: command-line development interface used for reproducible tests.
- Local designer UI: sphere-constrained path sketching and one-click generation through the core.
- CAD backend: Open CASCADE boundary-representation output with STEP re-import validation.
- Fusion workflow: direct STEP exchange; no add-in is required.

## Geometry rules

- Internal and user-facing dimensions are millimetres.
- Moving parts are never concatenated into a single anonymous mesh.
- Each printable/manufacturable component has an identity and local coordinate system.
- Motion-critical cam surfaces retain full precision.
- Mesh output is for preview and printing; STEP output must be a genuine solid model.

## Migration strategy

The first fixture is the legacy `ginger.txt` example. Every migrated stage must reproduce its
validated trajectory and later its pitch curve and follower motion before replacing the old code.
