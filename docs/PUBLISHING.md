# Publishing CamStudio on GitHub

## Before the first public release

1. Install and sign into [GitHub Desktop](https://desktop.github.com/) if needed.
2. Choose **File → Add Local Repository**, select this `CamStudio` folder, and choose
   **Publish repository**. A name such as `CamStudio` is easiest to recognize.
   Avoid GitHub's drag-and-drop web uploader because it may not preserve the executable permission
   required by `Launch CamStudio.command`.
3. Confirm that `build/`, `exports/`, `.DS_Store`, and personal STEP files are not included.
4. Add one screenshot or short GIF of the sphere sketcher and mechanism preview near the top of
   `README.md`. Use an image you own or have permission to publish.
5. Wait for the **Build and test** GitHub Action to turn green.
6. Ask one friend to test the Download ZIP → right-click launcher workflow on another Mac.
7. Create a GitHub release tagged `v0.1.0` and describe it as an experimental macOS prototype.

## Suggested repository description

> Draw a spherical 3D robot gait and generate a clean STEP composite cam for CAD and 3D printing.

Suggested topics: `cam`, `robotics`, `mechanism-design`, `3d-printing`, `fusion-360`, `opencascade`.

## What not to publish

- the local `build/` directory;
- generated designs under `exports/` unless intentionally added as an example;
- screenshots containing usernames, private paths, or unrelated windows;
- third-party media without permission;
- a claim that generated mechanisms are mechanically certified or universally safe.

## Release acceptance check

From a clean copy of the repository:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Then launch CamStudio, draw and close a small curve, apply Step 2 values, play one complete cycle,
generate the STEP, and import it into a new Fusion document. The STEP should arrive as one native
solid, and the gait-box dimensions should scale linearly when final leg length changes.
