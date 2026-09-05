# CamStudio

CamStudio turns a hand-drawn three-dimensional foot path into a grooved composite cam for a
small walking robot. Draw the gait on a sphere, preview the complete cam/leg motion, choose your
real dimensions, and export a clean native STEP solid for Fusion 360 or another CAD program.

CamStudio is an experimental design tool. Print a small test mechanism, verify clearances and
collisions, and keep hands clear of powered prototypes. A generated model is not a guarantee of
mechanical strength or safety.

## What you get

- a sphere-constrained 3D gait sketcher;
- an animated cam, groove, contact ball, pivot, leg, and foot preview;
- a measured X/Y/Z gait bounding box that responds to final leg length;
- guardrails for groove-to-shaft space and follower placement;
- CSV and plain XYZ TXT curve export;
- a smooth, editable STEP cam with flat sides and no attached shaft pin;
- automatic draft recovery after refreshing or reopening the usual local address.

## Quick start on macOS

1. On GitHub, choose **Code → Download ZIP**, then unzip the download.
2. Open the `CamStudio` folder.
3. Right-click **Launch CamStudio.command** and choose **Open**. Right-clicking is important the
   first time because the project is not an Apple-signed application.
4. Keep the Terminal window open while using CamStudio in the browser.

The first launch checks and builds the native STEP generator. If CMake or Open CASCADE is missing,
the launcher explains what is needed and can install it through [Homebrew](https://brew.sh).
The first setup therefore needs an internet connection and may take several minutes. Later starts
only rebuild files that changed.

CamStudio is currently tested on macOS. The double-click launcher does not support Windows.

## Design workflow

### 1. Draw the foot path

Draw one continuous loop around the orange outward marker on the sphere, then press **Close**.
The sphere radius is an approximate movement scale while sketching. You can export this curve as
CSV or TXT and share it independently of the cam.

Your active sketch and input values are saved automatically in the browser. Using the normal
`http://127.0.0.1:8765/` address lets CamStudio recover that draft on the next launch.

### 2. Build the mechanism

Choose **Continue to mechanism**, enter the physical dimensions, and press **Apply changes to
preview**. The dashed gait box reports the complete X, Y, and Z foot travel in millimetres.

The intended follower is one straight member:

`contact ball → fixed leg pivot → foot`

Final leg length scales the physical foot path without changing its shape. Doubling the leg length
doubles every gait-box dimension. It does not change the cam groove because the angular motion and
the short contact-ball-to-pivot segment remain the same.

Use **Play**, drag to orbit, or choose **Face view** to inspect the full cycle. The design check
reports the calculated cam envelope and blocks clearly unsafe parameter combinations.

### 3. Generate the cam

Choose **Generate CAM STEP** and wait for the success path at the bottom of the window. The design
is written under the folder below. After success, **Show output in Finder** selects the correct
STEP automatically.

```text
exports/sphere-sketches/<design-name>/
```

The folder contains:

- `cam-clean-disc.step` — the editable one-solid CAD cam;
- `cam-clean-disc-preview.stl` — a mesh preview/printing reference;
- `sphere-curve.csv` and `sphere-curve.txt` — the requested foot trajectory;
- `pitch-curve.csv` — the calculated groove centreline;
- `follower-motion.csv` — the angular motion table.

All files are staged privately and published together only after the STEP has been re-imported and
verified. If generation fails, an older successful STEP is not silently mixed with new curve files.

## Common problems

**The browser says “preview mode only.”** Close that tab and launch with
`Launch CamStudio.command`; opening `app/index.html` directly cannot run the STEP generator.

**macOS refuses to open the launcher.** Right-click the file, choose **Open**, then confirm **Open**
again. Do not double-click it for the first approval.

**The Terminal reports that Homebrew is missing.** Install it from
[brew.sh](https://brew.sh), then reopen the launcher.

**A value changed but the preview did not.** Press **Apply changes to preview**. Unapplied values
are deliberately marked so the displayed geometry and generated geometry cannot disagree.

**Fusion shows an older shape.** Generate again, wait for the full success message, and import the
new STEP into a new Fusion document. Fusion does not automatically refresh a previously imported
local file.

## Developer build and tests

Install the CAD kernel and configure a release build:

```sh
brew install cmake opencascade
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

The test suite covers trajectory parsing, spherical reconstruction, timing, invalid numerical
inputs, groove/shaft collisions, native STEP creation, and STEP re-import as one valid solid.
GitHub Actions repeats the complete build and test on macOS for every push and pull request.

The command-line interface remains available for reproducible development. Run
`./build/camstudio` without arguments to see its commands.

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the code structure and
[docs/PUBLISHING.md](docs/PUBLISHING.md) for the repository release checklist.

## Research origin and license

CamStudio modernizes behavior from the MIT-licensed
[3DCamFollower](https://github.com/chengcno/3DCamFollower) research prototype and the paper
“Spatial-Temporal Motion Control via Composite Cam-follower Mechanisms” by Cheng, Sun, Song, and
Liu (ACM Transactions on Graphics, 2021). Please cite that work when using this software in
scientific research.

CamStudio is distributed under the [MIT License](LICENSE). Original attribution and dependency
information are retained in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
