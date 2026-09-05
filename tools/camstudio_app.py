#!/usr/bin/env python3
"""Local CamStudio sphere-sketching UI and STEP-generation bridge."""

from __future__ import annotations

import csv
import json
import math
import os
import re
import subprocess
import tempfile
import threading
import webbrowser
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse


ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "app"
EXPORTS = ROOT / "exports" / "sphere-sketches"
BINARY = ROOT / "build" / "camstudio"
PREFERRED_PORT = 8765
MAXIMUM_COORDINATE_MM = 100_000.0


class CamStudioHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(APP), **kwargs)

    def log_message(self, format_string: str, *args: object) -> None:
        print(f"CamStudio: {format_string % args}")

    def send_json(self, status: int, payload: dict[str, object]) -> None:
        data = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self) -> None:  # noqa: N802 - required by BaseHTTPRequestHandler
        if urlparse(self.path).path == "/api/status":
            self.send_json(200, {
                "ready": BINARY.exists(),
                "generator": str(BINARY),
                "version": "0.1.0",
            })
            return
        super().do_GET()

    def do_POST(self) -> None:  # noqa: N802 - required by BaseHTTPRequestHandler
        route = urlparse(self.path).path
        if route == "/api/reveal":
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0 or length > 4_000:
                    raise ValueError("Invalid output-folder request")
                request = json.loads(self.rfile.read(length))
                if not isinstance(request, dict):
                    raise ValueError("Output-folder request must be a JSON object")
                name = re.sub(
                    r"[^A-Za-z0-9_-]+", "-", str(request.get("name", ""))
                ).strip("-")[:80]
                target = EXPORTS / name / "cam-clean-disc.step"
                if not name or not target.is_file():
                    raise ValueError("Generate the STEP successfully before opening its folder")
                process = subprocess.run(
                    ["open", "-R", str(target)], capture_output=True, text=True,
                    timeout=15, check=False,
                )
                if process.returncode != 0:
                    raise ValueError("Finder could not reveal the generated STEP")
                self.send_json(200, {"revealed": str(target)})
            except (ValueError, TypeError, json.JSONDecodeError, subprocess.TimeoutExpired) as error:
                self.send_json(400, {"error": str(error)})
            return
        if route != "/api/generate":
            self.send_json(404, {"error": "Unknown CamStudio action"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > 8_000_000:
                raise ValueError("Invalid sketch data size")
            request = json.loads(self.rfile.read(length))
            if not isinstance(request, dict):
                raise ValueError("Generation request must be a JSON object")
            name = re.sub(r"[^A-Za-z0-9_-]+", "-", str(request.get("name", "sphere-curve"))).strip("-")[:80]
            if not name:
                name = "sphere-curve"
            points = request.get("points")
            pivot = request.get("pivot")
            joint = request.get("joint")
            ball_radius = float(request.get("ballRadius", 2.5))
            clearance = float(request.get("clearance", 0.25))
            edge_margin = float(request.get("edgeMargin", 2.3))
            if not isinstance(points, list) or not 36 <= len(points) <= 1440:
                raise ValueError("A closed curve needs between 36 and 1440 output samples")
            if not isinstance(pivot, list) or len(pivot) != 3 or not isinstance(joint, list) or len(joint) != 3:
                raise ValueError("Sphere centre or follower joint is incomplete")
            if any(not isinstance(point, list) or len(point) != 3 for point in points):
                raise ValueError("Every curve point must have X, Y, and Z")
            numeric_points = [[float(value) for value in point] for point in points]
            pivot = [float(value) for value in pivot]
            joint = [float(value) for value in joint]
            all_numbers = [
                *[value for point in numeric_points for value in point],
                *pivot, *joint, ball_radius, clearance, edge_margin,
            ]
            if any(not math.isfinite(value) for value in all_numbers):
                raise ValueError("Dimensions and curve coordinates must be finite numbers")
            if any(abs(value) > MAXIMUM_COORDINATE_MM for value in all_numbers):
                raise ValueError("A dimension or coordinate exceeds the 100,000 mm safety limit")
            if ball_radius <= 0 or clearance < 0 or edge_margin <= 0:
                raise ValueError("Ball radius and edge thickness must be positive; clearance cannot be negative")
            if not BINARY.exists():
                raise ValueError("CamStudio must be built once before generating STEP")

            output = EXPORTS / name
            EXPORTS.mkdir(parents=True, exist_ok=True)
            # Generate every artifact in a private staging directory. A failed
            # CAD operation must never make newly written CSV files appear to
            # belong to an older STEP left in the named design folder.
            with tempfile.TemporaryDirectory(prefix=f".{name}-building-", dir=EXPORTS) as staging_name:
                staging = Path(staging_name)
                curve_path = staging / "sphere-curve.csv"
                with curve_path.open("w", newline="", encoding="utf-8") as file:
                    writer = csv.writer(file, lineterminator="\n")
                    writer.writerow(["index", "x_mm", "y_mm", "z_mm"])
                    for index, point in enumerate(numeric_points):
                        writer.writerow([index, *[f"{value:.9f}" for value in point]])

                curve_txt_path = staging / "sphere-curve.txt"
                with curve_txt_path.open("w", newline="", encoding="utf-8") as file:
                    file.write("# CamStudio spherical foot path\n# x_mm y_mm z_mm\n")
                    for point in numeric_points:
                        file.write(" ".join(f"{value:.9f}" for value in point) + "\n")

                command = [
                    str(BINARY), "cam-from-sphere", str(curve_path), str(staging),
                    *[f"{value:.9f}" for value in pivot],
                    *[f"{value:.9f}" for value in joint],
                    f"{ball_radius:.9f}", f"{clearance:.9f}", f"{edge_margin:.9f}",
                ]
                process = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=180, check=False)
                if process.returncode != 0:
                    detail = (process.stderr or process.stdout or "Cam generation failed").strip()
                    existing_note = (
                        f" No new STEP was created. The existing file at {output / 'cam-clean-disc.step'} "
                        "is from an earlier successful run."
                        if (output / "cam-clean-disc.step").exists()
                        else " No STEP file was created."
                    )
                    raise ValueError(detail + existing_note)

                artifact_names = [
                    "sphere-curve.csv", "sphere-curve.txt", "pitch-curve.csv",
                    "follower-motion.csv", "cam-clean-disc.step", "cam-clean-disc-preview.stl",
                ]
                missing = [filename for filename in artifact_names if not (staging / filename).exists()]
                if missing:
                    raise ValueError("Generation completed without expected files: " + ", ".join(missing))
                output.mkdir(parents=True, exist_ok=True)
                for filename in artifact_names:
                    os.replace(staging / filename, output / filename)

            curve_path = output / "sphere-curve.csv"
            curve_txt_path = output / "sphere-curve.txt"
            self.send_json(200, {
                "step": str(output / "cam-clean-disc.step"),
                "curve": str(curve_path),
                "curveTxt": str(curve_txt_path),
                "pitch": str(output / "pitch-curve.csv"),
                "motion": str(output / "follower-motion.csv"),
            })
        except (ValueError, TypeError, json.JSONDecodeError, subprocess.TimeoutExpired) as error:
            self.send_json(400, {"error": str(error)})
        except Exception as error:  # keep the local UI responsive with a useful error
            self.send_json(500, {"error": f"Unexpected generation error: {error}"})


def main() -> None:
    try:
        server = ThreadingHTTPServer(("127.0.0.1", PREFERRED_PORT), CamStudioHandler)
    except OSError:
        server = ThreadingHTTPServer(("127.0.0.1", 0), CamStudioHandler)
        print(f"Port {PREFERRED_PORT} is already in use; using a temporary local port.")
    url = f"http://127.0.0.1:{server.server_port}/"
    print(f"CamStudio is running at {url}")
    print("Keep this window open while using the sphere sketcher. Press Control-C to stop.")
    threading.Timer(0.4, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nCamStudio stopped.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
