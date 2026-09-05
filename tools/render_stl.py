#!/usr/bin/env python3
"""Small dependency-light STL preview renderer used for CAD export verification."""

import argparse
import struct
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


def read_binary_stl(path):
    with Path(path).open("rb") as stream:
        stream.read(80)
        count = struct.unpack("<I", stream.read(4))[0]
        raw = stream.read(count * 50)
    dtype = np.dtype([
        ("normal", "<f4", 3),
        ("vertices", "<f4", (3, 3)),
        ("attribute", "<u2"),
    ])
    triangles = np.frombuffer(raw, dtype=dtype, count=count)
    return triangles["vertices"].astype(np.float64)


def read_stl(path):
    path = Path(path)
    with path.open("rb") as stream:
        header = stream.read(5)
    if header.lower() != b"solid":
        return read_binary_stl(path)
    vertices = []
    with path.open("r", encoding="ascii", errors="strict") as stream:
        for line in stream:
            columns = line.split()
            if columns and columns[0] == "vertex":
                vertices.append([float(value) for value in columns[1:4]])
    if len(vertices) % 3 != 0:
        raise ValueError("ASCII STL contains an incomplete triangle")
    return np.asarray(vertices, dtype=np.float64).reshape((-1, 3, 3))


def read_obj(path):
    vertices = []
    faces = []
    with Path(path).open("r", encoding="utf-8", errors="strict") as stream:
        for line in stream:
            columns = line.split()
            if not columns:
                continue
            if columns[0] == "v":
                vertices.append([float(value) for value in columns[1:4]])
            elif columns[0] == "f":
                face = [int(value.split("/")[0]) - 1 for value in columns[1:]]
                for index in range(1, len(face) - 1):
                    faces.append([face[0], face[index], face[index + 1]])
    return np.asarray(vertices, dtype=np.float64)[np.asarray(faces, dtype=np.int64)]


def read_mesh(path):
    return read_obj(path) if Path(path).suffix.lower() == ".obj" else read_stl(path)


def unit(vector):
    magnitude = np.linalg.norm(vector)
    return vector / magnitude


def render(triangles, output, view_direction, width=1200, height=900):
    center = 0.5 * (triangles.min(axis=(0, 1)) + triangles.max(axis=(0, 1)))
    points = triangles - center
    forward = unit(np.asarray(view_direction, dtype=float))
    world_up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(forward, world_up)) > 0.95:
        world_up = np.array([0.0, 1.0, 0.0])
    right = unit(np.cross(world_up, forward))
    up = unit(np.cross(forward, right))
    basis = np.stack([right, up, forward], axis=1)
    camera = points @ basis

    xy = camera[:, :, :2]
    minimum = xy.min(axis=(0, 1))
    maximum = xy.max(axis=(0, 1))
    extent = np.maximum(maximum - minimum, 1e-9)
    scale = 0.84 * min(width / extent[0], height / extent[1])
    projected = (xy - 0.5 * (minimum + maximum)) * scale
    projected[:, :, 0] += width * 0.5
    projected[:, :, 1] = height * 0.5 - projected[:, :, 1]

    edges_a = camera[:, 1] - camera[:, 0]
    edges_b = camera[:, 2] - camera[:, 0]
    normals = np.cross(edges_a, edges_b)
    lengths = np.linalg.norm(normals, axis=1)
    valid = lengths > 1e-12
    normals[valid] /= lengths[valid, None]
    light = unit(np.array([-0.35, 0.45, 0.82]))
    brightness = 0.28 + 0.72 * np.abs(normals @ light)
    depths = camera[:, :, 2].mean(axis=1)

    image = Image.new("RGB", (width, height), (247, 248, 250))
    draw = ImageDraw.Draw(image)
    for index in np.argsort(depths):
        shade = int(185 * brightness[index] + 45)
        color = (shade, int(shade * 0.88), int(shade * 0.55))
        polygon = [tuple(value) for value in projected[index]]
        draw.polygon(polygon, fill=color)
    image.save(output)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("stl")
    parser.add_argument("output")
    parser.add_argument("--view", nargs=3, type=float, default=(1.0, -1.0, 0.7))
    arguments = parser.parse_args()
    render(read_mesh(arguments.stl), arguments.output, arguments.view)


if __name__ == "__main__":
    main()
