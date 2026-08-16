#!/usr/bin/env python3
"""Render the bundled PCD terrain as a top-down maximum-height image."""

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np


def load_heights(path):
    heights = {}
    data_started = False
    with path.open("r", encoding="ascii") as input_file:
        for line in input_file:
            if not data_started:
                data_started = line.strip() == "DATA ascii"
                continue
            values = line.split()
            if len(values) != 3:
                continue
            x, y, z = (float(value) for value in values)
            key = (round(x, 4), round(y, 4))
            heights[key] = max(heights.get(key, float("-inf")), z)
    if not heights:
        raise ValueError(f"No ASCII XYZ points found in {path}")
    return heights


def render(input_path, output_path):
    heights = load_heights(input_path)
    x_values = sorted({point[0] for point in heights})
    y_values = sorted({point[1] for point in heights})
    step = min(
        x_values[index + 1] - x_values[index]
        for index in range(len(x_values) - 1)
        if x_values[index + 1] > x_values[index])
    x_index = {value: index for index, value in enumerate(x_values)}
    y_index = {value: index for index, value in enumerate(y_values)}
    grid = np.full((len(y_values), len(x_values)), np.nan)
    for (x, y), z in heights.items():
        grid[y_index[y], x_index[x]] = z

    cmap = plt.get_cmap("turbo").copy()
    cmap.set_bad("white")
    figure, axis = plt.subplots(figsize=(9, 8), constrained_layout=True)
    image = axis.imshow(
        grid,
        origin="lower",
        interpolation="nearest",
        cmap=cmap,
        vmin=0.0,
        vmax=3.0,
        extent=(
            x_values[0] - step * 0.5,
            x_values[-1] + step * 0.5,
            y_values[0] - step * 0.5,
            y_values[-1] + step * 0.5,
        ),
    )
    axis.set_aspect("equal")
    axis.set_xlabel("X (m)")
    axis.set_ylabel("Y (m)")
    axis.set_title("Multilevel Ramp and Stairs PCD: Top-Down Maximum Height")
    axis.grid(color="white", alpha=0.25, linewidth=0.5)
    for x, label in ((-2.0, "Ramp"), (-0.5, "Platform"), (1.0, "Stairs")):
        axis.text(
            x, -2.55, label, color="white", fontsize=9, ha="center", va="center",
            bbox={"facecolor": "black", "alpha": 0.55, "pad": 2, "edgecolor": "none"},
        )
    colorbar = figure.colorbar(image, ax=axis, shrink=0.84)
    colorbar.set_label("Maximum point height (m)")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def main():
    maps_dir = Path(__file__).resolve().parents[1] / "maps"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        type=Path,
        default=maps_dir / "multilevel_ramp_stairs_0p1m.pcd",
        help="Input ASCII PCD file",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=maps_dir / "multilevel_ramp_stairs_0p1m_topdown.png",
        help="Output PNG file",
    )
    arguments = parser.parse_args()
    render(arguments.input, arguments.output)
    print(f"Wrote top-down PCD image to {arguments.output}")


if __name__ == "__main__":
    main()
