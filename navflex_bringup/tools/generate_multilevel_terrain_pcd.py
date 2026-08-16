#!/usr/bin/env python3
"""Generate a 0.1 m sampled PCD terrain with a ramp and stairs."""

from pathlib import Path


STEP = 0.1
POINTS = set()
OUTER_WALL_HEIGHT = 3.0
CORRIDOR_WALL_HEIGHT = 2.0
CORRIDOR_WALL_THICKNESS = 0.2
STRUCTURE_HEIGHT = 1.0


def add_point(x, y, z):
    POINTS.add((round(x, 4), round(y, 4), round(z, 4)))


def samples(start, stop, step=STEP):
    count = round((stop - start) / step)
    return [start + index * step for index in range(count + 1)]


def add_horizontal(x_min, x_max, y_min, y_max, z):
    for x in samples(x_min, x_max):
        for y in samples(y_min, y_max):
            add_point(x, y, z)


def add_vertical_x(x, y_min, y_max, z_min, z_max):
    for y in samples(y_min, y_max):
        for z in samples(z_min, z_max):
            add_point(x, y, z)


def add_vertical_y(y, x_min, x_max, z_min, z_max):
    for x in samples(x_min, x_max):
        for z in samples(z_min, z_max):
            add_point(x, y, z)


def inside_structure(x, y):
    return -3.0 <= x <= 2.0 and -4.0 <= y <= -1.0


def add_floor():
    for x in samples(-4.9, 4.9):
        for y in samples(-4.9, 4.9):
            if not inside_structure(x, y):
                add_point(x, y, 0.0)


def add_outer_walls():
    add_vertical_x(-5.0, -5.0, 5.0, 0.0, OUTER_WALL_HEIGHT)
    add_vertical_x(5.0, -5.0, 5.0, 0.0, OUTER_WALL_HEIGHT)
    add_vertical_y(-5.0, -5.0, 5.0, 0.0, OUTER_WALL_HEIGHT)
    add_vertical_y(5.0, -5.0, 5.0, 0.0, OUTER_WALL_HEIGHT)


def add_corridor_walls(x_min, x_max, y_min, y_max):
    for wall_min, wall_max in (
        (y_min, y_min + CORRIDOR_WALL_THICKNESS),
        (y_max - CORRIDOR_WALL_THICKNESS, y_max),
    ):
        for x in samples(x_min, x_max):
            for y in samples(wall_min, wall_max):
                for z in samples(0.0, CORRIDOR_WALL_HEIGHT):
                    add_point(x, y, z)


def corridor_interior(y_min, y_max):
    return (
        y_min + CORRIDOR_WALL_THICKNESS,
        y_max - CORRIDOR_WALL_THICKNESS,
    )


def add_connected_ramp_and_stairs():
    y_min, y_max = -4.0, -1.0
    terrain_y_min, terrain_y_max = corridor_interior(y_min, y_max)
    rise_start, platform_start, platform_end, stairs_end = -3.0, -1.0, 0.0, 2.0
    for x in samples(rise_start, platform_start):
        z = STRUCTURE_HEIGHT * (x - rise_start) / (platform_start - rise_start)
        for y in samples(terrain_y_min, terrain_y_max):
            add_point(x, y, z)
    add_horizontal(platform_start, platform_end, terrain_y_min, terrain_y_max, STRUCTURE_HEIGHT)
    tread, riser, count = 0.2, 0.1, 10
    for index in range(count):
        x_min = platform_end + index * tread
        x_max = x_min + tread
        z = STRUCTURE_HEIGHT - (index + 1) * riser
        add_vertical_x(x_min, terrain_y_min, terrain_y_max, z, z + riser)
        add_horizontal(x_min, x_max, terrain_y_min, terrain_y_max, z)
    add_corridor_walls(rise_start, stairs_end, y_min, y_max)


def write_pcd(path):
    points = sorted(POINTS)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii") as output:
        output.write("# .PCD v0.7 - Point Cloud Data file format\n")
        output.write(
            "# 10m x 10m terrain, 0.1m sampling, 3m outer walls, "
            "connected 1m ramp/stairs, 1m platform\n")
        output.write("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        output.write(f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n")
        output.write(f"POINTS {len(points)}\nDATA ascii\n")
        for x, y, z in points:
            output.write(f"{x:.4f} {y:.4f} {z:.4f}\n")


def main():
    add_floor()
    add_outer_walls()
    add_connected_ramp_and_stairs()
    output = Path(__file__).resolve().parents[1] / "maps" / "multilevel_ramp_stairs_0p1m.pcd"
    write_pcd(output)
    print(f"Wrote {len(POINTS)} points to {output}")


if __name__ == "__main__":
    main()
