"""Entry point for the cloth simulation demo."""
from __future__ import annotations

import argparse
import pathlib
import sys

import numpy as np

_MODULE_DIR = pathlib.Path(__file__).resolve().parent
if str(_MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(_MODULE_DIR))

from cloth3d import Cloth3D, SphereCollider
from draw3d import Draw3D
from mesh3d import Mesh3D


def create_square_mesh(size: float, subdivisions: int) -> Mesh3D:
    return Mesh3D.build_square(size=size, subdivisions=subdivisions)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="3D cloth simulation demo")
    parser.add_argument("--size", type=float, default=4.0, help="Side length of the square cloth")
    parser.add_argument("--subdivisions", type=int, default=20, help="Number of subdivisions per side")
    parser.add_argument("--spring", type=float, default=2000.0, help="Spring constant")
    parser.add_argument("--timestep", type=float, default=1 / 120.0, help="Integration time step")
    parser.add_argument(
        "--max-stretch",
        type=float,
        default=None,
        help="Optional maximum stretch ratio (>=1.0) applied to each spring",
    )
    parser.add_argument(
        "--scenario",
        choices=("cloth", "sphere"),
        default="sphere",
        help="Choose between the classic fixed-corner cloth or the sphere drop",
    )
    parser.add_argument(
        "--sphere-radius",
        type=float,
        default=0.6,
        help="Radius of the static collider sphere (sphere scenario)",
    )
    parser.add_argument(
        "--drop-height",
        type=float,
        default=0.8,
        help="Extra height above the sphere surface for the cloth start",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    mesh = create_square_mesh(size=args.size, subdivisions=args.subdivisions)
    colliders: list[SphereCollider] = []

    if args.scenario == "sphere":
        grid_n = args.subdivisions + 1
        corner_indices = [0, grid_n - 1, grid_n * (grid_n - 1), grid_n * grid_n - 1]
        for corner in corner_indices:
            mesh.release_vertex(corner, axes=(0, 1, 2), permanent=True)

        drop_offset = args.sphere_radius + args.drop_height
        translation = np.array([0.0, 0.0, drop_offset], dtype=np.float64)
        mesh.positions += translation
        mesh.initial_positions += translation

        colliders.append(SphereCollider(center=np.zeros(3, dtype=np.float64), radius=args.sphere_radius))

    cloth = Cloth3D(
        mesh=mesh,
        spring_k=args.spring,
        time_step=args.timestep,
        max_stretch_ratio=args.max_stretch,
        colliders=colliders,
    )
    viewer = Draw3D(mesh=mesh, cloth=cloth)
    viewer.run()


if __name__ == "__main__":
    main()
