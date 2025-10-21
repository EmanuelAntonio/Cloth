"""Entry point for the cloth simulation demo."""
from __future__ import annotations

import argparse

from .cloth3d import Cloth3D
from .draw3d import Draw3D
from .mesh3d import Mesh3D


def create_square_mesh(size: float, subdivisions: int) -> Mesh3D:
    return Mesh3D.build_square(size=size, subdivisions=subdivisions)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="3D cloth simulation demo")
    parser.add_argument("--size", type=float, default=2.0, help="Side length of the square cloth")
    parser.add_argument("--subdivisions", type=int, default=10, help="Number of subdivisions per side")
    parser.add_argument("--spring", type=float, default=800.0, help="Spring constant")
    parser.add_argument("--timestep", type=float, default=1 / 120.0, help="Integration time step")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    mesh = create_square_mesh(size=args.size, subdivisions=args.subdivisions)
    cloth = Cloth3D(mesh=mesh, spring_k=args.spring, time_step=args.timestep)
    viewer = Draw3D(mesh=mesh, cloth=cloth)
    viewer.run()


if __name__ == "__main__":
    main()
