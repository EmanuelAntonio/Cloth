"""Mesh representation for the cloth simulation."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, List, Tuple

import numpy as np


@dataclass
class Mesh3D:
    """Simple 3D mesh tailored for cloth simulations.

    The mesh stores vertex positions and a list of edges.  Each vertex has a mask
    that defines which translational degrees of freedom (x, y, z) are free.  This
    enables the simulation to constrain nodes individually along any axis.
    """

    positions: np.ndarray
    edges: List[Tuple[int, int]]
    free_dof: np.ndarray
    faces: List[Tuple[int, int, int]] = field(default_factory=list)
    initial_positions: np.ndarray = field(init=False)
    base_free_dof: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        self.positions = np.asarray(self.positions, dtype=np.float64)
        if self.positions.ndim != 2 or self.positions.shape[1] != 3:
            raise ValueError("positions must be an (N, 3) array")

        self.initial_positions = self.positions.copy()

        if len(self.edges) == 0:
            raise ValueError("At least one edge is required for the simulation")

        self.free_dof = np.asarray(self.free_dof, dtype=bool)
        if self.free_dof.shape != self.positions.shape:
            raise ValueError("free_dof must match the shape of positions")
        self.base_free_dof = self.free_dof.copy()

        # Faces are optional for simulation but required for shaded rendering.
        self.faces = [tuple(face) for face in self.faces]

    @property
    def n_vertices(self) -> int:
        return self.positions.shape[0]

    def fix_vertex(
        self,
        index: int,
        axes: Iterable[int] = (0, 1, 2),
        *,
        permanent: bool = True,
    ) -> None:
        """Constrains the vertex along the provided axes.

        Parameters
        ----------
        index:
            Index of the vertex to lock.
        axes:
            Iterable with the axis indices (0=x, 1=y, 2=z) to constrain.
        permanent:
            When ``True`` (default) the constraint is stored as part of the
            mesh's baseline state so that :meth:`reset` keeps the vertex fixed.
        """

        axes = tuple(axes)
        for axis in axes:
            self.free_dof[index, axis] = False
            self.positions[index, axis] = self.initial_positions[index, axis]
            if permanent:
                self.base_free_dof[index, axis] = False

    def release_vertex(
        self,
        index: int,
        axes: Iterable[int] = (0, 1, 2),
        *,
        permanent: bool = True,
    ) -> None:
        axes = tuple(axes)
        for axis in axes:
            self.free_dof[index, axis] = True
            if permanent:
                self.base_free_dof[index, axis] = True

    def axis_is_free(self, index: int, axis: int) -> bool:
        return bool(self.free_dof[index, axis])

    def clone(self) -> "Mesh3D":
        clone = Mesh3D(
            self.positions.copy(),
            list(self.edges),
            self.free_dof.copy(),
            list(self.faces),
        )
        clone.initial_positions = self.initial_positions.copy()
        clone.base_free_dof = self.base_free_dof.copy()
        return clone

    @staticmethod
    def build_square(size: float, subdivisions: int) -> "Mesh3D":
        """Builds a square cloth grid lying on the XY plane.

        The square is centered at the origin and subdivided into a regular grid.
        The four original corners are fully constrained (no movement).
        """

        if subdivisions < 1:
            raise ValueError("subdivisions must be >= 1")

        grid_n = subdivisions + 1
        step = size / subdivisions

        positions: List[Tuple[float, float, float]] = []
        free_dof: List[Tuple[bool, bool, bool]] = []
        edges: List[Tuple[int, int]] = []
        faces: List[Tuple[int, int, int]] = []

        # create vertices
        half = size / 2.0
        for iy in range(grid_n):
            for ix in range(grid_n):
                x = -half + ix * step
                y = -half + iy * step
                positions.append((x, y, 0.0))
                free_dof.append([True, True, True])

        # create structural edges (horizontal and vertical)
        def idx(ix: int, iy: int) -> int:
            return iy * grid_n + ix

        for iy in range(grid_n):
            for ix in range(grid_n):
                if ix + 1 < grid_n:
                    edges.append((idx(ix, iy), idx(ix + 1, iy)))
                if iy + 1 < grid_n:
                    edges.append((idx(ix, iy), idx(ix, iy + 1)))
                # add shear (diagonal) springs for stability
                if ix + 1 < grid_n and iy + 1 < grid_n:
                    edges.append((idx(ix, iy), idx(ix + 1, iy + 1)))
                if ix > 0 and iy + 1 < grid_n:
                    edges.append((idx(ix, iy), idx(ix - 1, iy + 1)))

                if ix + 1 < grid_n and iy + 1 < grid_n:
                    v0 = idx(ix, iy)
                    v1 = idx(ix + 1, iy)
                    v2 = idx(ix, iy + 1)
                    v3 = idx(ix + 1, iy + 1)
                    faces.append((v0, v1, v2))
                    faces.append((v2, v1, v3))

        mesh = Mesh3D(
            np.array(positions),
            edges,
            np.array(free_dof, dtype=bool),
            faces,
        )

        # Fix the four corners of the original square completely.
        corners = [idx(0, 0), idx(grid_n - 1, 0), idx(0, grid_n - 1), idx(grid_n - 1, grid_n - 1)]
        for corner in corners:
            mesh.fix_vertex(corner, axes=(0, 1, 2))

        return mesh

    def reset(self) -> None:
        self.positions[:] = self.initial_positions
        self.free_dof[:, :] = self.base_free_dof

    def compute_vertex_normals(self) -> np.ndarray:
        """Compute per-vertex normals from the available triangle faces."""

        normals = np.zeros_like(self.positions)
        for face in self.faces:
            i0, i1, i2 = face
            p0, p1, p2 = self.positions[[i0, i1, i2]]
            edge1 = p1 - p0
            edge2 = p2 - p0
            face_normal = np.cross(edge1, edge2)
            norm = np.linalg.norm(face_normal)
            if norm == 0:
                continue
            face_normal /= norm
            normals[i0] += face_normal
            normals[i1] += face_normal
            normals[i2] += face_normal

        norms = np.linalg.norm(normals, axis=1)
        nonzero = norms > 0
        normals[nonzero] /= norms[nonzero][:, None]
        # Default normal for isolated vertices.
        normals[~nonzero] = np.array([0.0, 0.0, 1.0])
        return normals


