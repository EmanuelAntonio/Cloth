"""Mass-spring cloth simulation."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from mesh3d import Mesh3D


@dataclass
class Cloth3D:
    mesh: Mesh3D
    spring_k: float
    time_step: float
    mass: float = 1.0
    damping: float = 0.02
    gravity: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, -9.81]))
    max_stretch_ratio: float | None = None

    def __post_init__(self) -> None:
        if self.spring_k <= 0:
            raise ValueError("spring_k must be positive")
        if self.time_step <= 0:
            raise ValueError("time_step must be positive")
        if self.max_stretch_ratio is not None and self.max_stretch_ratio < 1.0:
            raise ValueError("max_stretch_ratio must be >= 1.0 or None")

        self.velocities = np.zeros_like(self.mesh.positions)
        self.rest_lengths = np.array([
            np.linalg.norm(self.mesh.positions[j] - self.mesh.positions[i])
            for i, j in self.mesh.edges
        ])

    def compute(self) -> None:
        """Advance the simulation by one time step."""
        positions = self.mesh.positions
        forces = np.zeros_like(positions)

        # Spring forces
        for edge_index, (i, j) in enumerate(self.mesh.edges):
            delta = positions[j] - positions[i]
            length = np.linalg.norm(delta)
            if length == 0:
                continue
            direction = delta / length
            rest = self.rest_lengths[edge_index]
            force_magnitude = self.spring_k * (length - rest)
            force = force_magnitude * direction
            forces[i] += force
            forces[j] -= force

        # Gravity
        for idx in range(self.mesh.n_vertices):
            if self.mesh.axis_is_free(idx, 2):
                forces[idx] += self.mass * self.gravity

        # Integrate (semi-implicit Euler)
        accelerations = forces / self.mass
        self.velocities += accelerations * self.time_step
        self.velocities *= (1.0 - self.damping)
        positions[:] += self.velocities * self.time_step

        self._enforce_max_stretch()

        # Apply constraints
        for idx in range(self.mesh.n_vertices):
            for axis in range(3):
                if not self.mesh.axis_is_free(idx, axis):
                    positions[idx, axis] = self.mesh.initial_positions[idx, axis]
                    self.velocities[idx, axis] = 0.0

    def _enforce_max_stretch(self) -> None:
        """Clamp edge lengths to the configured stretch ratio."""

        if self.max_stretch_ratio is None:
            return

        ratio = self.max_stretch_ratio
        positions = self.mesh.positions

        for edge_index, (i, j) in enumerate(self.mesh.edges):
            rest = self.rest_lengths[edge_index]
            max_length = rest * ratio
            delta = positions[j] - positions[i]
            length = np.linalg.norm(delta)
            if length <= max_length or length == 0.0:
                continue

            direction = delta / length
            correction_mag = length - max_length

            free_i = self.mesh.free_dof[i]
            free_j = self.mesh.free_dof[j]
            has_free_i = bool(np.any(free_i))
            has_free_j = bool(np.any(free_j))

            if not has_free_i and not has_free_j:
                continue

            if has_free_i and has_free_j:
                share_i = share_j = 0.5
            elif has_free_i:
                share_i, share_j = 1.0, 0.0
            else:
                share_i, share_j = 0.0, 1.0

            move_i = direction * (share_i * correction_mag)
            move_j = -direction * (share_j * correction_mag)

            move_i = np.where(free_i, move_i, 0.0)
            move_j = np.where(free_j, move_j, 0.0)

            positions[i] += move_i
            positions[j] += move_j

            mask_i = np.abs(move_i) > 1e-9
            mask_j = np.abs(move_j) > 1e-9
            if np.any(mask_i):
                self.velocities[i, mask_i] = 0.0
            if np.any(mask_j):
                self.velocities[j, mask_j] = 0.0
