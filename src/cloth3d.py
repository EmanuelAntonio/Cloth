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

    def __post_init__(self) -> None:
        if self.spring_k <= 0:
            raise ValueError("spring_k must be positive")
        if self.time_step <= 0:
            raise ValueError("time_step must be positive")

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

        # Apply constraints
        for idx in range(self.mesh.n_vertices):
            for axis in range(3):
                if not self.mesh.axis_is_free(idx, axis):
                    positions[idx, axis] = self.mesh.initial_positions[idx, axis]
                    self.velocities[idx, axis] = 0.0
