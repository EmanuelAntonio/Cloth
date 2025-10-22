"""Mass-spring cloth simulation."""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from mesh3d import Mesh3D


@dataclass
class SphereCollider:
    center: np.ndarray
    radius: float

    def __post_init__(self) -> None:
        self.center = np.asarray(self.center, dtype=np.float64)
        if self.center.shape != (3,):
            raise ValueError("center must be a 3D vector")
        if self.radius <= 0:
            raise ValueError("radius must be positive")


@dataclass
class Cloth3D:
    mesh: Mesh3D
    spring_k: float
    time_step: float
    mass: float = 1.0
    damping: float = 0.02
    gravity: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, -9.81]))
    max_stretch_ratio: float | None = None
    max_stretch_relaxation: float = 0.2
    colliders: list[SphereCollider] = field(default_factory=list)
    self_collision_distance: float = 0.05
    self_collision_iterations: int = 1

    def __post_init__(self) -> None:
        if self.spring_k <= 0:
            raise ValueError("spring_k must be positive")
        if self.time_step <= 0:
            raise ValueError("time_step must be positive")
        if self.max_stretch_ratio is not None and self.max_stretch_ratio < 1.0:
            raise ValueError("max_stretch_ratio must be >= 1.0 or None")
        if not 0.0 < self.max_stretch_relaxation <= 1.0:
            raise ValueError("max_stretch_relaxation must be in the range (0, 1]")
        if self.self_collision_distance < 0.0:
            raise ValueError("self_collision_distance must be >= 0")
        if self.self_collision_iterations < 0:
            raise ValueError("self_collision_iterations must be >= 0")

        self.velocities = np.zeros_like(self.mesh.positions)
        self.rest_lengths = np.array([
            np.linalg.norm(self.mesh.positions[j] - self.mesh.positions[i])
            for i, j in self.mesh.edges
        ])
        self.max_stretch_relaxation = float(self.max_stretch_relaxation)
        self._edge_lookup = {
            (i, j) if i < j else (j, i)
            for i, j in self.mesh.edges
        }

        self._self_collision_distance = float(self.self_collision_distance)
        self._self_collision_distance_sq = self._self_collision_distance ** 2

    # ------------------------------------------------------------------

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
        self._resolve_self_collisions()
        self._resolve_collisions()

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
        relaxation = self.max_stretch_relaxation
        positions = self.mesh.positions

        for edge_index, (i, j) in enumerate(self.mesh.edges):
            rest = self.rest_lengths[edge_index]
            max_length = rest * ratio
            delta = positions[j] - positions[i]
            length = np.linalg.norm(delta)
            if length <= max_length or length == 0.0:
                continue

            direction = delta / length
            correction_mag = (length - max_length) * relaxation

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

    def _resolve_collisions(self) -> None:
        """Resolve collisions against registered colliders."""

        if not self.colliders:
            return

        positions = self.mesh.positions

        for collider in self.colliders:
            center = collider.center
            radius = collider.radius
            for idx in range(self.mesh.n_vertices):
                free_mask = self.mesh.free_dof[idx]
                if not np.any(free_mask):
                    continue

                offset = positions[idx] - center
                distance = np.linalg.norm(offset)
                if distance >= radius:
                    continue

                if distance < 1e-8:
                    normal = np.array([0.0, 0.0, 1.0])
                else:
                    normal = offset / distance

                target_position = center + normal * radius
                correction = target_position - positions[idx]
                correction = np.where(free_mask, correction, 0.0)

                if not np.any(np.abs(correction) > 1e-12):
                    continue

                positions[idx] += correction

                normal_velocity = np.dot(self.velocities[idx], normal)
                if normal_velocity < 0.0:
                    self.velocities[idx] -= normal_velocity * normal

    def _resolve_self_collisions(self) -> None:
        """Push apart vertices that get too close to each other."""

        if (
            self._self_collision_distance <= 0.0
            or self.self_collision_iterations <= 0
        ):
            return

        positions = self.mesh.positions
        free_dof = self.mesh.free_dof
        radius = self._self_collision_distance
        radius_sq = self._self_collision_distance_sq
        cell_size = radius
        if cell_size <= 0.0:
            return
        inv_cell = 1.0 / cell_size

        for _ in range(self.self_collision_iterations):
            grid: dict[tuple[int, int, int], list[int]] = {}
            for idx, pos in enumerate(positions):
                cell = tuple(np.floor(pos * inv_cell).astype(int))
                grid.setdefault(cell, []).append(idx)

            for idx, pos in enumerate(positions):
                cell = tuple(np.floor(pos * inv_cell).astype(int))
                cx, cy, cz = cell
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        for dz in (-1, 0, 1):
                            neighbour_cell = (cx + dx, cy + dy, cz + dz)
                            neighbours = grid.get(neighbour_cell)
                            if not neighbours:
                                continue
                            for other in neighbours:
                                if other <= idx:
                                    continue
                                key = (idx, other) if idx < other else (other, idx)
                                if key in self._edge_lookup:
                                    continue

                                delta = positions[other] - pos
                                dist_sq = float(np.dot(delta, delta))
                                if dist_sq >= radius_sq:
                                    continue

                                if dist_sq == 0.0:
                                    normal = np.array([0.0, 0.0, 1.0])
                                    dist = 0.0
                                else:
                                    dist = math.sqrt(dist_sq)
                                    normal = delta / dist

                                penetration = radius - dist
                                if penetration <= 0.0:
                                    continue

                                free_i = free_dof[idx]
                                free_j = free_dof[other]
                                has_i = bool(np.any(free_i))
                                has_j = bool(np.any(free_j))
                                if not has_i and not has_j:
                                    continue

                                if has_i and has_j:
                                    share_i = share_j = 0.5
                                elif has_i:
                                    share_i, share_j = 1.0, 0.0
                                else:
                                    share_i, share_j = 0.0, 1.0

                                correction_i = -normal * (share_i * penetration)
                                correction_j = normal * (share_j * penetration)
                                correction_i = np.where(free_i, correction_i, 0.0)
                                correction_j = np.where(free_j, correction_j, 0.0)

                                if np.any(correction_i):
                                    positions[idx] += correction_i
                                    vel_i = self.velocities[idx]
                                    normal_component_i = np.dot(vel_i, normal)
                                    vel_i = vel_i - normal_component_i * normal
                                    self.velocities[idx] = np.where(
                                        free_i, vel_i, self.velocities[idx]
                                    )

                                if np.any(correction_j):
                                    positions[other] += correction_j
                                    vel_j = self.velocities[other]
                                    normal_component_j = np.dot(vel_j, normal)
                                    vel_j = vel_j - normal_component_j * normal
                                    self.velocities[other] = np.where(
                                        free_j, vel_j, self.velocities[other]
                                    )
