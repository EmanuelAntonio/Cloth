"""OpenGL visualization utilities for the cloth simulation."""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
from OpenGL.GL import (
    glBegin,
    glClear,
    glClearColor,
    glColor3f,
    glEnable,
    glEnd,
    glLineWidth,
    glLoadIdentity,
    glMatrixMode,
    glPointSize,
    glViewport,
    glVertex3f,
    GL_COLOR_BUFFER_BIT,
    GL_DEPTH_BUFFER_BIT,
    GL_DEPTH_TEST,
    GL_LINES,
    GL_MODELVIEW,
    GL_POINTS,
    GL_PROJECTION,
)
from OpenGL.GLU import gluLookAt, gluPerspective
from OpenGL.GLUT import (
    GLUT_DEPTH,
    GLUT_DOUBLE,
    GLUT_LEFT_BUTTON,
    GLUT_RGB,
    GLUT_UP,
    glutCreateWindow,
    glutDisplayFunc,
    glutIdleFunc,
    glutInit,
    glutInitDisplayMode,
    glutInitWindowPosition,
    glutInitWindowSize,
    glutMainLoop,
    glutMotionFunc,
    glutMouseFunc,
    glutPostRedisplay,
    glutReshapeFunc,
    glutSwapBuffers,
    glutWMCloseFunc,
)

from .cloth3d import Cloth3D
from .mesh3d import Mesh3D


@dataclass
class Draw3D:
    mesh: Mesh3D
    cloth: Cloth3D
    window_size: Tuple[int, int] = (960, 720)
    fov_y: float = 45.0

    phi: float = math.pi / 4.0
    theta: float = math.pi / 4.0
    radius: float = 4.0
    center: np.ndarray = np.zeros(3)

    _left_button_down: bool = False
    _last_mouse_pos: Optional[Tuple[int, int]] = None

    def __post_init__(self) -> None:
        if isinstance(self.center, np.ndarray):
            center = self.center
        else:
            center = np.asarray(self.center, dtype=np.float64)

        if not np.any(center):
            center = self.mesh.positions.mean(axis=0)

        self.center = center.astype(np.float64)

    # -- OpenGL setup -----------------------------------------------------
    def run(self) -> None:
        glutInit()
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
        glutInitWindowSize(*self.window_size)
        glutInitWindowPosition(100, 100)
        glutCreateWindow(b"Cloth Simulation")

        glClearColor(0.1, 0.1, 0.1, 1.0)
        glEnable(GL_DEPTH_TEST)

        glutDisplayFunc(self.display)
        glutIdleFunc(self.idle)
        glutReshapeFunc(self.reshape)
        glutMouseFunc(self.mouse_button)
        glutMotionFunc(self.mouse_motion)
        glutWMCloseFunc(lambda: sys.exit(0))

        glutMainLoop()

    # -- Camera handling --------------------------------------------------
    def _compute_camera(self) -> Tuple[np.ndarray, np.ndarray]:
        theta = np.clip(self.theta, 1e-3, math.pi - 1e-3)
        phi = self.phi
        r = max(self.radius, 0.1)

        sin_theta = math.sin(theta)
        eye = np.array(
            [
                self.center[0] + r * sin_theta * math.cos(phi),
                self.center[1] + r * math.cos(theta),
                self.center[2] + r * sin_theta * math.sin(phi),
            ],
            dtype=np.float64,
        )

        forward = self.center - eye
        forward /= np.linalg.norm(forward)

        world_up = np.array([0.0, 1.0, 0.0])
        right = np.cross(forward, world_up)
        if np.linalg.norm(right) < 1e-6:
            world_up = np.array([0.0, 0.0, 1.0])
            right = np.cross(forward, world_up)
        right /= np.linalg.norm(right)
        up = np.cross(right, forward)
        up /= np.linalg.norm(up)
        return eye, up

    def mouse_button(self, button: int, state: int, x: int, y: int) -> None:
        if button == GLUT_LEFT_BUTTON:
            self._left_button_down = state != GLUT_UP
            self._last_mouse_pos = (x, y)
        elif state == GLUT_UP:
            self._last_mouse_pos = None

        # Scroll wheel (zoom) is encoded as buttons 3 (up) and 4 (down) in GLUT
        if button == 3 and state != GLUT_UP:
            self.radius = max(0.2, self.radius * 0.9)
        elif button == 4 and state != GLUT_UP:
            self.radius = min(100.0, self.radius * 1.1)

        glutPostRedisplay()

    def mouse_motion(self, x: int, y: int) -> None:
        if not self._left_button_down or self._last_mouse_pos is None:
            return

        dx = x - self._last_mouse_pos[0]
        dy = y - self._last_mouse_pos[1]
        self._last_mouse_pos = (x, y)

        sensitivity = 0.005
        self.phi += dx * sensitivity
        self.theta += dy * sensitivity
        self.theta = np.clip(self.theta, 1e-3, math.pi - 1e-3)

        glutPostRedisplay()

    # -- GLUT callbacks ---------------------------------------------------
    def reshape(self, width: int, height: int) -> None:
        if height == 0:
            height = 1
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect = width / float(height)
        gluPerspective(self.fov_y, aspect, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)

    def idle(self) -> None:
        self.cloth.compute()
        glutPostRedisplay()

    def display(self) -> None:
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        eye, up = self._compute_camera()
        gluLookAt(
            eye[0], eye[1], eye[2],
            self.center[0], self.center[1], self.center[2],
            up[0], up[1], up[2],
        )

        glColor3f(0.8, 0.8, 0.9)
        glLineWidth(2.0)
        glBegin(GL_LINES)
        for i, j in self.mesh.edges:
            vi = self.mesh.positions[i]
            vj = self.mesh.positions[j]
            glVertex3f(*vi)
            glVertex3f(*vj)
        glEnd()

        glPointSize(5.0)
        glBegin(GL_POINTS)
        for vertex in self.mesh.positions:
            glVertex3f(*vertex)
        glEnd()

        glutSwapBuffers()
