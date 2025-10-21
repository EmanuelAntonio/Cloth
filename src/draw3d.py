"""OpenGL visualization utilities for the cloth simulation."""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np
from OpenGL.GL import (
    glBegin,
    glClear,
    glClearColor,
    glEnable,
    glEnd,
    glLightfv,
    glLoadIdentity,
    glMaterialf,
    glMaterialfv,
    glMatrixMode,
    glNormal3fv,
    glShadeModel,
    glViewport,
    glVertex3f,
    GL_AMBIENT,
    GL_COLOR_BUFFER_BIT,
    GL_DEPTH_BUFFER_BIT,
    GL_DEPTH_TEST,
    GL_DIFFUSE,
    GL_FRONT_AND_BACK,
    GL_LIGHT0,
    GL_LIGHTING,
    GL_MODELVIEW,
    GL_PROJECTION,
    GL_SHININESS,
    GL_SPECULAR,
    GL_TRIANGLES,
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

from cloth3d import Cloth3D
from mesh3d import Mesh3D


@dataclass
class Draw3D:
    mesh: Mesh3D
    cloth: Cloth3D
    window_size: Tuple[int, int] = (960, 720)
    fov_y: float = 45.0

    theta: float = math.pi / 4.0
    phi: float = 0.5
    radius: float = 4.0
    center: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))

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

        glClearColor(0.05, 0.05, 0.05, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glShadeModel(GL_SMOOTH)

        glLightfv(GL_LIGHT0, GL_AMBIENT, (0.1, 0.1, 0.1, 1.0))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (0.8, 0.8, 0.8, 1.0))
        glLightfv(GL_LIGHT0, GL_SPECULAR, (0.9, 0.9, 0.9, 1.0))

        glutDisplayFunc(self.display)
        glutIdleFunc(self.idle)
        glutReshapeFunc(self.reshape)
        glutMouseFunc(self.mouse_button)
        glutMotionFunc(self.mouse_motion)
        glutWMCloseFunc(lambda: sys.exit(0))

        glutMainLoop()

    # -- Camera handling --------------------------------------------------
    def _compute_camera(self) -> Tuple[np.ndarray, np.ndarray]:
        theta = self.theta
        phi = np.clip(self.phi, -math.pi / 2 + 1e-3, math.pi / 2 - 1e-3)
        r = max(self.radius, 0.1)

        cos_phi = math.cos(phi)
        sin_phi = math.sin(phi)
        eye = np.array(
            [
                self.center[0] + r * math.cos(theta) * cos_phi,
                self.center[1] + r * math.sin(theta) * cos_phi,
                self.center[2] + r * sin_phi,
            ],
            dtype=np.float64,
        )

        forward = self.center - eye
        forward /= np.linalg.norm(forward)

        world_up = np.array([0.0, 0.0, 1.0])
        right = np.cross(forward, world_up)
        if np.linalg.norm(right) < 1e-6:
            world_up = np.array([0.0, 1.0, 0.0])
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
        self.theta += dx * sensitivity
        self.phi -= dy * sensitivity
        self.phi = np.clip(self.phi, -math.pi / 2 + 1e-3, math.pi / 2 - 1e-3)

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
        glLightfv(GL_LIGHT0, GL_POSITION, (eye[0], eye[1], eye[2], 1.0))
        gluLookAt(
            eye[0], eye[1], eye[2],
            self.center[0], self.center[1], self.center[2],
            up[0], up[1], up[2],
        )

        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, (0.1, 0.3, 0.05, 1.0))
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, (0.4, 0.9, 0.1, 1.0))
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, (0.7, 1.0, 0.6, 1.0))
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 64.0)

        normals = self.mesh.compute_vertex_normals()
        glBegin(GL_TRIANGLES)
        for face in self.mesh.faces:
            for vertex_index in face:
                glNormal3fv(normals[vertex_index])
                glVertex3f(*self.mesh.positions[vertex_index])
        glEnd()

        glutSwapBuffers()
