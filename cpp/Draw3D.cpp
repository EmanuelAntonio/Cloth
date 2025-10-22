#include "Draw3D.h"

#include <GL/freeglut.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <vector>

namespace {
constexpr double kPi = 3.14159265358979323846;
Draw3D* g_active_viewer = nullptr;

void DisplayWrapper() {
    if (g_active_viewer) {
        g_active_viewer->Display();
    }
}

void ReshapeWrapper(int width, int height) {
    if (g_active_viewer) {
        g_active_viewer->Reshape(width, height);
    }
}

void IdleWrapper() {
    if (g_active_viewer) {
        g_active_viewer->Idle();
    }
}

void MouseButtonWrapper(int button, int state, int x, int y) {
    if (g_active_viewer) {
        g_active_viewer->MouseButton(button, state, x, y);
    }
}

void MouseMotionWrapper(int x, int y) {
    if (g_active_viewer) {
        g_active_viewer->MouseMotion(x, y);
    }
}

void CloseWrapper() {
    std::exit(0);
}
}  // namespace

Draw3D::Draw3D(Mesh3D* mesh, Cloth3D* cloth, int width, int height, double fov_y)
    : mesh_(mesh),
      cloth_(cloth),
      window_width_(width),
      window_height_(height),
      fov_y_(fov_y),
      theta_(kPi / 4.0),
      phi_(0.5),
      radius_(4.0),
      center_(0.0, 0.0, 0.0),
      left_button_down_(false),
      has_last_mouse_(false),
      last_mouse_x_(0),
      last_mouse_y_(0) {
    if (mesh_ && !mesh_->positions.empty()) {
        Vec3 accum(0.0, 0.0, 0.0);
        for (const auto& pos : mesh_->positions) {
            accum += pos;
        }
        center_ = accum / static_cast<double>(mesh_->positions.size());
    }
}

void Draw3D::Run() {
    int argc = 1;
    char* argv[1];
    argv[0] = const_cast<char*>("cloth_viewer");
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(window_width_, window_height_);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("Cloth Simulation");

    glClearColor(0.05f, 0.05f, 0.05f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glShadeModel(GL_SMOOTH);

    GLfloat ambient[] = {0.1f, 0.1f, 0.1f, 1.0f};
    GLfloat diffuse[] = {0.8f, 0.8f, 0.8f, 1.0f};
    GLfloat specular[] = {0.9f, 0.9f, 0.9f, 1.0f};
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular);

    g_active_viewer = this;
    glutDisplayFunc(DisplayWrapper);
    glutIdleFunc(IdleWrapper);
    glutReshapeFunc(ReshapeWrapper);
    glutMouseFunc(MouseButtonWrapper);
    glutMotionFunc(MouseMotionWrapper);
    glutCloseFunc(CloseWrapper);

    glutMainLoop();
}

std::pair<Vec3, Vec3> Draw3D::ComputeCamera() const {
    double phi_clamped = std::clamp(phi_, -kPi / 2.0 + 1e-3, kPi / 2.0 - 1e-3);
    double radius = std::max(radius_, 0.1);

    double cos_phi = std::cos(phi_clamped);
    double sin_phi = std::sin(phi_clamped);
    Vec3 eye(
        center_.x + radius * std::cos(theta_) * cos_phi,
        center_.y + radius * std::sin(theta_) * cos_phi,
        center_.z + radius * sin_phi
    );

    Vec3 forward = center_ - eye;
    double forward_len = forward.Norm();
    if (forward_len > 0.0) {
        forward /= forward_len;
    }

    Vec3 world_up(0.0, 0.0, 1.0);
    Vec3 right = Cross(forward, world_up);
    if (right.Norm() < 1e-6) {
        world_up = Vec3(0.0, 1.0, 0.0);
        right = Cross(forward, world_up);
    }
    right = right.Normalized();
    Vec3 up = Cross(right, forward).Normalized();
    return {eye, up};
}

void Draw3D::Display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    auto [eye, up] = ComputeCamera();
    GLfloat light_pos[] = {static_cast<GLfloat>(eye.x), static_cast<GLfloat>(eye.y), static_cast<GLfloat>(eye.z), 1.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
    gluLookAt(
        eye.x, eye.y, eye.z,
        center_.x, center_.y, center_.z,
        up.x, up.y, up.z
    );

    DrawColliders();
    DrawClothSurface();

    glutSwapBuffers();
}

void Draw3D::Idle() {
    if (cloth_) {
        cloth_->ComputeStepImplicit();
    }
    glutPostRedisplay();
}

void Draw3D::Reshape(int width, int height) {
    if (height == 0) {
        height = 1;
    }
    window_width_ = width;
    window_height_ = height;
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    double aspect = static_cast<double>(width) / static_cast<double>(height);
    gluPerspective(fov_y_, aspect, 0.1, 100.0);
    glMatrixMode(GL_MODELVIEW);
}

void Draw3D::MouseButton(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        left_button_down_ = (state == GLUT_DOWN);
        has_last_mouse_ = left_button_down_;
        last_mouse_x_ = x;
        last_mouse_y_ = y;
    }

    if (state == GLUT_UP && button != GLUT_LEFT_BUTTON) {
        has_last_mouse_ = false;
    }

    if (state == GLUT_DOWN && button == 3) {
        radius_ = std::max(0.2, radius_ * 0.9);
    } else if (state == GLUT_DOWN && button == 4) {
        radius_ = std::min(100.0, radius_ * 1.1);
    }

    glutPostRedisplay();
}

void Draw3D::MouseMotion(int x, int y) {
    if (!left_button_down_ || !has_last_mouse_) {
        return;
    }

    int dx = x - last_mouse_x_;
    int dy = y - last_mouse_y_;
    last_mouse_x_ = x;
    last_mouse_y_ = y;

    double sensitivity = 0.005;
    theta_ -= dx * sensitivity;
    phi_ += dy * sensitivity;
    phi_ = std::clamp(phi_, -kPi / 2.0 + 1e-3, kPi / 2.0 - 1e-3);

    glutPostRedisplay();
}

void Draw3D::DrawClothSurface() {
    if (!mesh_) {
        return;
    }
    GLfloat ambient[] = {0.1f, 0.3f, 0.05f, 1.0f};
    GLfloat diffuse[] = {0.4f, 0.9f, 0.1f, 1.0f};
    GLfloat specular[] = {0.7f, 1.0f, 0.6f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 64.0f);

    std::vector<Vec3> normals = mesh_->ComputeVertexNormals();
    glBegin(GL_TRIANGLES);
    for (const auto& face : mesh_->faces) {
        for (int index : face) {
            const Vec3& normal = normals[index];
            const Vec3& vertex = mesh_->positions[index];
            glNormal3f(static_cast<GLfloat>(normal.x), static_cast<GLfloat>(normal.y), static_cast<GLfloat>(normal.z));
            glVertex3f(static_cast<GLfloat>(vertex.x), static_cast<GLfloat>(vertex.y), static_cast<GLfloat>(vertex.z));
        }
    }
    glEnd();
}

void Draw3D::DrawColliders() {
    const auto& colliders = cloth_->Colliders();
    if (colliders.empty()) {
        return;
    }

    GLfloat ambient[] = {0.2f, 0.2f, 0.2f, 1.0f};
    GLfloat diffuse[] = {0.5f, 0.5f, 0.5f, 1.0f};
    GLfloat specular[] = {0.3f, 0.3f, 0.3f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 8.0f);

    for (const auto& collider : colliders) {
        DrawSphere(collider);
    }
}

void Draw3D::DrawSphere(const SphereCollider& collider, int slices, int stacks) {
    for (int stack = 0; stack < stacks; ++stack) {
        double v0 = static_cast<double>(stack) / static_cast<double>(stacks);
        double v1 = static_cast<double>(stack + 1) / static_cast<double>(stacks);
        double lat0 = kPi * (v0 - 0.5);
        double lat1 = kPi * (v1 - 0.5);

        double sin_lat0 = std::sin(lat0);
        double cos_lat0 = std::cos(lat0);
        double sin_lat1 = std::sin(lat1);
        double cos_lat1 = std::cos(lat1);

        glBegin(GL_TRIANGLE_STRIP);
        for (int slice = 0; slice <= slices; ++slice) {
            double u = static_cast<double>(slice) / static_cast<double>(slices);
            double lon = 2.0 * kPi * u;
            double sin_lon = std::sin(lon);
            double cos_lon = std::cos(lon);

            Vec3 normal0(
                cos_lon * cos_lat0,
                sin_lon * cos_lat0,
                sin_lat0
            );
            Vec3 normal1(
                cos_lon * cos_lat1,
                sin_lon * cos_lat1,
                sin_lat1
            );

            Vec3 vertex0 = collider.center + normal0 * collider.radius * 0.95;
            Vec3 vertex1 = collider.center + normal1 * collider.radius * 0.95;

            glNormal3f(static_cast<GLfloat>(normal1.x), static_cast<GLfloat>(normal1.y), static_cast<GLfloat>(normal1.z));
            glVertex3f(static_cast<GLfloat>(vertex1.x), static_cast<GLfloat>(vertex1.y), static_cast<GLfloat>(vertex1.z));
            glNormal3f(static_cast<GLfloat>(normal0.x), static_cast<GLfloat>(normal0.y), static_cast<GLfloat>(normal0.z));
            glVertex3f(static_cast<GLfloat>(vertex0.x), static_cast<GLfloat>(vertex0.y), static_cast<GLfloat>(vertex0.z));
        }
        glEnd();
    }
}
