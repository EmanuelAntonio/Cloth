#pragma once

#include "Cloth3D.h"
#include "Mesh3D.h"
#include "Vec3.h"

#include <utility>

class Draw3D {
public:
    Draw3D(
        Mesh3D* mesh,
        Cloth3D* cloth,
        int width = 960,
        int height = 720,
        double fov_y = 45.0
    );

    void Run();
    void Display();
    void Idle();
    void Reshape(int width, int height);
    void MouseButton(int button, int state, int x, int y);
    void MouseMotion(int x, int y);

private:
    Mesh3D* mesh_;
    Cloth3D* cloth_;
    int window_width_;
    int window_height_;
    double fov_y_;

    double theta_;
    double phi_;
    double radius_;
    Vec3 center_;

    bool left_button_down_;
    bool has_last_mouse_;
    int last_mouse_x_;
    int last_mouse_y_;

    std::pair<Vec3, Vec3> ComputeCamera() const;
    void DrawClothSurface();
    void DrawColliders();
    void DrawSphere(const SphereCollider& collider, int slices = 36, int stacks = 18);
};
