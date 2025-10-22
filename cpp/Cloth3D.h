#pragma once

#include "Mesh3D.h"
#include "Vec3.h"

#include <cstddef>
#include <optional>
#include <unordered_set>
#include <vector>

struct SphereCollider {
    Vec3 center;
    double radius{0.0};
};

class Cloth3D {
public:
    Cloth3D(
        Mesh3D* mesh,
        double spring_k,
        double time_step,
        double mass = 1.0,
        double damping = 0.02,
        Vec3 gravity = Vec3(0.0, 0.0, -9.81),
        std::optional<double> max_stretch_ratio = std::nullopt,
        double max_stretch_relaxation = 0.2,
        std::vector<SphereCollider> colliders = {},
        double self_collision_distance = 0.05,
        int self_collision_iterations = 1,
        int worker_count = 1
    );

    void ComputeStep();

    Mesh3D& Mesh() { return *mesh_; }
    const Mesh3D& Mesh() const { return *mesh_; }
    const std::vector<SphereCollider>& Colliders() const { return colliders_; }

private:
    Mesh3D* mesh_;
    double spring_k_;
    double time_step_;
    double mass_;
    double damping_;
    Vec3 gravity_;
    std::optional<double> max_stretch_ratio_;
    double max_stretch_relaxation_;
    std::vector<SphereCollider> colliders_;
    double self_collision_distance_;
    int self_collision_iterations_;
    int worker_count_;

    std::vector<Vec3> velocities_;
    std::vector<double> rest_lengths_;
    double self_collision_distance_sq_;
    std::unordered_set<std::uint64_t> edge_lookup_;

    void AccumulateSpringForces(std::vector<Vec3>& out_forces) const;
    void EnforceMaxStretch();
    void ResolveColliders();
    void ResolveSelfCollisions();
};
