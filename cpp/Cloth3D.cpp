#include "Cloth3D.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace {
std::uint64_t EdgeKey(int a, int b) {
    if (a > b) {
        std::swap(a, b);
    }
    return (static_cast<std::uint64_t>(a) << 32) | static_cast<std::uint32_t>(b);
}

struct GridKey {
    int x;
    int y;
    int z;

    bool operator==(const GridKey& other) const noexcept {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct GridKeyHash {
    std::size_t operator()(const GridKey& key) const noexcept {
        std::size_t h = static_cast<std::size_t>(key.x);
        h = h * 73856093u ^ static_cast<std::size_t>(key.y) * 19349663u;
        h = h * 83492791u ^ static_cast<std::size_t>(key.z);
        return h;
    }
};
}  // namespace

Cloth3D::Cloth3D(
    Mesh3D* mesh,
    double spring_k,
    double time_step,
    double mass,
    double damping,
    Vec3 gravity,
    std::optional<double> max_stretch_ratio,
    double max_stretch_relaxation,
    std::vector<SphereCollider> colliders,
    double self_collision_distance,
    int self_collision_iterations,
    int worker_count
) :
    mesh_(mesh),
    spring_k_(spring_k),
    time_step_(time_step),
    mass_(mass),
    damping_(damping),
    gravity_(gravity),
    max_stretch_ratio_(max_stretch_ratio),
    max_stretch_relaxation_(max_stretch_relaxation),
    colliders_(std::move(colliders)),
    self_collision_distance_(self_collision_distance),
    self_collision_iterations_(self_collision_iterations),
    worker_count_(std::max(worker_count, 1)),
    velocities_(mesh->positions.size(), Vec3(0.0, 0.0, 0.0)),
    rest_lengths_(mesh->edges.size(), 0.0),
    self_collision_distance_sq_(self_collision_distance * self_collision_distance)
{
    if (!mesh_) {
        throw std::invalid_argument("mesh pointer cannot be null");
    }
    if (spring_k_ <= 0.0) {
        throw std::invalid_argument("spring_k must be positive");
    }
    if (time_step_ <= 0.0) {
        throw std::invalid_argument("time_step must be positive");
    }
    if (max_stretch_ratio_ && *max_stretch_ratio_ < 1.0) {
        throw std::invalid_argument("max_stretch_ratio must be >= 1.0 or nullopt");
    }
    if (!(max_stretch_relaxation_ > 0.0 && max_stretch_relaxation_ <= 1.0)) {
        throw std::invalid_argument("max_stretch_relaxation must be in (0, 1]");
    }
    if (self_collision_distance_ < 0.0) {
        throw std::invalid_argument("self_collision_distance must be >= 0");
    }
    if (self_collision_iterations_ < 0) {
        throw std::invalid_argument("self_collision_iterations must be >= 0");
    }

    const auto& positions = mesh_->positions;
    const auto& edges = mesh_->edges;
    for (std::size_t edge_index = 0; edge_index < edges.size(); ++edge_index) {
        const auto& edge = edges[edge_index];
        Vec3 delta = positions[edge.second] - positions[edge.first];
        rest_lengths_[edge_index] = delta.Norm();
        edge_lookup_.insert(EdgeKey(edge.first, edge.second));
    }
}

void Cloth3D::ComputeStepImplicit() {
    auto& positions = mesh_->positions;
    const std::size_t count = positions.size();
    if (count == 0) {
        return;
    }

    double dt = time_step_;
    double cfl = ComputeCFLTimeStep();
    std::cout << "CFL: " << cfl << std::endl;
    if (std::isfinite(cfl)) {
        dt = std::min(dt, cfl);
    }

    std::vector<Vec3> forces(count, Vec3(0.0, 0.0, 0.0));
    AccumulateSpringForces(forces);

    for (std::size_t idx = 0; idx < count; ++idx) {
        if (mesh_->AxisIsFree(static_cast<int>(idx), 2)) {
            forces[idx] += gravity_ * mass_;
        }
    }

    std::vector<Vec3> rhs(count, Vec3(0.0, 0.0, 0.0));
    for (std::size_t idx = 0; idx < count; ++idx) {
        rhs[idx] = velocities_[idx] * mass_ + forces[idx] * dt;
    }
    ApplyConstraintMask(rhs);

    std::vector<Vec3> solution = velocities_;
    ApplyConstraintMask(solution);

    if (!ConjugateGradient(solution, rhs, dt)) {
        ComputeStepExplicit();
        return;
    }

    velocities_ = solution;
    ApplyConstraintMask(velocities_);

    for (auto& velocity : velocities_) {
        velocity *= (1.0 - damping_);
    }
    for (std::size_t idx = 0; idx < count; ++idx) {
        positions[idx] += velocities_[idx] * dt;
    }

    EnforceMaxStretch();
    ResolveSelfCollisions();
    ResolveColliders();

    const auto& initial = mesh_->InitialPositions();
    auto& free_dof = mesh_->free_dof;
    for (std::size_t idx = 0; idx < count; ++idx) {
        for (int axis = 0; axis < 3; ++axis) {
            if (!free_dof[idx][axis]) {
                if (axis == 0) {
                    positions[idx].x = initial[idx].x;
                    velocities_[idx].x = 0.0;
                } else if (axis == 1) {
                    positions[idx].y = initial[idx].y;
                    velocities_[idx].y = 0.0;
                } else {
                    positions[idx].z = initial[idx].z;
                    velocities_[idx].z = 0.0;
                }
            }
        }
    }
}

void Cloth3D::ComputeStepExplicit() {
    auto& positions = mesh_->positions;
    double dt = time_step_;
    double cfl = ComputeCFLTimeStep();
    std::cout << "CFL: " << cfl << std::endl;
    if (std::isfinite(cfl)) {
        dt = std::min(dt, cfl);
    }
    std::vector<Vec3> forces(positions.size(), Vec3(0.0, 0.0, 0.0));

    AccumulateSpringForces(forces);

    for (std::size_t idx = 0; idx < positions.size(); ++idx) {
        if (mesh_->AxisIsFree(static_cast<int>(idx), 2)) {
            forces[idx] += gravity_ * mass_;
        }
    }

    for (std::size_t idx = 0; idx < positions.size(); ++idx) {
        Vec3 acceleration = forces[idx] / mass_;
        velocities_[idx] += acceleration * dt;
        velocities_[idx] *= (1.0 - damping_);
        positions[idx] += velocities_[idx] * dt;
    }

    EnforceMaxStretch();
    ResolveSelfCollisions();
    ResolveColliders();

    const auto& initial = mesh_->InitialPositions();
    auto& free_dof = mesh_->free_dof;
    for (std::size_t idx = 0; idx < positions.size(); ++idx) {
        for (int axis = 0; axis < 3; ++axis) {
            if (!free_dof[idx][axis]) {
                if (axis == 0) {
                    positions[idx].x = initial[idx].x;
                    velocities_[idx].x = 0.0;
                } else if (axis == 1) {
                    positions[idx].y = initial[idx].y;
                    velocities_[idx].y = 0.0;
                } else {
                    positions[idx].z = initial[idx].z;
                    velocities_[idx].z = 0.0;
                }
            }
        }
    }
}

void Cloth3D::AccumulateSpringForces(std::vector<Vec3>& out_forces) const {
    const auto& positions = mesh_->positions;
    const auto& edges = mesh_->edges;

    auto accumulate_single = [&](std::size_t edge_index, std::vector<Vec3>& target) {
        const auto& edge = edges[edge_index];
        int i = edge.first;
        int j = edge.second;
        Vec3 delta = positions[j] - positions[i];
        double length = delta.Norm();
        if (length == 0.0) {
            return;
        }
        Vec3 direction = delta / length;
        double rest = rest_lengths_[edge_index];
        double force_magnitude = spring_k_ * (length - rest);
        Vec3 force = direction * force_magnitude;
        target[i] += force;
        target[j] -= force;
    };

    if (worker_count_ <= 1) {
        for (std::size_t edge_index = 0; edge_index < edges.size(); ++edge_index) {
            accumulate_single(edge_index, out_forces);
        }
        return;
    }

#ifdef _OPENMP
    int available_threads = omp_get_max_threads();
    int desired_threads = std::min(worker_count_, available_threads);
    if (desired_threads <= 1) {
        for (std::size_t edge_index = 0; edge_index < edges.size(); ++edge_index) {
            accumulate_single(edge_index, out_forces);
        }
        return;
    }

    std::vector<std::vector<Vec3>> partial(desired_threads, std::vector<Vec3>(out_forces.size(), Vec3(0.0, 0.0, 0.0)));

#pragma omp parallel for schedule(static) num_threads(desired_threads)
    for (int edge_index = 0; edge_index < static_cast<int>(edges.size()); ++edge_index) {
        int tid = omp_get_thread_num();
        auto& local = partial[tid];
        accumulate_single(static_cast<std::size_t>(edge_index), local);
    }

    for (const auto& local : partial) {
        for (std::size_t idx = 0; idx < local.size(); ++idx) {
            out_forces[idx] += local[idx];
        }
    }
#else
    for (std::size_t edge_index = 0; edge_index < edges.size(); ++edge_index) {
        accumulate_single(edge_index, out_forces);
    }
#endif
}

void Cloth3D::ApplyStiffnessMatrix(const std::vector<Vec3>& in, std::vector<Vec3>& out) const {
    const auto& positions = mesh_->positions;
    const auto& edges = mesh_->edges;
    if (out.size() != positions.size()) {
        out.assign(positions.size(), Vec3(0.0, 0.0, 0.0));
    }
    for (auto& value : out) {
        value = Vec3(0.0, 0.0, 0.0);
    }

    for (std::size_t edge_index = 0; edge_index < edges.size(); ++edge_index) {
        const auto& edge = edges[edge_index];
        int i = edge.first;
        int j = edge.second;
        Vec3 delta = positions[j] - positions[i];
        double length = delta.Norm();
        if (length == 0.0) {
            continue;
        }
        Vec3 direction = delta / length;
        Vec3 diff = in[i] - in[j];
        double projection = Dot(direction, diff);
        Vec3 contribution = direction * (spring_k_ * projection);
        out[i] += contribution;
        out[j] -= contribution;
    }

    const auto& free_dof = mesh_->free_dof;
    for (std::size_t idx = 0; idx < out.size(); ++idx) {
        out[idx] = ClampAxis(out[idx], free_dof[idx]);
    }
}

void Cloth3D::ApplyImplicitMatrix(const std::vector<Vec3>& in, std::vector<Vec3>& out, std::vector<Vec3>& scratch, double dt) const {
    const auto& free_dof = mesh_->free_dof;
    const std::size_t count = mesh_->positions.size();
    if (out.size() != count) {
        out.assign(count, Vec3(0.0, 0.0, 0.0));
    }
    if (scratch.size() != count) {
        scratch.assign(count, Vec3(0.0, 0.0, 0.0));
    }

    for (std::size_t idx = 0; idx < count; ++idx) {
        Vec3 value = in[idx] * mass_;
        out[idx] = ClampAxis(value, free_dof[idx]);
    }

    ApplyStiffnessMatrix(in, scratch);
    double factor = dt * dt;
    for (std::size_t idx = 0; idx < count; ++idx) {
        out[idx] -= scratch[idx] * factor;
        out[idx] = ClampAxis(out[idx], free_dof[idx]);
    }
}

void Cloth3D::ApplyConstraintMask(std::vector<Vec3>& vec) const {
    const auto& free_dof = mesh_->free_dof;
    const std::size_t count = vec.size();
    for (std::size_t idx = 0; idx < count; ++idx) {
        vec[idx] = ClampAxis(vec[idx], free_dof[idx]);
    }
}

double Cloth3D::DotProduct(const std::vector<Vec3>& a, const std::vector<Vec3>& b) const {
    double result = 0.0;
    const std::size_t count = std::min(a.size(), b.size());
    for (std::size_t idx = 0; idx < count; ++idx) {
        result += Dot(a[idx], b[idx]);
    }
    return result;
}

bool Cloth3D::ConjugateGradient(std::vector<Vec3>& x, const std::vector<Vec3>& b, double dt) const {
    const std::size_t count = b.size();
    if (count == 0) {
        return true;
    }

    std::vector<Vec3> r(count, Vec3(0.0, 0.0, 0.0));
    std::vector<Vec3> p(count, Vec3(0.0, 0.0, 0.0));
    std::vector<Vec3> Ap(count, Vec3(0.0, 0.0, 0.0));
    std::vector<Vec3> scratch(count, Vec3(0.0, 0.0, 0.0));

    ApplyImplicitMatrix(x, Ap, scratch, dt);
    for (std::size_t idx = 0; idx < count; ++idx) {
        r[idx] = b[idx] - Ap[idx];
    }
    ApplyConstraintMask(r);
    p = r;

    double rhs_norm = std::sqrt(DotProduct(b, b));
    double tolerance = rhs_norm > 0.0 ? rhs_norm * 1e-6 : 1e-9;
    double rs_old = DotProduct(r, r);
    if (std::sqrt(rs_old) <= tolerance) {
        return true;
    }

    int max_iterations = std::max<int>(static_cast<int>(count), 50);
    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        ApplyImplicitMatrix(p, Ap, scratch, dt);
        double denom = DotProduct(p, Ap);
        if (std::abs(denom) < 1e-12) {
            return false;
        }
        double alpha = rs_old / denom;
        for (std::size_t idx = 0; idx < count; ++idx) {
            x[idx] += p[idx] * alpha;
            r[idx] -= Ap[idx] * alpha;
        }
        ApplyConstraintMask(x);
        ApplyConstraintMask(r);

        double rs_new = DotProduct(r, r);
        if (std::sqrt(rs_new) <= tolerance) {
            return true;
        }
        double beta = rs_new / rs_old;
        for (std::size_t idx = 0; idx < count; ++idx) {
            p[idx] = r[idx] + p[idx] * beta;
        }
        ApplyConstraintMask(p);
        rs_old = rs_new;
    }
    return false;
}

void Cloth3D::EnforceMaxStretch() {
    if (!max_stretch_ratio_) {
        return;
    }

    auto& positions = mesh_->positions;
    auto& free_dof = mesh_->free_dof;
    double ratio = *max_stretch_ratio_;
    double relaxation = max_stretch_relaxation_;

    for (std::size_t edge_index = 0; edge_index < mesh_->edges.size(); ++edge_index) {
        const auto& edge = mesh_->edges[edge_index];
        int i = edge.first;
        int j = edge.second;
        double rest = rest_lengths_[edge_index];
        double max_length = rest * ratio;
        Vec3 delta = positions[j] - positions[i];
        double length = delta.Norm();
        if (length <= max_length || length == 0.0) {
            continue;
        }

        Vec3 direction = delta / length;
        double correction_mag = (length - max_length) * relaxation;

        const auto& free_i = free_dof[i];
        const auto& free_j = free_dof[j];
        bool has_free_i = free_i[0] || free_i[1] || free_i[2];
        bool has_free_j = free_j[0] || free_j[1] || free_j[2];
        if (!has_free_i && !has_free_j) {
            continue;
        }

        double share_i = 0.5;
        double share_j = 0.5;
        if (has_free_i && !has_free_j) {
            share_i = 1.0;
            share_j = 0.0;
        } else if (!has_free_i && has_free_j) {
            share_i = 0.0;
            share_j = 1.0;
        }

        Vec3 move_i = direction * (share_i * correction_mag);
        Vec3 move_j = direction * (-share_j * correction_mag);
        move_i = ClampAxis(move_i, free_i);
        move_j = ClampAxis(move_j, free_j);

        positions[i] += move_i;
        positions[j] += move_j;

        if (std::abs(move_i.x) > 1e-9) velocities_[i].x = 0.0;
        if (std::abs(move_i.y) > 1e-9) velocities_[i].y = 0.0;
        if (std::abs(move_i.z) > 1e-9) velocities_[i].z = 0.0;
        if (std::abs(move_j.x) > 1e-9) velocities_[j].x = 0.0;
        if (std::abs(move_j.y) > 1e-9) velocities_[j].y = 0.0;
        if (std::abs(move_j.z) > 1e-9) velocities_[j].z = 0.0;
    }
}

void Cloth3D::ResolveColliders() {
    if (colliders_.empty()) {
        return;
    }

    auto& positions = mesh_->positions;
    auto& free_dof = mesh_->free_dof;

    for (const auto& collider : colliders_) {
        for (std::size_t idx = 0; idx < positions.size(); ++idx) {
            const auto& mask = free_dof[idx];
            if (!mask[0] && !mask[1] && !mask[2]) {
                continue;
            }
            Vec3 offset = positions[idx] - collider.center;
            double distance = offset.Norm();
            if (distance >= collider.radius) {
                continue;
            }
            Vec3 normal;
            if (distance < 1e-8) {
                normal = Vec3(0.0, 0.0, 1.0);
            } else {
                normal = offset / distance;
            }
            Vec3 target = collider.center + normal * collider.radius;
            Vec3 correction = target - positions[idx];
            correction = ClampAxis(correction, mask);
            if (std::abs(correction.x) < 1e-12 && std::abs(correction.y) < 1e-12 && std::abs(correction.z) < 1e-12) {
                continue;
            }
            positions[idx] += correction;
            double normal_velocity = Dot(velocities_[idx], normal);
            if (normal_velocity < 0.0) {
                Vec3 adjustment = normal * normal_velocity;
                velocities_[idx] -= adjustment;
            }
        }
    }
}

void Cloth3D::ResolveSelfCollisions() {
    if (self_collision_distance_ <= 0.0 || self_collision_iterations_ <= 0) {
        return;
    }

    auto& positions = mesh_->positions;
    auto& free_dof = mesh_->free_dof;
    double radius = self_collision_distance_;
    double radius_sq = self_collision_distance_sq_;
    double cell_size = radius;
    if (cell_size <= 0.0) {
        return;
    }
    double inv_cell = 1.0 / cell_size;

    for (int iteration = 0; iteration < self_collision_iterations_; ++iteration) {
        std::unordered_map<GridKey, std::vector<int>, GridKeyHash> grid;
        grid.reserve(positions.size());

        for (std::size_t idx = 0; idx < positions.size(); ++idx) {
            const Vec3& pos = positions[idx];
            GridKey cell{
                static_cast<int>(std::floor(pos.x * inv_cell)),
                static_cast<int>(std::floor(pos.y * inv_cell)),
                static_cast<int>(std::floor(pos.z * inv_cell))
            };
            grid[cell].push_back(static_cast<int>(idx));
        }

        for (std::size_t idx = 0; idx < positions.size(); ++idx) {
            const Vec3& pos = positions[idx];
            GridKey cell{
                static_cast<int>(std::floor(pos.x * inv_cell)),
                static_cast<int>(std::floor(pos.y * inv_cell)),
                static_cast<int>(std::floor(pos.z * inv_cell))
            };
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        GridKey neighbour{cell.x + dx, cell.y + dy, cell.z + dz};
                        auto it = grid.find(neighbour);
                        if (it == grid.end()) {
                            continue;
                        }
                        const auto& candidates = it->second;
                        for (int other : candidates) {
                            if (other <= static_cast<int>(idx)) {
                                continue;
                            }
                            if (edge_lookup_.count(EdgeKey(static_cast<int>(idx), other)) > 0) {
                                continue;
                            }
                            Vec3 delta = positions[other] - positions[idx];
                            double dist_sq = delta.NormSquared();
                            if (dist_sq >= radius_sq) {
                                continue;
                            }
                            Vec3 normal;
                            double dist = 0.0;
                            if (dist_sq == 0.0) {
                                normal = Vec3(0.0, 0.0, 1.0);
                            } else {
                                dist = std::sqrt(dist_sq);
                                normal = delta / dist;
                            }
                            double penetration = radius - dist;
                            if (penetration <= 0.0) {
                                continue;
                            }
                            const auto& free_i = free_dof[idx];
                            const auto& free_j = free_dof[other];
                            bool has_i = free_i[0] || free_i[1] || free_i[2];
                            bool has_j = free_j[0] || free_j[1] || free_j[2];
                            if (!has_i && !has_j) {
                                continue;
                            }
                            double share_i = 0.5;
                            double share_j = 0.5;
                            if (has_i && !has_j) {
                                share_i = 1.0;
                                share_j = 0.0;
                            } else if (!has_i && has_j) {
                                share_i = 0.0;
                                share_j = 1.0;
                            }
                            Vec3 correction_i = normal * (-share_i * penetration);
                            Vec3 correction_j = normal * (share_j * penetration);
                            correction_i = ClampAxis(correction_i, free_i);
                            correction_j = ClampAxis(correction_j, free_j);
                            if (std::abs(correction_i.x) > 0.0 || std::abs(correction_i.y) > 0.0 || std::abs(correction_i.z) > 0.0) {
                                positions[idx] += correction_i;
                                double normal_component = Dot(velocities_[idx], normal);
                                Vec3 tangential = velocities_[idx] - normal_component * normal;
                                velocities_[idx] = ClampAxis(tangential, free_i);
                            }
                            if (std::abs(correction_j.x) > 0.0 || std::abs(correction_j.y) > 0.0 || std::abs(correction_j.z) > 0.0) {
                                positions[other] += correction_j;
                                double normal_component = Dot(velocities_[other], normal);
                                Vec3 tangential = velocities_[other] - normal_component * normal;
                                velocities_[other] = ClampAxis(tangential, free_j);
                            }
                        }
                    }
                }
            }
        }
    }
}

double Cloth3D::ComputeCFLTimeStep(){
    if (spring_k_ <= 0.0) {
        return std::numeric_limits<double>::infinity();
    }

    double max_stiffness = 0.0;
    std::vector<double> per_vertex(mesh_->positions.size(), 0.0);
    for (const auto& edge : mesh_->edges) {
        per_vertex[edge.first] += spring_k_;
        per_vertex[edge.second] += spring_k_;
    }
    for (double value : per_vertex) {
        if (value > max_stiffness) {
            max_stiffness = value;
        }
    }

    if (max_stiffness <= 0.0 || mass_ <= 0.0) {
        return std::numeric_limits<double>::infinity();
    }

    double dt = std::sqrt(mass_ / max_stiffness);
    return dt;
}
