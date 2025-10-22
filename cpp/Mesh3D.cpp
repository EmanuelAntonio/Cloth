#include "Mesh3D.h"

#include <stdexcept>

Mesh3D::Mesh3D(
    std::vector<Vec3> positions_in,
    std::vector<std::pair<int, int>> edges_in,
    std::vector<std::array<bool, 3>> free_dof_in,
    std::vector<std::array<int, 3>> faces_in
) :
    positions(std::move(positions_in)),
    edges(std::move(edges_in)),
    free_dof(std::move(free_dof_in)),
    faces(std::move(faces_in)),
    initial_positions_(positions),
    base_free_dof_(free_dof)
{
    if (positions.empty()) {
        throw std::invalid_argument("positions cannot be empty");
    }
    if (edges.empty()) {
        throw std::invalid_argument("edges cannot be empty");
    }
    if (free_dof.size() != positions.size()) {
        throw std::invalid_argument("free_dof must match positions length");
    }
}

Mesh3D Mesh3D::BuildSquare(double size, int subdivisions) {
    if (subdivisions < 1) {
        throw std::invalid_argument("subdivisions must be >= 1");
    }

    int grid_n = subdivisions + 1;
    double step = size / static_cast<double>(subdivisions);

    std::vector<Vec3> positions;
    std::vector<std::array<bool, 3>> free_dof;
    std::vector<std::pair<int, int>> edges;
    std::vector<std::array<int, 3>> faces;

    positions.reserve(grid_n * grid_n);
    free_dof.reserve(grid_n * grid_n);

    double half = size / 2.0;
    for (int iy = 0; iy < grid_n; ++iy) {
        for (int ix = 0; ix < grid_n; ++ix) {
            double x = -half + ix * step;
            double y = -half + iy * step;
            positions.emplace_back(x, y, 0.0);
            free_dof.push_back({true, true, true});
        }
    }

    auto idx = [grid_n](int ix, int iy) {
        return iy * grid_n + ix;
    };

    for (int iy = 0; iy < grid_n; ++iy) {
        for (int ix = 0; ix < grid_n; ++ix) {
            if (ix + 1 < grid_n) {
                edges.emplace_back(idx(ix, iy), idx(ix + 1, iy));
            }
            if (iy + 1 < grid_n) {
                edges.emplace_back(idx(ix, iy), idx(ix, iy + 1));
            }
            if (ix + 1 < grid_n && iy + 1 < grid_n) {
                edges.emplace_back(idx(ix, iy), idx(ix + 1, iy + 1));
            }
            if (ix > 0 && iy + 1 < grid_n) {
                edges.emplace_back(idx(ix, iy), idx(ix - 1, iy + 1));
            }

            if (ix + 1 < grid_n && iy + 1 < grid_n) {
                int v0 = idx(ix, iy);
                int v1 = idx(ix + 1, iy);
                int v2 = idx(ix, iy + 1);
                int v3 = idx(ix + 1, iy + 1);
                faces.push_back({v0, v1, v2});
                faces.push_back({v2, v1, v3});
            }
        }
    }

    Mesh3D mesh(std::move(positions), std::move(edges), std::move(free_dof), std::move(faces));

    std::vector<int> corners = {
        idx(0, 0),
        idx(grid_n - 1, 0),
        idx(0, grid_n - 1),
        idx(grid_n - 1, grid_n - 1)
    };
    for (int corner : corners) {
        mesh.FixVertex(corner);
    }

    return mesh;
}

void Mesh3D::FixVertex(int index, const std::vector<int>& axes, bool permanent) {
    for (int axis : axes) {
        if (axis < 0 || axis >= 3) {
            continue;
        }
        free_dof[index][axis] = false;
        positions[index].x = initial_positions_[index].x;
        positions[index].y = initial_positions_[index].y;
        positions[index].z = initial_positions_[index].z;
        if (permanent) {
            base_free_dof_[index][axis] = false;
        }
    }
}

void Mesh3D::ReleaseVertex(int index, const std::vector<int>& axes, bool permanent) {
    for (int axis : axes) {
        if (axis < 0 || axis >= 3) {
            continue;
        }
        free_dof[index][axis] = true;
        if (permanent) {
            base_free_dof_[index][axis] = true;
        }
    }
}

bool Mesh3D::AxisIsFree(int index, int axis) const {
    if (axis < 0 || axis >= 3) {
        return false;
    }
    return free_dof[index][axis];
}

void Mesh3D::Reset() {
    positions = initial_positions_;
    free_dof = base_free_dof_;
}

void Mesh3D::Translate(const Vec3& delta, bool update_initial) {
    for (auto& pos : positions) {
        pos += delta;
    }
    if (update_initial) {
        for (auto& init : initial_positions_) {
            init += delta;
        }
    }
}

std::vector<Vec3> Mesh3D::ComputeVertexNormals() const {
    std::vector<Vec3> normals(positions.size(), Vec3(0.0, 0.0, 0.0));
    for (const auto& face : faces) {
        int i0 = face[0];
        int i1 = face[1];
        int i2 = face[2];
        const Vec3& p0 = positions[i0];
        const Vec3& p1 = positions[i1];
        const Vec3& p2 = positions[i2];
        Vec3 edge1 = p1 - p0;
        Vec3 edge2 = p2 - p0;
        Vec3 face_normal = Cross(edge1, edge2);
        double norm = face_normal.Norm();
        if (norm == 0.0) {
            continue;
        }
        face_normal /= norm;
        normals[i0] += face_normal;
        normals[i1] += face_normal;
        normals[i2] += face_normal;
    }

    for (auto& normal : normals) {
        double len = normal.Norm();
        if (len == 0.0) {
            normal = Vec3(0.0, 0.0, 1.0);
        } else {
            normal /= len;
        }
    }
    return normals;
}
