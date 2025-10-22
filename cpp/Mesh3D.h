#pragma once

#include "Vec3.h"

#include <array>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

class Mesh3D {
public:
    std::vector<Vec3> positions;
    std::vector<std::pair<int, int>> edges;
    std::vector<std::array<bool, 3>> free_dof;
    std::vector<std::array<int, 3>> faces;

    Mesh3D() = default;
    Mesh3D(
        std::vector<Vec3> positions,
        std::vector<std::pair<int, int>> edges,
        std::vector<std::array<bool, 3>> free_dof,
        std::vector<std::array<int, 3>> faces = {}
    );

    static Mesh3D BuildSquare(double size, int subdivisions);

    std::size_t VertexCount() const { return positions.size(); }

    void FixVertex(int index, const std::vector<int>& axes = {0, 1, 2}, bool permanent = true);
    void ReleaseVertex(int index, const std::vector<int>& axes = {0, 1, 2}, bool permanent = true);
    bool AxisIsFree(int index, int axis) const;

    void Reset();
    void Translate(const Vec3& delta, bool update_initial = true);
    std::vector<Vec3> ComputeVertexNormals() const;

    const std::vector<Vec3>& InitialPositions() const { return initial_positions_; }
    const std::vector<std::array<bool, 3>>& BaseFreeDof() const { return base_free_dof_; }

private:
    std::vector<Vec3> initial_positions_;
    std::vector<std::array<bool, 3>> base_free_dof_;
};
