#include "Cloth3D.h"
#include "Draw3D.h"
#include "Mesh3D.h"

#include <cstdlib>
#include <cstring>
#include <exception>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

struct Options {
    double size = 2.0;
    int subdivisions = 10;
    double spring = 800.0;
    double timestep = 1.0 / 120.0;
    std::optional<double> max_stretch;
    double max_stretch_relaxation = 0.2;
    std::string scenario = "cloth";
    double sphere_radius = 0.6;
    double drop_height = 0.8;
    double self_collision_distance = 0.05;
    int self_collision_iterations = 1;
    int workers = 1;
};

namespace {

double ParseDouble(const char* value, const char* flag) {
    char* end = nullptr;
    double result = std::strtod(value, &end);
    if (!value || end == value) {
        throw std::runtime_error(std::string("Invalid value for ") + flag);
    }
    return result;
}

int ParseInt(const char* value, const char* flag) {
    char* end = nullptr;
    long parsed = std::strtol(value, &end, 10);
    if (!value || end == value) {
        throw std::runtime_error(std::string("Invalid value for ") + flag);
    }
    return static_cast<int>(parsed);
}

Options ParseArgs(int argc, char** argv) {
    Options opts;
    for (int i = 1; i < argc; ++i) {
        const char* arg = argv[i];
        auto require_value = [&](const char* flag) -> const char* {
            if (i + 1 >= argc) {
                throw std::runtime_error(std::string("Missing value for ") + flag);
            }
            return argv[++i];
        };

        if (std::strcmp(arg, "--size") == 0) {
            opts.size = ParseDouble(require_value(arg), arg);
        } else if (std::strcmp(arg, "--subdivisions") == 0) {
            opts.subdivisions = ParseInt(require_value(arg), arg);
        } else if (std::strcmp(arg, "--spring") == 0) {
            opts.spring = ParseDouble(require_value(arg), arg);
        } else if (std::strcmp(arg, "--timestep") == 0) {
            opts.timestep = ParseDouble(require_value(arg), arg);
        } else if (std::strcmp(arg, "--max-stretch") == 0) {
            double value = ParseDouble(require_value(arg), arg);
            opts.max_stretch = value;
        } else if (std::strcmp(arg, "--max-stretch-relaxation") == 0) {
            opts.max_stretch_relaxation = ParseDouble(require_value(arg), arg);
        } else if (std::strcmp(arg, "--scenario") == 0) {
            opts.scenario = require_value(arg);
            if (opts.scenario != "cloth" && opts.scenario != "sphere") {
                throw std::runtime_error("--scenario must be 'cloth' or 'sphere'");
            }
        } else if (std::strcmp(arg, "--sphere-radius") == 0) {
            opts.sphere_radius = ParseDouble(require_value(arg), arg);
        } else if (std::strcmp(arg, "--drop-height") == 0) {
            opts.drop_height = ParseDouble(require_value(arg), arg);
        } else if (std::strcmp(arg, "--self-collision-distance") == 0) {
            opts.self_collision_distance = ParseDouble(require_value(arg), arg);
        } else if (std::strcmp(arg, "--self-collision-iterations") == 0) {
            opts.self_collision_iterations = ParseInt(require_value(arg), arg);
        } else if (std::strcmp(arg, "--workers") == 0) {
            opts.workers = ParseInt(require_value(arg), arg);
        } else if (std::strcmp(arg, "--help") == 0 || std::strcmp(arg, "-h") == 0) {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "  --size <float>\n"
                      << "  --subdivisions <int>\n"
                      << "  --spring <float>\n"
                      << "  --timestep <float>\n"
                      << "  --max-stretch <float>\n"
                      << "  --max-stretch-relaxation <float>\n"
                      << "  --scenario <cloth|sphere>\n"
                      << "  --sphere-radius <float>\n"
                      << "  --drop-height <float>\n"
                      << "  --self-collision-distance <float>\n"
                      << "  --self-collision-iterations <int>\n"
                      << "  --workers <int>\n";
            std::exit(0);
        } else {
            throw std::runtime_error(std::string("Unknown argument: ") + arg);
        }
    }
    return opts;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        Options opts = ParseArgs(argc, argv);
        Mesh3D mesh = Mesh3D::BuildSquare(opts.size, opts.subdivisions);
        std::vector<SphereCollider> colliders;

        if (opts.scenario == "sphere") {
            int grid_n = opts.subdivisions + 1;
            std::vector<int> corners = {
                0,
                grid_n - 1,
                grid_n * (grid_n - 1),
                grid_n * grid_n - 1
            };
            for (int corner : corners) {
                mesh.ReleaseVertex(corner, {0, 1, 2}, true);
            }
            double drop_offset = opts.sphere_radius + opts.drop_height;
            mesh.Translate(Vec3(0.0, 0.0, drop_offset));
            colliders.push_back({Vec3(0.0, 0.0, 0.0), opts.sphere_radius});
        }

        Cloth3D cloth(
            &mesh,
            opts.spring,
            opts.timestep,
            1.0,
            0.02,
            Vec3(0.0, 0.0, -9.81),
            opts.max_stretch,
            opts.max_stretch_relaxation,
            colliders,
            opts.self_collision_distance,
            opts.self_collision_iterations,
            opts.workers
        );

        Draw3D viewer(&mesh, &cloth);
        viewer.Run();
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }
    return 0;
}
