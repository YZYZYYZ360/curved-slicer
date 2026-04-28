#include "surface/iso_surface.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <set>
#include <stdexcept>
#include <utility>

namespace {

using cslc::AABB;
using cslc::ScalarField;
using cslc::Vec3;

std::size_t scalarIndex(const ScalarField& field, int x, int y, int z)
{
    return static_cast<std::size_t>(x) +
        static_cast<std::size_t>(field.nx) *
            (static_cast<std::size_t>(y) + static_cast<std::size_t>(field.ny) * static_cast<std::size_t>(z));
}

Vec3 samplePoint(const ScalarField& field, int x, int y, int z)
{
    return {
        field.bbox.min.x + (static_cast<double>(x) + 0.5) * field.spacing,
        field.bbox.min.y + (static_cast<double>(y) + 0.5) * field.spacing,
        field.bbox.min.z + (static_cast<double>(z) + 0.5) * field.spacing,
    };
}

ScalarField makeField(int n, double spacing)
{
    ScalarField field;
    field.spacing = spacing;
    field.nx = n;
    field.ny = n;
    field.nz = n;
    const double extent = static_cast<double>(n) * spacing;
    field.bbox = AABB{{-0.5 * extent, -0.5 * extent, -0.5 * extent},
                      {0.5 * extent, 0.5 * extent, 0.5 * extent}};
    field.values.resize(static_cast<std::size_t>(n) * static_cast<std::size_t>(n) * static_cast<std::size_t>(n));
    return field;
}

ScalarField makeCubeField()
{
    ScalarField field = makeField(9, 0.25);
    for (int z = 0; z < field.nz; ++z) {
        for (int y = 0; y < field.ny; ++y) {
            for (int x = 0; x < field.nx; ++x) {
                const Vec3 p = samplePoint(field, x, y, z);
                field.values[scalarIndex(field, x, y, z)] =
                    std::max({std::abs(p.x), std::abs(p.y), std::abs(p.z)}) - 0.5;
            }
        }
    }
    return field;
}

ScalarField makeSphereField()
{
    ScalarField field = makeField(25, 0.1);
    constexpr double radius = 0.8;
    for (int z = 0; z < field.nz; ++z) {
        for (int y = 0; y < field.ny; ++y) {
            for (int x = 0; x < field.nx; ++x) {
                const Vec3 p = samplePoint(field, x, y, z);
                field.values[scalarIndex(field, x, y, z)] =
                    std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z) - radius;
            }
        }
    }
    return field;
}

int eulerCharacteristic(const cslc::IsoMesh& mesh)
{
    std::set<std::pair<std::size_t, std::size_t>> edges;
    for (const cslc::Tri& tri : mesh.triangles) {
        const std::array<std::size_t, 3> vertices{tri.v0, tri.v1, tri.v2};
        for (int i = 0; i < 3; ++i) {
            std::size_t a = vertices[static_cast<std::size_t>(i)];
            std::size_t b = vertices[static_cast<std::size_t>((i + 1) % 3)];
            if (a > b) {
                std::swap(a, b);
            }
            edges.emplace(a, b);
        }
    }

    return static_cast<int>(mesh.vertices.size()) -
        static_cast<int>(edges.size()) +
        static_cast<int>(mesh.triangles.size());
}

void require(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

}  // namespace

int main()
{
    const ScalarField cube = makeCubeField();
    const cslc::IsoMesh cube_mesh = cslc::extractIsoSurface(cube, 0.0, 0);
    require(!cube_mesh.vertices.empty(), "cube MC mesh must have vertices");
    require(!cube_mesh.triangles.empty(), "cube MC mesh must have triangles");
    require(cslc::countConnectedComponents(cube_mesh) == 1, "cube MC mesh must be one connected component");
    require(cube_mesh.triangles.size() >= 12, "cube MC mesh should have at least the canonical 12 triangles");

    const ScalarField sphere = makeSphereField();
    const cslc::IsoMesh sphere_mesh = cslc::extractIsoSurface(sphere, 0.0, 0);
    require(!sphere_mesh.vertices.empty(), "sphere MC mesh must have vertices");
    require(!sphere_mesh.triangles.empty(), "sphere MC mesh must have triangles");
    require(cslc::countConnectedComponents(sphere_mesh) == 1, "sphere MC mesh must be one connected component");
    require(eulerCharacteristic(sphere_mesh) == 2, "sphere MC mesh must satisfy V - E + F = 2");

    std::cout << "cube_vertices=" << cube_mesh.vertices.size()
              << " cube_triangles=" << cube_mesh.triangles.size()
              << " sphere_vertices=" << sphere_mesh.vertices.size()
              << " sphere_triangles=" << sphere_mesh.triangles.size()
              << " sphere_euler=" << eulerCharacteristic(sphere_mesh)
              << '\n';
    return 0;
}
