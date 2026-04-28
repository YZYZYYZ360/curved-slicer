#pragma once

#include "core/types.h"
#include "field/poisson.h"

#include <filesystem>
#include <string>
#include <vector>

namespace cslc {

struct IsoExtractParams {
    double layer_thickness_mm = 0.8;
    std::string iso_spacing = "uniform";
    int max_layers = 500;
    double phi_start_offset = 0.0;
};

struct IsoMesh {
    std::vector<Vec3> vertices;
    std::vector<Tri> triangles;
    double iso_value = 0.0;
    int layer_id = -1;
};

std::vector<double> planIsoLevels(const ScalarField& phi, const IsoExtractParams& params);
IsoMesh extractIsoSurface(const ScalarField& phi, double iso_value, int layer_id);
int countConnectedComponents(const IsoMesh& mesh);
void writeIsoMeshPly(const IsoMesh& mesh, const std::filesystem::path& output_path);
void writeIsoMeshStl(const IsoMesh& mesh, const std::filesystem::path& output_path);

}  // namespace cslc
