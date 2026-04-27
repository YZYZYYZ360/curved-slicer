#include "io/config_loader.h"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>

namespace {

int fail(const std::string& message)
{
    std::cerr << "config_loader_unit_test failed: " << message << '\n';
    return 1;
}

bool nearlyEqual(double lhs, double rhs, double eps = 1e-12)
{
    return std::abs(lhs - rhs) <= eps;
}

std::filesystem::path sourceRoot()
{
#ifdef CSLC_SOURCE_DIR
    return std::filesystem::path(CSLC_SOURCE_DIR);
#else
    return std::filesystem::current_path();
#endif
}

}  // namespace

int main()
{
    try {
        const std::filesystem::path config_path = sourceRoot() / "config" / "batch_three_models.toml";
        const cslc::PipelineConfig config = cslc::loadPipelineConfig(config_path);

        if (!nearlyEqual(config.kuka.limits[1].min_deg, -195.0)) {
            return fail("kuka.limits.a2.min_deg should be -195.0");
        }
        if (!nearlyEqual(config.kuka.limits[2].max_deg, 150.0)) {
            return fail("kuka.limits.a3.max_deg should be 150.0");
        }
        if (config.io.models.size() != 3) {
            return fail("batch_three_models.toml should contain three models");
        }
        if (config.io.models[2].stl_path.u8string().find(u8"毛主席头雕") == std::string::npos) {
            return fail("Chinese model path should survive TOML parsing");
        }
        if (!nearlyEqual(config.algorithm.field.laplacian.tolerance, 1e-6) ||
            config.algorithm.field.poisson.max_iterations != 500) {
            return fail("algorithm.field params should parse");
        }
        if (!config.metrics.enabled || !config.metrics.output_per_layer) {
            return fail("metrics flags should parse");
        }
        if (config.logging.level != "info") {
            return fail("logging.level should parse");
        }
    } catch (const std::exception& ex) {
        return fail(ex.what());
    }

    return 0;
}
