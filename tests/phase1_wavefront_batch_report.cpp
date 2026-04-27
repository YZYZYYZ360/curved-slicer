#include "app/pipeline.h"
#include "io/config_loader.h"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

namespace {

int fail(const std::string& message)
{
    std::cerr << "phase1_wavefront_batch_report failed: " << message << '\n';
    return 1;
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

int main(int argc, char** argv)
{
    try {
        double spacing_mm = 0.5;
        if (argc >= 2) {
            spacing_mm = std::stod(argv[1]);
        }

        cslc::PipelineConfig config = cslc::loadPipelineConfig(sourceRoot() / "config" / "batch_three_models.toml");
        config.algorithm.mode = "wavefront";
        config.voxel.spacing_mm = spacing_mm;
        config.voxel.padding_mm = 0.0;
        config.io.debug_dump_intermediates = true;
        config.io.output_root = sourceRoot() / "runs" / "phase1_wavefront";

        const cslc::BatchReport report = cslc::runBatch(config);
        cslc::printBatchReport(report, std::cout);

        if (report.success_count != config.io.models.size() || report.failure_count != 0) {
            return fail("all configured models should complete");
        }

        for (const cslc::ModelReport& model : report.models) {
            if (model.connected_components != 1) {
                return fail("expected connected_components=1 for " + model.name);
            }
        }
    } catch (const std::exception& ex) {
        return fail(ex.what());
    }

    return 0;
}
