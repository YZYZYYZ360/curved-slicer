#include "io/robot_writer.h"

#include <fstream>
#include <stdexcept>

namespace cslc {

RobotWriter::RobotWriter(std::ostream& output)
    : output_(&output)
{
}

void RobotWriter::writeComment(const std::string& text)
{
    (*output_) << "; " << text << '\n';
}

void RobotWriter::writePhase0PlaceholderKrl(const std::string& model_name)
{
    writeComment("curved-slicer Phase 0 placeholder");
    writeComment("model: " + model_name);
    writeComment("no KRL toolpath is generated in Phase 0");
}

void writePhase0PlaceholderKrl(const std::filesystem::path& output_path,
                               const std::string& model_name)
{
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Failed to open KRL output: " + output_path.u8string());
    }

    RobotWriter writer(output);
    writer.writePhase0PlaceholderKrl(model_name);
}

}  // namespace cslc
