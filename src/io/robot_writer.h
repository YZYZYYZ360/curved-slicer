#pragma once

#include <filesystem>
#include <ostream>
#include <string>

namespace cslc {

class RobotWriter {
public:
    explicit RobotWriter(std::ostream& output);

    void writeComment(const std::string& text);
    void writePhase0PlaceholderKrl(const std::string& model_name);

private:
    std::ostream* output_ = nullptr;
};

void writePhase0PlaceholderKrl(const std::filesystem::path& output_path,
                               const std::string& model_name);

}  // namespace cslc
