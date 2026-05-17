#include "io/KRLWriter.h"

#include <Eigen/Dense>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

static void require(bool cond, const char* msg)
{
    if (!cond) throw std::runtime_error(msg);
}

static double degToRad(double deg)
{
    return deg * 3.141592653589793238462643383279502884 / 180.0;
}

static std::string readText(const std::filesystem::path& path)
{
    std::ifstream input(path);
    require(input.is_open(), "krl: output file opened");
    return std::string(std::istreambuf_iterator<char>(input),
                       std::istreambuf_iterator<char>());
}

static int countSubstring(const std::string& text, const std::string& needle)
{
    int count = 0;
    size_t pos = 0;
    while ((pos = text.find(needle, pos)) != std::string::npos) {
        ++count;
        pos += needle.size();
    }
    return count;
}

static cslc::KRLWriter::LinSegment makeSegment(
    double a1, double a2, double a3, double a4, double a5, double a6,
    double velocity_mm_s)
{
    cslc::KRLWriter::LinSegment segment;
    segment.q_rad << degToRad(a1), degToRad(a2), degToRad(a3),
                     degToRad(a4), degToRad(a5), degToRad(a6);
    segment.velocity_mm_s = velocity_mm_s;
    return segment;
}

int main()
{
    try {
        const std::vector<cslc::KRLWriter::LinSegment> segments = {
            makeSegment(0.0, -90.0, 90.0, 0.0, 90.0, 0.0, 50.0),
            makeSegment(10.0, -80.0, 80.0, 5.0, 85.0, 1.0, 60.0),
            makeSegment(20.0, -70.0, 70.0, 10.0, 80.0, 2.0, 70.0),
        };

        const auto path =
            std::filesystem::temp_directory_path() / "curved_slicer_test_program.src";
        cslc::KRLWriter::write(path.string(), "test_program", segments);

        const std::string text = readText(path);
        require(text.find("&ACCESS RVP") != std::string::npos, "krl: access header");
        require(text.find("DEF test_program()") != std::string::npos, "krl: DEF line");
        require(text.find("  INI") != std::string::npos, "krl: INI line");
        require(countSubstring(text, "LIN {A1 ") == 3, "krl: three LIN segments");
        require(text.find("0.0000") != std::string::npos, "krl: contains zero angle");
        require(text.find("-90.0000") != std::string::npos, "krl: rad to deg negative");
        require(text.find("90.0000") != std::string::npos, "krl: rad to deg positive");
        require(text.find("$VEL.CP = 0.0500") != std::string::npos,
                "krl: velocity converted mm/s to m/s");
        require(text.find("\nEND\n") != std::string::npos, "krl: END line");

        std::cout << "test_krl_writer_minimal PASSED\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "test_krl_writer_minimal FAILED: " << e.what() << '\n';
        return 1;
    }
}
