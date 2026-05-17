#include "io/KRLWriter.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace cslc {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

double radToDeg(double rad)
{
    return rad * 180.0 / kPi;
}

}  // namespace

void KRLWriter::write(
    const std::string& path,
    const std::string& program_name,
    const std::vector<LinSegment>& segments)
{
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("KRLWriter: failed to open output path: " + path);
    }

    output << std::fixed << std::setprecision(4);
    output << "&ACCESS RVP\n";
    output << "&REL 1\n";
    output << "DEF " << program_name << "()\n";
    output << "  INI\n";
    output << "  $BASE = $WORLD\n";
    output << "  $TOOL = $NULLFRAME\n";
    output << "  $VEL.CP = 0.0500  ; m/s default\n";
    output << "  PTP HOME\n\n";
    output << "  ; -- generated segments --\n";

    for (const LinSegment& segment : segments) {
        output << "  $VEL.CP = " << (segment.velocity_mm_s / 1000.0) << '\n';
        output << "  LIN {A1 " << radToDeg(segment.q_rad(0))
               << ", A2 " << radToDeg(segment.q_rad(1))
               << ", A3 " << radToDeg(segment.q_rad(2))
               << ", A4 " << radToDeg(segment.q_rad(3))
               << ", A5 " << radToDeg(segment.q_rad(4))
               << ", A6 " << radToDeg(segment.q_rad(5))
               << "} C_DIS\n";
    }

    output << "\n";
    output << "  PTP HOME\n";
    output << "END\n";
}

}  // namespace cslc
