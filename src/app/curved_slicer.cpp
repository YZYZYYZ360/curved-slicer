#include <iostream>
#include <string>
#include <vector>

namespace {

struct Options {
    std::string input;
    std::string robot;
    std::string nozzle;
    std::string voxel;
    std::string layer_height;
    std::string weights;
    std::string enable_ik_field;
    std::string out;
    bool help = false;
    std::vector<std::string> warnings;
};

std::string valueOrUnset(const std::string& value)
{
    return value.empty() ? "(unset)" : value;
}

bool takeValue(int& index, int argc, char** argv, const std::string& flag, std::string& target, Options& options)
{
    if (index + 1 >= argc) {
        options.warnings.push_back(flag + " expects a value");
        return false;
    }
    target = argv[++index];
    return true;
}

Options parseArgs(int argc, char** argv)
{
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            options.help = true;
        } else if (arg == "--input") {
            takeValue(i, argc, argv, arg, options.input, options);
        } else if (arg == "--robot") {
            takeValue(i, argc, argv, arg, options.robot, options);
        } else if (arg == "--nozzle") {
            takeValue(i, argc, argv, arg, options.nozzle, options);
        } else if (arg == "--voxel") {
            takeValue(i, argc, argv, arg, options.voxel, options);
        } else if (arg == "--layer-height") {
            takeValue(i, argc, argv, arg, options.layer_height, options);
        } else if (arg == "--weights") {
            takeValue(i, argc, argv, arg, options.weights, options);
        } else if (arg == "--enable-ik-field") {
            takeValue(i, argc, argv, arg, options.enable_ik_field, options);
        } else if (arg == "--out") {
            takeValue(i, argc, argv, arg, options.out, options);
        } else {
            options.warnings.push_back("unknown argument: " + arg);
        }
    }
    return options;
}

void printUsage()
{
    std::cout << "Usage: curved_slicer --input <stl> --robot <yaml> --nozzle <yaml> --voxel <mm> "
                 "--layer-height <mm> --weights <yaml> --enable-ik-field <bool> --out <dir>\n";
}

}  // namespace

int main(int argc, char** argv)
{
    const Options options = parseArgs(argc, argv);

    std::cout << "v6.2 curved_slicer\n";
    std::cout << "Skeleton, pipeline implementation pending\n";
    if (options.help) {
        printUsage();
    }

    std::cout << "input=" << valueOrUnset(options.input) << '\n';
    std::cout << "robot=" << valueOrUnset(options.robot) << '\n';
    std::cout << "nozzle=" << valueOrUnset(options.nozzle) << '\n';
    std::cout << "voxel=" << valueOrUnset(options.voxel) << '\n';
    std::cout << "layer_height=" << valueOrUnset(options.layer_height) << '\n';
    std::cout << "weights=" << valueOrUnset(options.weights) << '\n';
    std::cout << "enable_ik_field=" << valueOrUnset(options.enable_ik_field) << '\n';
    std::cout << "out=" << valueOrUnset(options.out) << '\n';
    for (const std::string& warning : options.warnings) {
        std::cout << "warning: " << warning << '\n';
    }
    return 0;
}
