#include <iostream>
#include <string>
#include <vector>

namespace {

struct Options {
    std::string layers;
    std::string robot;
    std::string nozzle;
    std::string yaw_samples;
    std::string top_k;
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
        } else if (arg == "--layers") {
            takeValue(i, argc, argv, arg, options.layers, options);
        } else if (arg == "--robot") {
            takeValue(i, argc, argv, arg, options.robot, options);
        } else if (arg == "--nozzle") {
            takeValue(i, argc, argv, arg, options.nozzle, options);
        } else if (arg == "--yaw-samples") {
            takeValue(i, argc, argv, arg, options.yaw_samples, options);
        } else if (arg == "--top-k") {
            takeValue(i, argc, argv, arg, options.top_k, options);
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
    std::cout << "Usage: toolpath_validate --layers <dir> --robot <yaml> --nozzle <yaml> "
                 "--yaw-samples <int> --top-k <int> --out <dir>\n";
}

}  // namespace

int main(int argc, char** argv)
{
    const Options options = parseArgs(argc, argv);

    std::cout << "v6.2 toolpath_validate\n";
    std::cout << "Skeleton, pipeline implementation pending\n";
    if (options.help) {
        printUsage();
    }

    std::cout << "layers=" << valueOrUnset(options.layers) << '\n';
    std::cout << "robot=" << valueOrUnset(options.robot) << '\n';
    std::cout << "nozzle=" << valueOrUnset(options.nozzle) << '\n';
    std::cout << "yaw_samples=" << valueOrUnset(options.yaw_samples) << '\n';
    std::cout << "top_k=" << valueOrUnset(options.top_k) << '\n';
    std::cout << "out=" << valueOrUnset(options.out) << '\n';
    for (const std::string& warning : options.warnings) {
        std::cout << "warning: " << warning << '\n';
    }
    return 0;
}
