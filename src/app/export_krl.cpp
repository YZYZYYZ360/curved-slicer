#include <iostream>
#include <string>
#include <vector>

namespace {

struct Options {
    std::string trajectory;
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
        } else if (arg == "--trajectory") {
            takeValue(i, argc, argv, arg, options.trajectory, options);
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
    std::cout << "Usage: export_krl --trajectory <csv> --out <dir>\n";
}

}  // namespace

int main(int argc, char** argv)
{
    const Options options = parseArgs(argc, argv);

    std::cout << "v6.2 export_krl\n";
    std::cout << "Skeleton, pipeline implementation pending\n";
    if (options.help) {
        printUsage();
    }

    std::cout << "trajectory=" << valueOrUnset(options.trajectory) << '\n';
    std::cout << "out=" << valueOrUnset(options.out) << '\n';
    for (const std::string& warning : options.warnings) {
        std::cout << "warning: " << warning << '\n';
    }
    return 0;
}
