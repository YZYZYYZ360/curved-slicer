#include "io/config_loader.h"

#include <toml++/toml.hpp>

#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

namespace cslc {
namespace {

std::filesystem::path resolvePath(const std::filesystem::path& config_dir,
                                  const std::string& value)
{
    std::filesystem::path path = std::filesystem::u8path(value);
    if (path.is_absolute()) {
        return path.lexically_normal();
    }
    return (config_dir / path).lexically_normal();
}

const toml::table* tableAt(const toml::table& root, std::initializer_list<std::string_view> keys)
{
    const toml::table* current = &root;
    for (std::string_view key : keys) {
        const toml::node* node = current->get(key);
        if (node == nullptr) {
            return nullptr;
        }
        current = node->as_table();
        if (current == nullptr) {
            return nullptr;
        }
    }
    return current;
}

std::runtime_error typeError(std::string_view key, std::string_view expected)
{
    return std::runtime_error("Config key '" + std::string(key) + "' should be " + std::string(expected));
}

std::optional<double> nodeDouble(const toml::node* node)
{
    if (node == nullptr) {
        return std::nullopt;
    }
    if (const std::optional<double> value = node->value<double>()) {
        return value;
    }
    if (const std::optional<std::int64_t> value = node->value<std::int64_t>()) {
        return static_cast<double>(*value);
    }
    return std::nullopt;
}

void readString(const toml::table* table, std::string_view key, std::string& target)
{
    if (table == nullptr) {
        return;
    }
    const toml::node* node = table->get(key);
    if (node == nullptr) {
        return;
    }
    const std::optional<std::string> value = node->value<std::string>();
    if (!value) {
        throw typeError(key, "a string");
    }
    target = *value;
}

void readBool(const toml::table* table, std::string_view key, bool& target)
{
    if (table == nullptr) {
        return;
    }
    const toml::node* node = table->get(key);
    if (node == nullptr) {
        return;
    }
    const std::optional<bool> value = node->value<bool>();
    if (!value) {
        throw typeError(key, "a boolean");
    }
    target = *value;
}

void readDouble(const toml::table* table, std::string_view key, double& target)
{
    if (table == nullptr) {
        return;
    }
    const std::optional<double> value = nodeDouble(table->get(key));
    if (value) {
        target = *value;
        return;
    }
    if (table->get(key) != nullptr) {
        throw typeError(key, "a number");
    }
}

void readInt(const toml::table* table, std::string_view key, int& target)
{
    if (table == nullptr) {
        return;
    }
    const toml::node* node = table->get(key);
    if (node == nullptr) {
        return;
    }
    const std::optional<std::int64_t> value = node->value<std::int64_t>();
    if (!value) {
        throw typeError(key, "an integer");
    }
    target = static_cast<int>(*value);
}

void readPath(const toml::table* table,
              std::string_view key,
              const std::filesystem::path& config_dir,
              std::filesystem::path& target)
{
    if (table == nullptr) {
        return;
    }
    const toml::node* node = table->get(key);
    if (node == nullptr) {
        return;
    }
    const std::optional<std::string> value = node->value<std::string>();
    if (!value) {
        throw typeError(key, "a path string");
    }
    target = resolvePath(config_dir, *value);
}

void readDoubleArray3(const toml::table* table, std::string_view key, std::array<double, 3>& target)
{
    if (table == nullptr) {
        return;
    }
    const toml::node* node = table->get(key);
    if (node == nullptr) {
        return;
    }
    const toml::array* array = node->as_array();
    if (array == nullptr || array->size() != target.size()) {
        throw typeError(key, "an array of 3 numbers");
    }
    for (std::size_t i = 0; i < target.size(); ++i) {
        const std::optional<double> value = nodeDouble(array->get(i));
        if (!value) {
            throw typeError(key, "an array of 3 numbers");
        }
        target[i] = *value;
    }
}

void readDoubleArray6(const toml::table* table, std::string_view key, std::array<double, 6>& target)
{
    if (table == nullptr) {
        return;
    }
    const toml::node* node = table->get(key);
    if (node == nullptr) {
        return;
    }
    const toml::array* array = node->as_array();
    if (array == nullptr || array->size() != target.size()) {
        throw typeError(key, "an array of 6 numbers");
    }
    for (std::size_t i = 0; i < target.size(); ++i) {
        const std::optional<double> value = nodeDouble(array->get(i));
        if (!value) {
            throw typeError(key, "an array of 6 numbers");
        }
        target[i] = *value;
    }
}

void readIntVector(const toml::table* table, std::string_view key, std::vector<int>& target)
{
    if (table == nullptr) {
        return;
    }
    const toml::node* node = table->get(key);
    if (node == nullptr) {
        return;
    }
    const toml::array* array = node->as_array();
    if (array == nullptr) {
        throw typeError(key, "an integer array");
    }
    target.clear();
    target.reserve(array->size());
    for (const toml::node& item : *array) {
        const std::optional<std::int64_t> value = item.value<std::int64_t>();
        if (!value) {
            throw typeError(key, "an integer array");
        }
        target.push_back(static_cast<int>(*value));
    }
}

void readMatrix4x4(const toml::table* table,
                   std::string_view key,
                   std::array<std::array<double, 4>, 4>& target)
{
    if (table == nullptr) {
        return;
    }
    const toml::node* node = table->get(key);
    if (node == nullptr) {
        return;
    }
    const toml::array* rows = node->as_array();
    if (rows == nullptr || rows->size() != target.size()) {
        throw typeError(key, "a 4x4 number array");
    }

    for (std::size_t row = 0; row < target.size(); ++row) {
        const toml::array* cols = rows->get(row)->as_array();
        if (cols == nullptr || cols->size() != target[row].size()) {
            throw typeError(key, "a 4x4 number array");
        }
        for (std::size_t col = 0; col < target[row].size(); ++col) {
            const std::optional<double> value = nodeDouble(cols->get(col));
            if (!value) {
                throw typeError(key, "a 4x4 number array");
            }
            target[row][col] = *value;
        }
    }
}

void readVoxelIndex(const toml::table* table, std::string_view key, VoxelIndex& target)
{
    if (table == nullptr) {
        return;
    }
    const toml::node* node = table->get(key);
    if (node == nullptr) {
        return;
    }
    const toml::array* array = node->as_array();
    if (array == nullptr || array->size() != 3) {
        throw typeError(key, "an array of 3 integers");
    }
    const std::optional<std::int64_t> x = array->get(0)->value<std::int64_t>();
    const std::optional<std::int64_t> y = array->get(1)->value<std::int64_t>();
    const std::optional<std::int64_t> z = array->get(2)->value<std::int64_t>();
    if (!x || !y || !z) {
        throw typeError(key, "an array of 3 integers");
    }
    target = {static_cast<int>(*x), static_cast<int>(*y), static_cast<int>(*z)};
}

void parseModelList(const toml::table* io_table,
                    const std::filesystem::path& config_dir,
                    std::vector<ModelConfig>& models)
{
    if (io_table == nullptr) {
        return;
    }

    const toml::node* models_node = io_table->get("models");
    if (models_node == nullptr) {
        return;
    }
    const toml::array* model_array = models_node->as_array();
    if (model_array == nullptr) {
        throw typeError("io.models", "an array of tables");
    }

    models.clear();
    for (const toml::node& node : *model_array) {
        const toml::table* model_table = node.as_table();
        if (model_table == nullptr) {
            throw typeError("io.models", "an array of tables");
        }

        ModelConfig model;
        readString(model_table, "name", model.name);
        readPath(model_table, "stl_path", config_dir, model.stl_path);
        models.push_back(model);
    }
}

void validateModels(const std::vector<ModelConfig>& models)
{
    for (const ModelConfig& model : models) {
        if (model.name.empty()) {
            throw std::runtime_error("Config model is missing name");
        }
        if (model.stl_path.empty()) {
            throw std::runtime_error("Config model '" + model.name + "' is missing stl_path");
        }
    }
}

}  // namespace

PipelineConfig loadPipelineConfig(const std::filesystem::path& config_path)
{
    toml::table root;
    try {
        root = toml::parse_file(config_path.u8string());
    } catch (const toml::parse_error& ex) {
        throw std::runtime_error("Failed to parse config file '" + config_path.u8string() +
            "': " + std::string(ex.description()));
    }

    PipelineConfig config;
    const std::filesystem::path config_dir =
        config_path.has_parent_path() ? config_path.parent_path() : std::filesystem::current_path();

    const toml::table* io = tableAt(root, {"io"});
    readPath(io, "output_root", config_dir, config.io.output_root);
    readPath(io, "runs_csv", config_dir, config.io.runs_csv);
    readBool(io, "debug_dump_intermediates", config.io.debug_dump_intermediates);
    parseModelList(io, config_dir, config.io.models);

    const toml::table* voxel = tableAt(root, {"voxel"});
    readDouble(voxel, "spacing_mm", config.voxel.spacing_mm);
    readDouble(voxel, "padding_mm", config.voxel.padding_mm);
    readDouble(voxel, "sdf_band_mm", config.voxel.sdf_band_mm);

    const toml::table* boundary = tableAt(root, {"field", "boundary"});
    readString(boundary, "strategy", config.field_boundary.strategy);
    readDoubleArray3(boundary, "print_direction", config.field_boundary.print_direction);
    readDouble(boundary, "bottom_dot_threshold", config.field_boundary.bottom_dot_threshold);
    readDouble(boundary, "bottom_sdf_band", config.field_boundary.bottom_sdf_band);
    // v4 §10: geometric_z 策略参数
    readDouble(boundary, "bottom_band_mm", config.field_boundary.bottom_band_mm);
    readDouble(boundary, "top_band_mm", config.field_boundary.top_band_mm);
    readDouble(boundary, "band_min_mm", config.field_boundary.band_min_mm);
    readDouble(boundary, "band_max_mm", config.field_boundary.band_max_mm);

    const toml::table* algorithm = tableAt(root, {"algorithm"});
    readString(algorithm, "pipeline", config.algorithm.pipeline);

    const toml::table* laplacian = tableAt(root, {"algorithm", "field", "laplacian"});
    readInt(laplacian, "max_iterations", config.algorithm.field.laplacian.max_iterations);
    readDouble(laplacian, "tolerance", config.algorithm.field.laplacian.tolerance);

    const toml::table* poisson = tableAt(root, {"algorithm", "field", "poisson"});
    readInt(poisson, "max_iterations", config.algorithm.field.poisson.max_iterations);
    readDouble(poisson, "tolerance", config.algorithm.field.poisson.tolerance);
    readBool(poisson, "use_precondition", config.algorithm.field.poisson.use_precondition);
    readVoxelIndex(poisson, "anchor_voxel", config.algorithm.field.poisson.anchor_voxel);

    const toml::table* smoothing = tableAt(root, {"algorithm", "field", "smoothing"});
    readInt(smoothing, "passes", config.algorithm.field.smoothing.passes);
    readDouble(smoothing, "center_weight", config.algorithm.field.smoothing.center_weight);
    readDouble(smoothing, "neighbor_weight", config.algorithm.field.smoothing.neighbor_weight);
    readBool(smoothing, "preserve_bc", config.algorithm.field.smoothing.preserve_bc);

    const toml::table* iso_surface = tableAt(root, {"iso_surface"});
    readDouble(iso_surface, "layer_thickness_mm", config.iso_surface.layer_thickness_mm);
    readString(iso_surface, "iso_spacing", config.iso_surface.iso_spacing);
    readInt(iso_surface, "max_layers", config.iso_surface.max_layers);
    readDouble(iso_surface, "phi_start_offset", config.iso_surface.phi_start_offset);

    const toml::table* path = tableAt(root, {"path"});
    readString(path, "strategy", config.path.strategy);
    readDouble(path, "line_spacing_mm", config.path.line_spacing_mm);
    readDouble(path, "contour_resample_mm", config.path.contour_resample_mm);
    readInt(path, "smooth_position_window", config.path.smooth_position_window);

    const toml::table* smooth_orientation = tableAt(root, {"path", "smooth_orientation"});
    readDouble(smooth_orientation, "lambda_data", config.path.smooth_orientation.lambda_data);
    readDouble(smooth_orientation, "lambda_smooth", config.path.smooth_orientation.lambda_smooth);

    const toml::table* spline = tableAt(root, {"path", "spline"});
    readBool(spline, "enabled", config.path.spline.enabled);
    readInt(spline, "resample_n_per_segment", config.path.spline.resample_n_per_segment);
    readDouble(spline, "tension", config.path.spline.tension);

    constexpr std::array<std::string_view, 6> axis_names{"a1", "a2", "a3", "a4", "a5", "a6"};
    for (std::size_t axis = 0; axis < axis_names.size(); ++axis) {
        const toml::table* limit = tableAt(root, {"kuka", "limits", axis_names[axis]});
        readDouble(limit, "min_deg", config.kuka.limits[axis].min_deg);
        readDouble(limit, "max_deg", config.kuka.limits[axis].max_deg);
        readDouble(limit, "speed_deg_per_s", config.kuka.limits[axis].speed_deg_per_s);
    }

    const toml::table* robot = tableAt(root, {"kuka", "robot"});
    readInt(robot, "status", config.kuka.robot.status);
    readInt(robot, "turn", config.kuka.robot.turn);
    readInt(robot, "tool_no", config.kuka.robot.tool_no);
    readInt(robot, "base_no", config.kuka.robot.base_no);
    readDouble(robot, "platform_x", config.kuka.robot.platform_x);
    readDouble(robot, "platform_y", config.kuka.robot.platform_y);
    readDouble(robot, "platform_z", config.kuka.robot.platform_z);
    readDouble(robot, "z_offset", config.kuka.robot.z_offset);
    readDoubleArray3(robot, "robot_ini_abc", config.kuka.robot.robot_ini_abc);

    const toml::table* reachability = tableAt(root, {"kuka", "reachability"});
    std::array<double, 3> workpiece_up{
        config.kuka.reachability.workpiece_up.x,
        config.kuka.reachability.workpiece_up.y,
        config.kuka.reachability.workpiece_up.z,
    };
    readDoubleArray3(reachability, "workpiece_up", workpiece_up);
    config.kuka.reachability.workpiece_up = {workpiece_up[0], workpiece_up[1], workpiece_up[2]};
    readDouble(reachability, "min_dot_threshold", config.kuka.reachability.min_dot_threshold);

    const toml::table* kuka_home = tableAt(root, {"kuka", "home"});
    if (kuka_home) {
        readDoubleArray6(kuka_home, "q_deg", config.kuka.home.q_deg);
    }

    readMatrix4x4(tableAt(root, {"kuka", "world_to_base"}), "world_to_base", config.kuka.world_to_base);

    const toml::table* geodesic = tableAt(root, {"path", "geodesic"});
    if (geodesic) {
        readDouble(geodesic, "line_spacing_mm", config.geodesic.line_spacing_mm);
        readDouble(geodesic, "resample_step_mm", config.geodesic.resample_step_mm);
        readString(geodesic, "seed_strategy", config.geodesic.seed_strategy);
    }

    const toml::table* trajectory = tableAt(root, {"trajectory"});
    if (trajectory) {
        readDouble(trajectory, "target_line_speed_mm_per_s", config.trajectory.target_line_speed_mm_per_s);
        readDouble(trajectory, "sample_period_ms", config.trajectory.sample_period_ms);
        readDouble(trajectory, "max_joint_velocity_deg_per_s", config.trajectory.max_joint_velocity_deg_per_s);
        readDouble(trajectory, "max_joint_accel_deg_per_s2", config.trajectory.max_joint_accel_deg_per_s2);
        readDouble(trajectory, "max_joint_delta_per_cycle_deg", config.trajectory.max_joint_delta_per_cycle_deg);
    }

    const toml::table* metrics = tableAt(root, {"metrics"});
    readBool(metrics, "enabled", config.metrics.enabled);
    readIntVector(metrics, "sample_layer_ids", config.metrics.sample_layer_ids);
    readBool(metrics, "output_per_layer", config.metrics.output_per_layer);

    readString(tableAt(root, {"logging"}), "level", config.logging.level);

    validateModels(config.io.models);

    if (config.io.output_root.empty()) {
        config.io.output_root = (config_dir / "../runs/phase0").lexically_normal();
    }
    if (config.io.runs_csv.empty()) {
        config.io.runs_csv = (config.io.output_root / "runs.csv").lexically_normal();
    }

    return config;
}

}  // namespace cslc
