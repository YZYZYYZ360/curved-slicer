#include "field/wavefront.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <vector>

namespace cslc {
namespace {

struct QueueItem {
    double distance = 0.0;
    VoxelIndex voxel;
};

bool operator>(const QueueItem& lhs, const QueueItem& rhs)
{
    return lhs.distance > rhs.distance;
}

std::size_t scalarIndex(const ScalarField& field, int x, int y, int z)
{
    return static_cast<std::size_t>(x) +
        static_cast<std::size_t>(field.nx) *
            (static_cast<std::size_t>(y) + static_cast<std::size_t>(field.ny) * static_cast<std::size_t>(z));
}

std::vector<VoxelIndex> bottomSeeds(const VoxelGrid& grid)
{
    const std::vector<VoxelIndex> occupied = grid.occupiedVoxels();
    if (occupied.empty()) {
        return {};
    }

    int min_z = occupied.front().z;
    for (const VoxelIndex& voxel : occupied) {
        min_z = std::min(min_z, voxel.z);
    }

    std::vector<VoxelIndex> seeds;
    for (const VoxelIndex& voxel : occupied) {
        if (voxel.z == min_z) {
            seeds.push_back(voxel);
        }
    }
    return seeds;
}

}  // namespace

ScalarField solveWavefront(const VoxelGrid& grid, const WavefrontParams& params)
{
    if (grid.nx() <= 0 || grid.ny() <= 0 || grid.nz() <= 0) {
        throw std::runtime_error("solveWavefront requires a non-empty voxel grid");
    }
    if (grid.occupiedVoxelCount() == 0) {
        throw std::runtime_error("solveWavefront requires occupied voxels");
    }
    if (params.seed_strategy != "bottom") {
        throw std::runtime_error("solveWavefront currently supports seed_strategy=bottom");
    }

    ScalarField phi;
    phi.bbox = grid.bbox();
    phi.spacing = grid.spacing();
    phi.nx = grid.nx();
    phi.ny = grid.ny();
    phi.nz = grid.nz();
    const auto total_voxels = static_cast<std::size_t>(phi.nx) *
        static_cast<std::size_t>(phi.ny) * static_cast<std::size_t>(phi.nz);
    phi.values.assign(total_voxels, std::numeric_limits<double>::infinity());

    // SFF.cpp:1060  threadFun_first 入口；新实现收敛为 solveWavefront 的第一层种子扩散。
    // SFF.cpp:1067  PROFILE_FUNC 已剥离；Phase 1 用 pipeline 统一记录 M6。
    // SFF.cpp:1069  voxel_num 对应 bottomSeeds(grid).size()。
    // SFF.cpp:1070  left/right 分片边界检查由单队列 Dijkstra 避免。
    // SFF.cpp:1074  threadFun_first 主循环在这里转为多源初始化。
    // SFF.cpp:1076  x 方向 heightInterval 窗口被 6-neighbor 单步传播取代。
    // SFF.cpp:1078  y 方向 heightInterval 窗口被 6-neighbor 单步传播取代。
    // SFF.cpp:1080  z 方向 1.5*heightInterval 窗口被 clean wavefront 队列取代。
    // SFF.cpp:1084  bDotInBox/getVoxel 条件对应 grid.inBounds/grid.occupied。
    // SFF.cpp:1088  glm::distance 改为每条网格边累加 grid.spacing()。
    // SFF.cpp:1089  sign=-1 翻转已剥离（v3 §7 #1 废弃）。
    // SFF.cpp:1095  curvPenalty 项已剥离（v3 §7 #5 废弃）。
    // SFF.cpp:1100  disVal 仅保留欧氏波前距离，不再叠加 sign/lambda_c。
    // SFF.cpp:1102  voxel_layer.find 对应 phi.values 的 infinity 判定。
    // SFF.cpp:1104  取较小 disVal 对应 Dijkstra 松弛。
    // SFF.cpp:1111  新体素距离写入 phi.values。
    // SFF.cpp:1118  app_log profile 已剥离，M6 由 pipeline 汇总。
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;
    const std::vector<VoxelIndex> seeds = bottomSeeds(grid);
    if (seeds.empty()) {
        throw std::runtime_error("solveWavefront could not find bottom seeds");
    }

    for (const VoxelIndex& seed : seeds) {
        phi.values[scalarIndex(phi, seed.x, seed.y, seed.z)] = 0.0;
        queue.push({0.0, seed});
    }

    // SFF.cpp:1130  threadFun_CurveLayer 入口；后续层统一由队列弹出最近体素。
    // SFF.cpp:1137  层内 profile 已剥离，避免把 benchmark 与算法混在一起。
    // SFF.cpp:1143  seed 分片 for-loop 对应 priority_queue 主循环。
    // SFF.cpp:1145  x 方向 1.5*heightInterval 搜索窗口收敛为相邻体素传播。
    // SFF.cpp:1147  y 方向 1.5*heightInterval 搜索窗口收敛为相邻体素传播。
    // SFF.cpp:1149  z 方向 1.5*heightInterval 搜索窗口收敛为相邻体素传播。
    // SFF.cpp:1151  getVoxel != 255/2 状态机改为 final distance 是否已更优。
    // SFF.cpp:1154  geoDist 仍是空间距离，这里用 axis-aligned edge length。
    // SFF.cpp:1155  curvPenalty 项已剥离（v3 §7 #5 废弃）。
    // SFF.cpp:1160  disVal = geoDist + heightInterval*layer 简化为累计最短距离。
    // SFF.cpp:1163  thread_local map 查找对应邻居松弛。
    // SFF.cpp:1165  保留“更小值覆盖”语义。
    // SFF.cpp:1172  首次到达体素写入距离场。
    // SFF.cpp:1180  CurveLayer profile 已剥离。
    constexpr int kNeighborCount = 6;
    constexpr int dx[kNeighborCount] = {1, -1, 0, 0, 0, 0};
    constexpr int dy[kNeighborCount] = {0, 0, 1, -1, 0, 0};
    constexpr int dz[kNeighborCount] = {0, 0, 0, 0, 1, -1};

    // SFF.cpp:1195  getDistanceFiled 主入口迁移为 solveWavefront。
    // SFF.cpp:1203  curvelayer_voxels 被 phi.values 取代。
    // SFF.cpp:1204  next_curvelayer_voxels 被 priority_queue 隐式取代。
    // SFF.cpp:1205  next_initVoxels 对应 bottom seeds 与后续队列邻居。
    // SFF.cpp:1208  iso 选择不在 wavefront 内完成，交给 planIsoLevels。
    // SFF.cpp:1209  precision_ 对应 grid.spacing()。
    // SFF.cpp:1211  layer 计数由距离值 / height_interval_mm 隐含表达。
    // SFF.cpp:1247  getFirstCurvelayer 对应 bottomSeeds(grid)。
    // SFF.cpp:1257  第一层 threadFun_first 调用对应上方多源初始化。
    // SFF.cpp:1267  getSingleMeshlayer 已移至 surface/extractIsoSurface。
    // SFF.cpp:1302  while(true) 主循环对应 Dijkstra queue loop。
    // SFF.cpp:1311  线程数启发式（保留为 serial_threshold 配置项）；当前串行保证确定性。
    // SFF.cpp:1319  std::async CurveLayer 分发未迁移，Phase 1 先保持单线程 baseline。
    // SFF.cpp:1347  thread_local_maps 合并被 priority_queue 松弛取代。
    // SFF.cpp:1372  heightInterval 边界标记对应 planIsoLevels 的等值层规划。
    // SFF.cpp:1391  #if 0 旧实现未迁移（v3 §7 #6）。
    // SFF.cpp:1432  活动 #else 的局部 next seed 搜索由邻接传播覆盖。
    // SFF.cpp:1516  empty/limit break 对应 queue 耗尽。
    // SFF.cpp:1531  IsoSelectionMode::Median 分支已剥离（v3 §7 #4 废弃）。
    // SFF.cpp:1534  getMeshCurvelayer 已移至 surface/extractIsoSurface。
    while (!queue.empty()) {
        const QueueItem item = queue.top();
        queue.pop();

        const std::size_t current_index = scalarIndex(phi, item.voxel.x, item.voxel.y, item.voxel.z);
        if (item.distance > phi.values[current_index]) {
            continue;
        }

        for (int neighbor = 0; neighbor < kNeighborCount; ++neighbor) {
            const VoxelIndex next{
                item.voxel.x + dx[neighbor],
                item.voxel.y + dy[neighbor],
                item.voxel.z + dz[neighbor],
            };
            if (!grid.inBounds(next.x, next.y, next.z) || !grid.occupied(next.x, next.y, next.z)) {
                continue;
            }

            const std::size_t next_index = scalarIndex(phi, next.x, next.y, next.z);
            const double candidate = item.distance + grid.spacing();
            if (candidate < phi.values[next_index]) {
                phi.values[next_index] = candidate;
                queue.push({candidate, next});
            }
        }
    }

    return phi;
}

}  // namespace cslc
