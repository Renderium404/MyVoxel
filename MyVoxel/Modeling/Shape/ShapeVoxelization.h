#ifndef MYVOXEL_MODELING_SHAPE_SHAPEVOXELIZATION_H
#define MYVOXEL_MODELING_SHAPE_SHAPEVOXELIZATION_H

#include <cstdint>

#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Instance/Shape.h"
#include "MyVoxel/Topology/Shape/Topology_Shape.h"

namespace MyVoxel
{
namespace Modeling
{

class VoxelizationWorkspace;

// 记录连续Shape窄带TSDF体素化过程中的候选根、距离采样、细分和叶块生成数量。
struct VoxelizationStatistics
{
    VoxelizationStatistics();

    // 清空全部体素化统计数据。
    void reset();
    // 累加另一次体素化统计数据。
    void accumulate(const VoxelizationStatistics& other);

    std::uint64_t rootCandidateCount; // Shape查询包围盒向外扩展B后覆盖的第0层候选根数量。
    std::uint64_t createdRootCount; // 体素化后实际保存的非背景第0层根树数量。
    std::uint64_t gridCellBoundsCount; // 通过VoxelGrid和体素地址直接计算包围盒的次数。
    std::uint64_t derivedChildBoundsCount; // 通过父包围盒二分生成子包围盒的次数。
    std::uint64_t visitedCellCount; // 实际执行中心有符号距离判定的逻辑体素数量。
    std::uint64_t outsideCellCount; // 根据中心距离和包围半径证明整个区域截断后恒为+B的体素数量。
    std::uint64_t insideCellCount; // 根据中心距离和包围半径证明整个区域截断后恒为-B的体素数量。
    std::uint64_t intersectingCellCount; // 无法证明整块已经饱和到±B、需要继续保存窄带信息的体素数量。
    std::uint64_t centerSampleCount; // 逻辑体素中心执行精确signed-distance查询的次数。
    std::uint64_t splitCount; // 创建普通Node细分结构的次数。
    std::uint64_t maskLeafBuildCount; // 直接生成64个最高层TSDF距离样本的MaskLeaf次数。
    std::uint64_t maskLeafStoredCount; // 最终实际以非统一MaskLeaf保存的次数。
    std::uint64_t mergeSuccessCount; // 生成或root prune过程中发生无损Value归约的事件次数。
};

/// 局部Topology_Shape体素化

// 使用指定体素网格将支持精确signed distance的局部数学Shape离散为单位变换下的窄带TSDF VoxelShape。
VoxelShape voxelize(const Topology_Shape& topology, const VoxelGrid& grid);
// 使用指定体素网格执行窄带TSDF体素化，并记录统计。
VoxelShape voxelize(const Topology_Shape& topology, const VoxelGrid& grid, VoxelizationStatistics& statistics);
// 使用指定基础体素边长和最高层级执行窄带TSDF体素化。
VoxelShape voxelize(const Topology_Shape& topology, double baseVoxelEdgeLength, VoxelLevel maximumLevel);
// 使用指定基础体素边长和最高层级执行窄带TSDF体素化，并记录统计。
VoxelShape voxelize(const Topology_Shape& topology, double baseVoxelEdgeLength, VoxelLevel maximumLevel, VoxelizationStatistics& statistics);

/// Shape实例局部体素化

// 在Shape自身局部空间完成窄带TSDF体素化，并将Shape实例变换传递给结果VoxelShape。
VoxelShape voxelize(const Shape& shape, const VoxelGrid& localGrid);
// 在Shape自身局部空间完成窄带TSDF体素化，将Shape实例变换传递给结果，并记录统计。
VoxelShape voxelize(const Shape& shape, const VoxelGrid& localGrid, VoxelizationStatistics& statistics);
// 使用指定基础体素边长和最高层级在Shape自身局部空间完成窄带TSDF体素化。
VoxelShape voxelize(const Shape& shape, double baseVoxelEdgeLength, VoxelLevel maximumLevel);
// 使用指定基础体素边长和最高层级在Shape自身局部空间完成窄带TSDF体素化，并记录统计。
VoxelShape voxelize(const Shape& shape, double baseVoxelEdgeLength, VoxelLevel maximumLevel, VoxelizationStatistics& statistics);

/// Shape实例对齐体素化

// 将Shape直接体素化到reference的体素地址空间中，结果继承reference的VoxelGrid、backgroundDistance和空间变换。
VoxelShape voxelizeAligned(const Shape& shape, const VoxelShape& reference);
// 将Shape直接体素化到reference的体素地址空间中，并记录统计。
VoxelShape voxelizeAligned(const Shape& shape, const VoxelShape& reference, VoxelizationStatistics& statistics);

/// 工作区对齐体素化

// 使用可复用工作区将Shape体素化到reference的体素地址空间中。
// 返回结果由workspace持有，在下一次使用该workspace体素化、reset或release前有效。
const VoxelShape& voxelizeAligned(const Shape& shape, const VoxelShape& reference, VoxelizationWorkspace& workspace);
// 使用可复用工作区执行对齐窄带TSDF体素化，并记录统计。
const VoxelShape& voxelizeAligned(const Shape& shape, const VoxelShape& reference, VoxelizationWorkspace& workspace, VoxelizationStatistics& statistics);

}
}

#endif // MYVOXEL_MODELING_SHAPE_SHAPEVOXELIZATION_H