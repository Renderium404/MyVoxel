#ifndef MYVOXEL_MODELING_SHAPE_SHAPEVOXELIZATION_H
#define MYVOXEL_MODELING_SHAPE_SHAPEVOXELIZATION_H

#include <cstdint>

#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Topology/Shape.h"
#include "MyVoxel/Topology/Topology_Shape.h"

namespace MyVoxel
{
namespace Modeling
{

class VoxelizationWorkspace;

// 记录连续Shape体素化过程中的候选根、包围盒生成、分类、采样和压缩数量。
struct VoxelizationStatistics
{
    VoxelizationStatistics();

    // 清空全部体素化统计数据。
    void reset();
    // 累加另一次体素化统计数据。
    void accumulate(const VoxelizationStatistics& other);

    std::uint64_t rootCandidateCount; // 连续Shape包围盒覆盖的第0层候选根数量。
    std::uint64_t createdRootCount; // 体素化后实际保存的第0层根树数量。
    std::uint64_t gridCellBoundsCount; // 通过VoxelGrid和体素地址直接计算包围盒的次数。
    std::uint64_t derivedChildBoundsCount; // 通过父包围盒二分生成子包围盒的次数。
    std::uint64_t visitedCellCount; // 实际执行空间分类的体素数量。
    std::uint64_t outsideCellCount; // 被分类为完全位于Shape外部的体素数量。
    std::uint64_t insideCellCount; // 被分类为完全位于Shape内部的体素数量。
    std::uint64_t intersectingCellCount; // 被分类为与Shape边界相交的体素数量。
    std::uint64_t centerSampleCount; // 最高层相交体素执行中心采样的次数。
    std::uint64_t splitCount; // 创建普通节点细分结构的次数。
    std::uint64_t maskLeafBuildCount; // 直接计算64位掩码叶材料掩码的次数。
    std::uint64_t maskLeafStoredCount; // 最终以部分材料MaskLeaf保存的次数。
    std::uint64_t mergeSuccessCount; // 八个子体素状态一致并向上折叠的次数。
};

/// 局部Topology_Shape体素化

// 使用指定体素网格将局部数学Shape离散为单位变换下的VoxelShape。
VoxelShape voxelize(const Topology_Shape& topology, const VoxelGrid& grid);
// 使用指定体素网格将局部数学Shape离散为单位变换下的VoxelShape，并记录统计。
VoxelShape voxelize(const Topology_Shape& topology, const VoxelGrid& grid, VoxelizationStatistics& statistics);
// 使用指定基础体素边长和最高层级将局部数学Shape离散为单位变换下的VoxelShape。
VoxelShape voxelize(const Topology_Shape& topology, double baseVoxelEdgeLength, VoxelLevel maximumLevel);
// 使用指定基础体素边长和最高层级体素化局部数学Shape，并记录统计。
VoxelShape voxelize(const Topology_Shape& topology, double baseVoxelEdgeLength, VoxelLevel maximumLevel, VoxelizationStatistics& statistics);

/// Shape实例局部体素化

// 在Shape自身局部空间完成体素化，并将Shape实例变换传递给结果VoxelShape。
VoxelShape voxelize(const Shape& shape, const VoxelGrid& localGrid);
// 在Shape自身局部空间完成体素化，将Shape实例变换传递给结果，并记录统计。
VoxelShape voxelize(const Shape& shape, const VoxelGrid& localGrid, VoxelizationStatistics& statistics);
// 使用指定基础体素边长和最高层级在Shape自身局部空间完成体素化。
VoxelShape voxelize(const Shape& shape, double baseVoxelEdgeLength, VoxelLevel maximumLevel);
// 使用指定基础体素边长和最高层级在Shape自身局部空间完成体素化，并记录统计。
VoxelShape voxelize(const Shape& shape, double baseVoxelEdgeLength, VoxelLevel maximumLevel, VoxelizationStatistics& statistics);

/// Shape实例对齐体素化

// 将Shape直接体素化到reference的体素地址空间中，结果与reference使用完全相同的VoxelGrid和空间变换。
VoxelShape voxelizeAligned(const Shape& shape, const VoxelShape& reference);
// 将Shape直接体素化到reference的体素地址空间中，并记录统计。
VoxelShape voxelizeAligned(const Shape& shape, const VoxelShape& reference, VoxelizationStatistics& statistics);

/// 工作区对齐体素化

// 使用可复用工作区将Shape体素化到reference的体素地址空间中。
//
// 返回结果由workspace持有，在下一次使用该workspace体素化、reset或release前有效。
const VoxelShape& voxelizeAligned(const Shape& shape, const VoxelShape& reference, VoxelizationWorkspace& workspace);
// 使用可复用工作区执行对齐体素化，并记录统计。
const VoxelShape& voxelizeAligned(const Shape& shape, const VoxelShape& reference,
                                  VoxelizationWorkspace& workspace, VoxelizationStatistics& statistics);

}
}

#endif // MYVOXEL_MODELING_SHAPE_SHAPEVOXELIZATION_H
