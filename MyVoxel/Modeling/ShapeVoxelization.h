#ifndef MYVOXEL_MODELING_SHAPEVOXELIZATION_H
#define MYVOXEL_MODELING_SHAPEVOXELIZATION_H

#include <cstdint>

#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Geometry/Shape.h"
#include "MyVoxel/Geometry/ShapeInstance.h"

namespace MyVoxel
{
namespace Modeling
{

// 记录连续Shape体素化过程中的候选根体素和节点分类数量。
struct VoxelizationStatistics
{
    VoxelizationStatistics();

    // 清空全部体素化统计数据。
    void reset();

    // 累加另一次体素化统计数据。
    void accumulate(const VoxelizationStatistics& other);

    std::uint64_t rootCandidateCount; // 连续Shape局部包围盒覆盖的第0层候选根体素数量。
    std::uint64_t createdRootCount; // 体素化后实际保留的第0层根树数量。
    std::uint64_t visitedCellCount; // 实际执行连续几何分类的体素数量。
    std::uint64_t outsideCellCount; // 被分类为完全位于连续Shape外部的体素数量。
    std::uint64_t insideCellCount; // 被分类为完全位于连续Shape内部的体素数量。
    std::uint64_t intersectingCellCount; // 被分类为与连续Shape边界相交的体素数量。
    std::uint64_t centerSampleCount; // 最高层级相交体素执行中心点采样的次数。
    std::uint64_t splitCount; // 体素化过程中执行的节点细分次数。
    std::uint64_t mergeSuccessCount; // 子节点状态一致并成功向上合并的次数。
};

/// 局部Shape体素化

// 使用指定体素网格将连续局部Shape离散为单位变换下的VoxelShape。
VoxelShape voxelize(const Geometry::Shape& shape, const VoxelGrid& grid);

// 使用指定体素网格将连续局部Shape离散为单位变换下的VoxelShape，并记录统计数据。
VoxelShape voxelize(const Geometry::Shape& shape, const VoxelGrid& grid, VoxelizationStatistics& statistics);

// 使用指定基础体素边长和最高层级将连续局部Shape离散为单位变换下的VoxelShape。
VoxelShape voxelize(const Geometry::Shape& shape, double baseVoxelEdgeLength, VoxelLevel maximumLevel);

// 使用指定基础体素边长和最高层级将连续局部Shape离散为单位变换下的VoxelShape，并记录统计数据。
VoxelShape voxelize(const Geometry::Shape& shape, double baseVoxelEdgeLength, VoxelLevel maximumLevel, VoxelizationStatistics& statistics);

/// Shape实例体素化

// 在Shape局部坐标中完成体素化，并将ShapeInstance变换传递给结果VoxelShape。
VoxelShape voxelize(const Geometry::ShapeInstance& instance, const VoxelGrid& grid);

// 在Shape局部坐标中完成体素化，将实例变换传递给结果，并记录统计数据。
VoxelShape voxelize(const Geometry::ShapeInstance& instance, const VoxelGrid& grid, VoxelizationStatistics& statistics);

// 使用指定基础体素边长和最高层级体素化ShapeInstance。
VoxelShape voxelize(const Geometry::ShapeInstance& instance, double baseVoxelEdgeLength, VoxelLevel maximumLevel);

// 使用指定基础体素边长和最高层级体素化ShapeInstance，并记录统计数据。
VoxelShape voxelize(const Geometry::ShapeInstance& instance, double baseVoxelEdgeLength, VoxelLevel maximumLevel, VoxelizationStatistics& statistics);

}
}

#endif // MYVOXEL_MODELING_SHAPEVOXELIZATION_H