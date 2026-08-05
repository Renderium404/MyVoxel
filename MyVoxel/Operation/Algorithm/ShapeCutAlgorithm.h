#ifndef MYVOXEL_OPERATION_ALGORITHM_SHAPECUTALGORITHM_H
#define MYVOXEL_OPERATION_ALGORITHM_SHAPECUTALGORITHM_H

#include <cstdint>

#include "MyVoxel/Core/Change/VoxelChangeSet.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Geometry/ShapeInstance.h"

namespace MyVoxel
{
namespace Operation
{
namespace Algorithm
{

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

// 保存连续几何体直接切削体素体时的内部执行统计。
struct ShapeCutStatistics
{
    ShapeCutStatistics();

    // 清空全部统计数据。
    void reset();

    // 累加另一次连续几何切削统计。
    void accumulate(const ShapeCutStatistics& other);

    std::uint64_t broadPhaseRootCandidateCount; // 几何包围盒覆盖范围内的已有工件根数量。
    std::uint64_t processedRootCount; // 实际进入几何分类或切削的工件根数量。
    std::uint64_t changedRootCount; // 材料实际发生变化的工件根数量。
    std::uint64_t removedRootCount; // 被完整删除的工件根数量。

    std::uint64_t rootBoundsConstructionCount; // 通过VoxelGrid构造第0层根中心和半尺寸的次数。
    std::uint64_t scalarFastClassificationCount; // 通过中心和半尺寸单独执行快速包围盒分类的次数。
    std::uint64_t octantBatchClassificationCount; // 一次批量分类八个等尺寸子体素的次数。
    std::uint64_t derivedChildBoundsCount; // 由父中心和半尺寸直接推导实际递归子体素边界的次数。

    std::uint64_t visitedCellCount; // 实际访问的工件逻辑体素数量。
    std::uint64_t emptySkippedCellCount; // 因工件体素为空而跳过的数量。
    std::uint64_t classifiedCellCount; // 执行连续几何包围盒分类的次数。
    std::uint64_t outsideCellCount; // 被分类为完全位于几何体外部的体素数量。
    std::uint64_t insideCellCount; // 被分类为完全位于几何体内部的体素数量。
    std::uint64_t intersectingCellCount; // 被分类为与几何边界相交的体素数量。

    std::uint64_t centerSampleCount; // 最高层相交体素执行中心采样的次数。
    std::uint64_t centerRemovedCellCount; // 中心采样后被删除的最高层体素数量。
    std::uint64_t removedBranchCount; // 被几何体完整覆盖并直接删除的细分分支数量。

    std::uint64_t splitCount; // 为处理局部相交而创建普通细分节点的次数。
    std::uint64_t maskLeafBuildCount; // 直接生成64位几何掩码的次数。
    std::uint64_t maskLeafOperationCount; // 直接执行64位叶掩码差集的次数。
    std::uint64_t maskLeafRemovedCellCount; // 64位叶掩码路径删除的最高层体素数量。

    std::uint64_t mergeAttemptCount; // 子节点处理后尝试折叠当前体素的次数。
    std::uint64_t mergeSuccessCount; // 成功折叠为空或材料体素的次数。

    double rootClassificationMilliseconds; // 收集并分类几何包围盒范围内已有根的墙钟耗时。
    double rootPreparationMilliseconds; // 建立相交根独立结果树和任务记录的墙钟耗时。
    double intersectingRootExecutionMilliseconds; // 并行执行全部相交根递归切削的墙钟耗时。
    double intersectingRootCpuMilliseconds; // 各相交根递归切削任务耗时的累计值。
    double commitMilliseconds; // 串行提交相交根结果和完整删除根的墙钟耗时。
};

#endif

// 将一个连续几何实例直接从体素体中减去。
//
// 几何实例会转换到object的体素局部空间。
// 算法只访问几何包围盒范围内已有的工件根树，不构造工具VoxelShape。
class ShapeCutAlgorithm
{
public:
    // 原地执行连续几何差集，返回object是否发生实际变化。
    static bool apply(VoxelShape& object, const Geometry::ShapeInstance& tool, VoxelChangeSet* changes = nullptr);

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

    // 原地执行连续几何差集并记录内部统计，返回object是否发生实际变化。
    static bool apply(VoxelShape& object, const Geometry::ShapeInstance& tool,
                      VoxelChangeSet* changes, ShapeCutStatistics& statistics);

#endif

private:
    ShapeCutAlgorithm() = delete;
};

}
}
}

#endif // MYVOXEL_OPERATION_ALGORITHM_SHAPECUTALGORITHM_H