#ifndef MYVOXEL_OPERATION_BOOLEANOPERATION_H
#define MYVOXEL_OPERATION_BOOLEANOPERATION_H

#include <cstddef>
#include <cstdint>

#include "Algorithm/BooleanAlgorithmConfig.h"
#include "MyVoxel/Core/Change/VoxelChangeSet.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Geometry/ShapeInstance.h"

namespace MyVoxel
{
namespace Operation
{

// 控制连续Shape布尔切削的并行执行方式。
struct BooleanOperationOptions
{
    unsigned int workerCount = 0; // 工作线程数量，0表示根据硬件并发数量自动确定。
    std::size_t minimumParallelRootCount = 8; // 候选根节点少于8个时使用单线程。
};

#if MYVOXEL_BOOLEAN_STATISTICS_ENABLED

// 记录一次或多次体素布尔运算的耗时和节点处理数量。
struct BooleanOperationStatistics
{
    // 清空全部统计数据。
    void reset();

    // 累加另一次布尔运算的统计数据。
    void accumulate(const BooleanOperationStatistics& other);

    /// 耗时统计

    double totalMilliseconds = 0.0; // 布尔运算总耗时。
    double toolIndexMilliseconds = 0.0; // 建立刀具查询上下文、宽相范围和坐标变换的耗时。
    double materialCollectionMilliseconds = 0.0; // 收集工件递归入口节点的耗时。
    double detachMilliseconds = 0.0; // 写时复制工件森林根映射的耗时。
    double planningMilliseconds = 0.0; // 并行分析原始体素树并生成切削计划的耗时。
    double rootPreparationMilliseconds = 0.0; // 删除完整根树并分离待修改根树的耗时。
    double planApplicationMilliseconds = 0.0; // 并行应用切削计划的耗时。
    double cuttingMilliseconds = 0.0; // 计划生成、根树准备和计划应用的总耗时。

    /// 连续Shape并行统计

    std::uint64_t planningWorkerCount = 0; // 计划生成实际使用的线程数量。
    std::uint64_t planApplicationWorkerCount = 0; // 计划应用实际使用的线程数量。
    std::uint64_t cutPlanNodeCount = 0; // 切削计划分析和保存的节点数量。
    std::uint64_t cutPlanGroupCount = 0; // 切削计划保存的八子节点组数量。
    std::uint64_t preparedRootTreeCount = 0; // 完成分离并交给并行应用的根树数量。
    std::uint64_t removedRootTreeCount = 0; // 直接删除的完整根树数量。

    /// 连续Shape宽相统计

    std::uint64_t broadPhaseRootCandidateCount = 0; // 连续Shape包围盒覆盖的第0层候选根数量。
    std::uint64_t broadPhaseExistingRootCount = 0; // 宽相范围内实际存在的根数量。

    /// 工件递归统计

    std::uint64_t toolMaterialCellCount = 0; // 旧扁平刀具索引材料节点数量，当前固定为0。
    std::uint64_t inputMaterialCellCount = 0; // 进入递归的入口节点数量。
    std::uint64_t visitedCellCount = 0; // 递归访问的工件节点数量。
    std::uint64_t emptySkippedCellCount = 0; // 连续Shape计划中直接跳过的已有空节点数量。
    std::uint64_t classifiedCellCount = 0; // 执行区域分类的节点数量。
    std::uint64_t outsideCellCount = 0; // 被判定为完全外部的节点数量。
    std::uint64_t insideCellCount = 0; // 被判定为完全内部的节点数量。
    std::uint64_t intersectingCellCount = 0; // 被判定为边界相交的节点数量。
    std::uint64_t centerSampleCount = 0; // 在目标层级执行中心采样的节点数量。
    std::uint64_t pointQueryCount = 0; // 对刀具执行点占用查询的数量。
    std::uint64_t centerRemovedCellCount = 0; // 通过中心采样删除的节点数量。
    std::uint64_t removedBranchCount = 0; // 通过Inside判定整分支删除的数量。
    std::uint64_t mergeAttemptCount = 0; // 体素刀具路径尝试合并的节点数量。
    std::uint64_t mergeSuccessCount = 0; // 体素刀具路径成功合并的节点数量。

    /// 体素刀具区域查询统计

    std::uint64_t rootBucketVisitCount = 0; // 旧扁平索引根桶访问数量，当前固定为0。
    std::uint64_t materialBoxTestCount = 0; // 旧扁平索引包围盒测试数量，当前固定为0。
    std::uint64_t queryRootCandidateCount = 0; // 森林查询覆盖的第0层候选根数量。
    std::uint64_t queryExistingRootCount = 0; // 森林查询命中的实际根数量。
    std::uint64_t queryNodeBoundsTestCount = 0; // 森林查询执行的节点包围盒测试数量。
    std::uint64_t queryVisitedNodeCount = 0; // 森林查询实际访问的节点数量。

    /// 体素刀具点访问器统计

    std::uint64_t accessorRootCacheHitCount = 0; // 点查询命中最近根缓存的次数。
    std::uint64_t accessorRootCacheMissCount = 0; // 点查询未命中最近根缓存的次数。
    std::uint64_t accessorPathReuseCount = 0; // 点查询复用至少一级路径的次数。
    std::uint64_t accessorReusedPathLevelCount = 0; // 点查询累计复用的路径层级数量。
    std::uint64_t accessorNodeVisitCount = 0; // 点访问器实际检查节点状态的次数。

    /// 对齐体素掩码路径统计

    std::uint64_t alignedMaskCutCount = 0; // 实际选择完全对齐体素掩码快速路径的次数。
    std::uint64_t alignedMaskRootCandidateCount = 0; // 掩码路径遍历的工具根节点数量。
    std::uint64_t alignedMaskIntersectingRootCount = 0; // 掩码预检确认存在材料交集的根节点数量。
    std::uint64_t alignedMaskOperationCount = 0; // 直接执行64位材料掩码差集的次数。
    std::uint64_t alignedMaskChangedCount = 0; // 64位材料掩码差集实际改变对象的次数。
};

#endif

/// 体素刀具布尔减

VoxelShape cut(const VoxelShape& object, const VoxelShape& tool);
VoxelShape cut(const VoxelShape& object, const VoxelShape& tool, VoxelChangeSet& changes);

#if MYVOXEL_BOOLEAN_STATISTICS_ENABLED

VoxelShape cut(const VoxelShape& object, const VoxelShape& tool, BooleanOperationStatistics& statistics);
VoxelShape cut(const VoxelShape& object, const VoxelShape& tool, VoxelChangeSet& changes, BooleanOperationStatistics& statistics);

#endif

/// 连续刀具布尔减

VoxelShape cut(const VoxelShape& object, const Geometry::ShapeInstance& tool);
VoxelShape cut(const VoxelShape& object, const Geometry::ShapeInstance& tool, VoxelChangeSet& changes);
VoxelShape cut(const VoxelShape& object, const Geometry::ShapeInstance& tool, const BooleanOperationOptions& options);
VoxelShape cut(const VoxelShape& object, const Geometry::ShapeInstance& tool, const BooleanOperationOptions& options, VoxelChangeSet& changes);

#if MYVOXEL_BOOLEAN_STATISTICS_ENABLED

VoxelShape cut(const VoxelShape& object, const Geometry::ShapeInstance& tool, BooleanOperationStatistics& statistics);
VoxelShape cut(const VoxelShape& object, const Geometry::ShapeInstance& tool, VoxelChangeSet& changes, BooleanOperationStatistics& statistics);
VoxelShape cut(const VoxelShape& object, const Geometry::ShapeInstance& tool, const BooleanOperationOptions& options, BooleanOperationStatistics& statistics);
VoxelShape cut(const VoxelShape& object, const Geometry::ShapeInstance& tool, const BooleanOperationOptions& options, VoxelChangeSet& changes, BooleanOperationStatistics& statistics);

#endif

}
}

#endif // MYVOXEL_OPERATION_BOOLEANOPERATION_H