#ifndef MYVOXEL_BOOLEANOPERATION_H
#define MYVOXEL_BOOLEANOPERATION_H

#include <cstddef>
#include <cstdint>

#include "../Core/VoxelChangeSet.h"
#include "../Core/VoxelShape.h"
#include "../Shape/Shape.h"

namespace MyVoxel
{

// 控制连续Shape布尔切削的并行计划生成方式。
struct BooleanOperationOptions
{
    unsigned int workerCount = 0; // 工作线程数量，0表示根据硬件并发数量自动确定。
    std::size_t minimumParallelRootCount = 8; // 候选根节点少于8个时使用单线程，避免线程调度成本超过收益。
};

// 记录一次或多次体素布尔运算的耗时和节点处理数量。
struct BooleanOperationStatistics
{
    // 清空全部统计数据。
    void reset();

    // 累加另一次布尔运算的统计数据。
    void accumulate(const BooleanOperationStatistics& other);

    /// 耗时统计

    double totalMilliseconds = 0.0; // 布尔运算总耗时。
    double toolIndexMilliseconds = 0.0; // 建立右操作数查询上下文、宽相范围和坐标变换的耗时。
    double materialCollectionMilliseconds = 0.0; // 收集左操作数递归入口节点的耗时。
    double detachMilliseconds = 0.0; // 写时复制左操作数森林根映射的耗时。
    double planningMilliseconds = 0.0; // 并行分析原始体素树并生成切削计划的耗时。
    double rootPreparationMilliseconds = 0.0; // 主线程删除完整根树并分离待修改根树的耗时。
    double planApplicationMilliseconds = 0.0; // 并行应用切削计划并修改独立根树的耗时。
    double cuttingMilliseconds = 0.0; // 切削计划生成、根树准备和计划应用的总耗时。

    /// 连续Shape并行统计

    std::uint64_t planningWorkerCount = 0; // 切削计划生成实际使用的工作线程数量。
    std::uint64_t planApplicationWorkerCount = 0; // 计划应用阶段实际使用的工作线程数量。
    std::uint64_t cutPlanNodeCount = 0; // 切削计划分析和保存的节点数量。
    std::uint64_t cutPlanGroupCount = 0; // 切削计划中保存的八子节点组数量。
    std::uint64_t preparedRootTreeCount = 0; // 主线程完成分离并交给并行应用的根树数量。
    std::uint64_t removedRootTreeCount = 0; // 直接从结果森林删除的完整根树数量。

    /// 连续Shape宽相统计

    std::uint64_t broadPhaseRootCandidateCount = 0; // 连续Shape包围盒覆盖的第0层索引数量，溢出时饱和为最大值。
    std::uint64_t broadPhaseExistingRootCount = 0; // 宽相范围内实际存在并进入递归的第0层根节点数量。

    /// 左操作数递归统计

    std::uint64_t toolMaterialCellCount = 0; // 旧扁平索引提取的右操作数材料节点数量，当前查询路径固定为0。
    std::uint64_t inputMaterialCellCount = 0; // 左操作数进入递归的入口节点数量，体素路径为材料节点，连续Shape路径为根节点。
    std::uint64_t visitedCellCount = 0; // 递归访问的左操作数单元数量。
    std::uint64_t emptySkippedCellCount = 0; // 根节点递归中直接跳过的已有空单元数量。
    std::uint64_t classifiedCellCount = 0; // 执行区域分类的左操作数单元数量。
    std::uint64_t outsideCellCount = 0; // 被判定为完全不相交的单元数量。
    std::uint64_t insideCellCount = 0; // 被判定为完全位于右操作数内部的单元数量。
    std::uint64_t intersectingCellCount = 0; // 被判定为边界相交的单元数量。
    std::uint64_t centerSampleCount = 0; // 在目标层级执行中心采样的材料单元数量。
    std::uint64_t pointQueryCount = 0; // 对右操作数执行点占用查询的数量。
    std::uint64_t centerRemovedCellCount = 0; // 通过中心采样删除的单元数量。
    std::uint64_t removedBranchCount = 0; // 通过Inside判定整分支删除的数量。
    std::uint64_t mergeAttemptCount = 0; // 应用切削计划时尝试合并的节点数量。
    std::uint64_t mergeSuccessCount = 0; // 应用切削计划时实际成功合并的节点数量。

    /// 体素右操作数区域查询统计

    std::uint64_t rootBucketVisitCount = 0; // 旧扁平索引访问的根节点桶数量，当前查询路径固定为0。
    std::uint64_t materialBoxTestCount = 0; // 旧扁平索引执行的材料包围盒测试数量，当前查询路径固定为0。
    std::uint64_t queryRootCandidateCount = 0; // 森林查询覆盖的第0层候选根节点数量。
    std::uint64_t queryExistingRootCount = 0; // 森林查询命中的实际根节点数量。
    std::uint64_t queryNodeBoundsTestCount = 0; // 森林查询执行的节点包围盒测试数量。
    std::uint64_t queryVisitedNodeCount = 0; // 森林查询实际访问的相交节点数量。

    /// 体素右操作数点访问器统计

    std::uint64_t accessorRootCacheHitCount = 0; // 点查询命中最近根节点缓存的次数。
    std::uint64_t accessorRootCacheMissCount = 0; // 点查询未命中最近根节点缓存的次数。
    std::uint64_t accessorPathReuseCount = 0; // 点查询复用至少一级下级节点路径的次数。
    std::uint64_t accessorReusedPathLevelCount = 0; // 点查询累计复用的下级节点路径层级数。
    std::uint64_t accessorNodeVisitCount = 0; // 点查询访问器实际检查节点状态的次数。
};

/// 体素右操作数

// 返回从object中减去体素tool后的体素形体。
VoxelShape cut(const VoxelShape& object, const VoxelShape& tool);

// 返回从object中减去体素tool后的体素形体，并记录实际修改根节点。
VoxelShape cut(const VoxelShape& object, const VoxelShape& tool, VoxelChangeSet& changes);

// 返回从object中减去体素tool后的体素形体，并记录运算统计数据。
VoxelShape cut(const VoxelShape& object, const VoxelShape& tool, BooleanOperationStatistics& statistics);

// 返回从object中减去体素tool后的体素形体，并记录实际修改根节点和运算统计数据。
VoxelShape cut(const VoxelShape& object, const VoxelShape& tool, VoxelChangeSet& changes, BooleanOperationStatistics& statistics);

/// 连续右操作数

// 使用默认并行选项返回从object中减去连续tool后的体素形体。
VoxelShape cut(const VoxelShape& object, const Shape& tool);

// 使用默认并行选项返回从object中减去连续tool后的体素形体，并记录实际修改根节点。
VoxelShape cut(const VoxelShape& object, const Shape& tool, VoxelChangeSet& changes);

// 使用默认并行选项返回从object中减去连续tool后的体素形体，并记录运算统计数据。
VoxelShape cut(const VoxelShape& object, const Shape& tool, BooleanOperationStatistics& statistics);

// 使用默认并行选项返回从object中减去连续tool后的体素形体，并记录实际修改根节点和运算统计数据。
VoxelShape cut(const VoxelShape& object, const Shape& tool, VoxelChangeSet& changes, BooleanOperationStatistics& statistics);

// 使用指定并行选项返回从object中减去连续tool后的体素形体。
VoxelShape cut(const VoxelShape& object, const Shape& tool, const BooleanOperationOptions& options);

// 使用指定并行选项返回从object中减去连续tool后的体素形体，并记录实际修改根节点。
VoxelShape cut(const VoxelShape& object, const Shape& tool, const BooleanOperationOptions& options, VoxelChangeSet& changes);

// 使用指定并行选项返回从object中减去连续tool后的体素形体，并记录运算统计数据。
VoxelShape cut(const VoxelShape& object, const Shape& tool, const BooleanOperationOptions& options, BooleanOperationStatistics& statistics);

// 使用指定并行选项返回从object中减去连续tool后的体素形体，并记录实际修改根节点和运算统计数据。
VoxelShape cut(const VoxelShape& object, const Shape& tool, const BooleanOperationOptions& options, VoxelChangeSet& changes, BooleanOperationStatistics& statistics);

}

#endif // MYVOXEL_BOOLEANOPERATION_H