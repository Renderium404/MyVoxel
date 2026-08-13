#ifndef MYVOXEL_OPERATION_ALGORITHM_SHAPECUTALGORITHM_H
#define MYVOXEL_OPERATION_ALGORITHM_SHAPECUTALGORITHM_H

#include <cstdint>

#include "MyVoxel/Core/Change/VoxelChangeSet.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Instance/Shape.h"

namespace MyVoxel
{
namespace Operation
{
namespace Algorithm
{

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

// 保存Instance::Shape连续体直接执行TSDF差集切削时的内部统计。
struct ShapeCutStatistics
{
    ShapeCutStatistics();

    // 清空全部统计数据。
    void reset();
    // 累加另一次连续Shape切削统计。
    void accumulate(const ShapeCutStatistics& other);

    std::uint64_t broadPhaseRootCandidateCount; // Shape查询包围盒向外扩展B后覆盖范围内的已有工件Root数量。
    std::uint64_t processedRootCount; // 实际建立独立切削任务的工件Root数量。
    std::uint64_t changedRootCount; // TSDF实际发生变化的工件Root数量。
    std::uint64_t removedRootCount; // 最终完全退化为缺失Root对应+B背景并被删除的Root数量。

    std::uint64_t rootBoundsConstructionCount; // 构造第0层Root中心和半尺寸的次数。
    std::uint64_t scalarFastClassificationCount; // 使用工具中心SDF和Cell半对角线执行区间判定的次数。
    std::uint64_t octantBatchClassificationCount; // 保留旧统计槽；当前TSDF切削不再使用二值Octant分类，因此始终为0。
    std::uint64_t derivedChildBoundsCount; // 由父Cell直接推导子Cell中心和半尺寸的次数。

    std::uint64_t visitedCellCount; // 实际访问的逻辑Cell数量。
    std::uint64_t emptySkippedCellCount; // 工件当前终止Value已经为+B而直接跳过的Cell数量。
    std::uint64_t classifiedCellCount; // 完成工具SDF区间判定的Cell数量。
    std::uint64_t outsideCellCount; // 工具在整个Cell上至少位于+B之外、结果可证明不变的Cell数量。
    std::uint64_t insideCellCount; // 工具在整个Cell上至少位于-B内部、结果可直接写为+B的Cell数量。
    std::uint64_t intersectingCellCount; // 工具TSDF窄带覆盖、必须继续细分或显式采样的Cell数量。

    std::uint64_t centerSampleCount; // 执行工具signed-distance中心采样的次数。
    std::uint64_t centerRemovedCellCount; // 最高层样本从材料侧变为空侧的数量。
    std::uint64_t removedBranchCount; // 工具深内部直接将已细分工件分支写为+B的次数。

    std::uint64_t splitCount; // 为保存局部差集结果而无损细分终止Tile的次数。
    std::uint64_t maskLeafBuildCount; // 直接生成64个最高层TSDF差集样本的次数。
    std::uint64_t maskLeafOperationCount; // 64样本LeafBlock实际发生Value变化的次数。
    std::uint64_t maskLeafRemovedCellCount; // LeafBlock路径中从材料侧变为空侧的最高层样本数量。

    std::uint64_t mergeAttemptCount; // Root切削完成后执行Value-aware prune的次数。
    std::uint64_t mergeSuccessCount; // Value-aware prune实际改变压缩结构的Root数量。

    double rootClassificationMilliseconds; // 收集Shape扩展窄带范围内已有Root的墙钟耗时。
    double rootPreparationMilliseconds; // 建立独立Root结果树和脏区任务的墙钟耗时。
    double intersectingRootExecutionMilliseconds; // 并行执行全部候选Root TSDF差集的墙钟耗时。
    double intersectingRootCpuMilliseconds; // 各Root独立TSDF差集任务耗时累计值。
    double commitMilliseconds; // 串行提交结果Root和删除+B背景Root的墙钟耗时。
};

#endif

// 将当前Instance::Shape连续体直接从VoxelShape TSDF中减去。
//
// 工具实例空间放置由Shape自身的Instance_Object语义提供，查询空间使用object.transform()。
// 差集统一使用dResult=max(dObject,-dTool)，结果保持object固定[-B,+B]截断范围。
// ShapeQuery必须在object体素局部空间中支持精确signed distance；算法不会构造工具VoxelShape。
class ShapeCutAlgorithm
{
public:
    // 原地执行VoxelShape-Shape连续TSDF差集，返回object距离场是否发生实际变化。
    static bool apply(VoxelShape& object, const Shape& tool, VoxelChangeSet* changes = nullptr);

#ifdef MYVOXEL_ENABLE_OPERATION_STATISTICS

    // 原地执行VoxelShape-Shape连续TSDF差集并输出内部统计。
    static bool apply(VoxelShape& object, const Shape& tool, VoxelChangeSet* changes, ShapeCutStatistics& statistics);

#endif

private:
    ShapeCutAlgorithm() = delete;
};

}
}
}

#endif // MYVOXEL_OPERATION_ALGORITHM_SHAPECUTALGORITHM_H