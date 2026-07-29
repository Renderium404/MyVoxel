#include "BooleanAlgorithmConfig.h"

#if MYVOXEL_BOOLEAN_STATISTICS_ENABLED

#include "BooleanCutAlgorithm.h"

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "BooleanAlgorithmCommon.h"
#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Core/Query/VoxelForestQuery.h"
#include "MyVoxel/Core/Tree/VoxelForestConstAccessor.h"
#include "MyVoxel/Core/Tree/VoxelTreeEditor.h"
#include "MyVoxel/Operation/BooleanOperation.h"
#include "VoxelMaskForestCutAlgorithm.h"
namespace
{

using Clock = std::chrono::steady_clock;

const double HalfScale = 0.5; // 体素半边长为完整边长的一半。

// 保存工件局部坐标到体素刀具局部坐标的固定变换数据。
struct CellTransformContext
{
    MyMath::Matrix4 objectToTool; // 工件局部坐标到刀具局部坐标的变换。
};

// 返回两个时间点之间的毫秒数。
double elapsedMilliseconds(const Clock::time_point& start, const Clock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

// 创建工件局部坐标到体素刀具局部坐标的固定变换。
CellTransformContext createCellTransformContext(const MyVoxel::VoxelShape& object, const MyVoxel::VoxelShape& tool)
{
    assert(object.transform().isRigidTransform());
    assert(tool.transform().isRigidTransform());

    MyMath::Matrix4 worldToTool;
    const bool inverted = tool.transform().inverted(worldToTool);

    assert(inverted);

    CellTransformContext context;
    context.objectToTool = worldToTool * object.transform();
    return context;
}

// 返回工件体素映射到刀具局部坐标后的保守轴对齐包围盒。
MyVoxel::Bounds3 transformedCellBounds(const MyVoxel::VoxelShape& object, const CellTransformContext& context, const MyVoxel::VoxelCellAddress& address)
{
    return object.grid().cellBounds(address).transformed(context.objectToTool);
}

// 将连续坐标转换为指定网格索引。
MyVoxel::VoxelIndex coordinateIndex(double coordinate, double origin, double edgeLength)
{
    const double indexValue = std::floor((coordinate - origin) / edgeLength);
    const double minimumValue = static_cast<double>((std::numeric_limits<MyVoxel::VoxelIndex>::min)());
    const double maximumValue = static_cast<double>((std::numeric_limits<MyVoxel::VoxelIndex>::max)());

    assert(indexValue >= minimumValue && indexValue <= maximumValue);
    return static_cast<MyVoxel::VoxelIndex>(indexValue);
}

// 返回指定点在VoxelGrid目标层级对应的体素地址。
MyVoxel::VoxelCellAddress gridAddressAtPoint(const MyVoxel::VoxelGrid& grid, const MyMath::Vector3& point, MyVoxel::VoxelLevel level)
{
    const MyVoxel::VoxelCellAddress zeroAddress(MyVoxel::VoxelCellIndex(0, 0, 0), level);
    const MyVoxel::Bounds3 zeroBounds = grid.cellBounds(zeroAddress);
    const MyMath::Vector3& origin = zeroBounds.minimum();
    const double edgeLength = zeroBounds.maximum().x() - zeroBounds.minimum().x();

    return MyVoxel::VoxelCellAddress(
        MyVoxel::VoxelCellIndex(
            coordinateIndex(point.x(), origin.x(), edgeLength),
            coordinateIndex(point.y(), origin.y(), edgeLength),
            coordinateIndex(point.z(), origin.z(), edgeLength)),
        level);
}

// 累加一次体素森林区域查询统计。
void accumulateQueryStatistics(MyVoxel::Operation::BooleanOperationStatistics& target, const MyVoxel::VoxelForestQueryStatistics& source)
{
    target.queryRootCandidateCount += source.rootCandidateCount;
    target.queryExistingRootCount += source.existingRootCount;
    target.queryNodeBoundsTestCount += source.nodeBoundsTestCount;
    target.queryVisitedNodeCount += source.visitedNodeCount;
}

// 返回体素刀具区域分类并记录查询统计。
MyVoxel::Operation::Algorithm::CellRelation classifyObjectCellWithVoxelToolStatistics(const MyVoxel::VoxelShape& object, const CellTransformContext& context, const MyVoxel::VoxelForestQuery& toolQuery, const MyVoxel::VoxelCellAddress& address, MyVoxel::Operation::BooleanOperationStatistics& statistics)
{
    ++statistics.classifiedCellCount;

    MyVoxel::VoxelForestQueryStatistics queryStatistics;
    const MyVoxel::VoxelRegionRelation relation = toolQuery.classify(transformedCellBounds(object, context, address), queryStatistics);

    accumulateQueryStatistics(statistics, queryStatistics);

    switch (relation)
    {
    case MyVoxel::VoxelRegionRelation::Outside:
        return MyVoxel::Operation::Algorithm::CellRelation::Outside;

    case MyVoxel::VoxelRegionRelation::Intersecting:
        return MyVoxel::Operation::Algorithm::CellRelation::Intersecting;

    case MyVoxel::VoxelRegionRelation::Inside:
        return MyVoxel::Operation::Algorithm::CellRelation::Inside;
    }

    assert(false);
    return MyVoxel::Operation::Algorithm::CellRelation::Outside;
}

// 检查工件体素中心是否位于体素刀具材料中并记录点查询。
bool voxelToolContainsObjectCellCenterStatistics(const MyVoxel::VoxelShape& object, const MyVoxel::VoxelShape& tool, const CellTransformContext& context, MyVoxel::VoxelForestConstAccessor& toolAccessor, const MyVoxel::VoxelCellAddress& address, MyVoxel::Operation::BooleanOperationStatistics& statistics)
{
    ++statistics.pointQueryCount;

    const MyMath::Vector3 toolPoint = context.objectToTool.transformPoint(MyVoxel::Operation::Algorithm::cellCenter(object, address));
    const MyVoxel::VoxelCellAddress toolAddress = gridAddressAtPoint(tool.grid(), toolPoint, tool.grid().maximumLevel());

    return toolAccessor.state(toolAddress) == MyVoxel::VoxelState::Material;
}

// 递归执行带统计的体素刀具布尔减。
MyVoxel::Operation::Algorithm::CutCellResult cutMaterialCellWithVoxelToolStatistics(MyVoxel::VoxelTreeEditor editor, const MyVoxel::VoxelShape& object, const MyVoxel::VoxelShape& tool, const CellTransformContext& context, const MyVoxel::VoxelForestQuery& toolQuery, MyVoxel::VoxelForestConstAccessor& toolAccessor, const MyVoxel::VoxelCellAddress& address, MyVoxel::VoxelLevel targetLevel, MyVoxel::Operation::BooleanOperationStatistics& statistics)
{
    ++statistics.visitedCellCount;

    if (address.level == targetLevel)
    {
        ++statistics.centerSampleCount;

        if (voxelToolContainsObjectCellCenterStatistics(object, tool, context, toolAccessor, address, statistics))
        {
            ++statistics.centerRemovedCellCount;
            editor.setState(MyVoxel::VoxelState::Empty);
            return MyVoxel::Operation::Algorithm::CutCellResult(MyVoxel::VoxelState::Empty, true);
        }

        return MyVoxel::Operation::Algorithm::CutCellResult(MyVoxel::VoxelState::Material, false);
    }

    const MyVoxel::Operation::Algorithm::CellRelation relation = classifyObjectCellWithVoxelToolStatistics(object, context, toolQuery, address, statistics);

    if (relation == MyVoxel::Operation::Algorithm::CellRelation::Outside)
    {
        ++statistics.outsideCellCount;
        return MyVoxel::Operation::Algorithm::CutCellResult(MyVoxel::VoxelState::Material, false);
    }

    if (relation == MyVoxel::Operation::Algorithm::CellRelation::Inside)
    {
        ++statistics.insideCellCount;
        ++statistics.removedBranchCount;
        editor.setState(MyVoxel::VoxelState::Empty);
        return MyVoxel::Operation::Algorithm::CutCellResult(MyVoxel::VoxelState::Empty, true);
    }

    ++statistics.intersectingCellCount;

    assert(address.level < targetLevel);

    editor.split();

    bool changed = false;

    for (int cornerIndex = 0; cornerIndex < MyVoxel::VoxelCornerCount; ++cornerIndex)
    {
        const MyVoxel::VoxelCorner corner = static_cast<MyVoxel::VoxelCorner>(cornerIndex);
        const MyVoxel::VoxelCellAddress childAddress = MyVoxel::childCellAddress(address, corner);
        const MyVoxel::Operation::Algorithm::CutCellResult childResult = cutMaterialCellWithVoxelToolStatistics(editor.child(corner), object, tool, context, toolQuery, toolAccessor, childAddress, targetLevel, statistics);

        changed = childResult.changed || changed;
    }

    ++statistics.mergeAttemptCount;

    const MyVoxel::VoxelState mergedState = editor.merge();

    if (mergedState != MyVoxel::VoxelState::Subdivided)
    {
        ++statistics.mergeSuccessCount;
    }

    return MyVoxel::Operation::Algorithm::CutCellResult(mergedState, changed);
}
// 将对齐体素森林掩码切削统计写入公共布尔运算统计。
void assignAlignedMaskStatistics(
    MyVoxel::Operation::BooleanOperationStatistics& target,
    const MyVoxel::Operation::Algorithm::VoxelMaskForestCutStatistics& source)
{
    const MyVoxel::Operation::Algorithm::VoxelMaskCutStatistics& treeStatistics = source.treeStatistics;

    target.alignedMaskCutCount = 1;
    target.alignedMaskRootCandidateCount = source.rootCandidateCount;
    target.alignedMaskIntersectingRootCount = source.intersectingRootCount;
    target.alignedMaskOperationCount = treeStatistics.maskOperationCount;
    target.alignedMaskChangedCount = treeStatistics.maskChangedCount;

    target.inputMaterialCellCount = source.existingObjectRootCount;
    target.visitedCellCount = treeStatistics.visitedCellCount;
    target.emptySkippedCellCount = treeStatistics.emptyObjectSkipCount + treeStatistics.emptyToolSkipCount;

    target.outsideCellCount = treeStatistics.emptyToolSkipCount;
    target.insideCellCount = treeStatistics.materialToolRemoveCount;
    target.intersectingCellCount = treeStatistics.recursiveNodeCount + treeStatistics.maskOperationCount;
    target.classifiedCellCount = target.outsideCellCount + target.insideCellCount + target.intersectingCellCount;

    target.removedBranchCount =
        treeStatistics.materialToolRemoveCount +
        source.directRootEraseCount;

    target.mergeAttemptCount = treeStatistics.mergeAttemptCount;
    target.mergeSuccessCount = treeStatistics.mergeSuccessCount;

    target.preparedRootTreeCount = source.detachedRootCount;
    target.removedRootTreeCount =
        source.directRootEraseCount +
        source.emptiedRootEraseCount;
}
}

namespace MyVoxel
{
namespace Operation
{
namespace Algorithm
{

VoxelShape cutWithVoxelToolStatistics(const VoxelShape& object, const VoxelShape& tool, VoxelChangeSet* changes, BooleanOperationStatistics& statistics)
{
    statistics.reset();

    if (changes)
    {
        changes->clear();
    }

    const Clock::time_point totalStart = Clock::now();

    assert(object.grid().isValid());
    assert(tool.grid().isValid());
    assert(object.transform().isRigidTransform());
    assert(tool.transform().isRigidTransform());
    if (object.isEmpty() || tool.isEmpty())
    {
        statistics.totalMilliseconds = elapsedMilliseconds(totalStart, Clock::now());
        return object;
    }

    const Clock::time_point compatibilityStart = Clock::now();
    const bool useAlignedMaskCut = canUseAlignedVoxelMaskCut(object, tool);
    const Clock::time_point compatibilityEnd = Clock::now();

    statistics.toolIndexMilliseconds =
        elapsedMilliseconds(
            compatibilityStart,
            compatibilityEnd);

    if (useAlignedMaskCut)
    {
        VoxelShape result = object;

        const Clock::time_point detachStart = Clock::now();
        VoxelForest& resultForest = result.editForest();
        const Clock::time_point detachEnd = Clock::now();

        VoxelMaskForestCutStatistics maskStatistics;

        const Clock::time_point cuttingStart = Clock::now();

        const bool changed =
            cutAlignedVoxelForest(
                resultForest,
                tool.forest(),
                changes,
                &maskStatistics);

        const Clock::time_point cuttingEnd = Clock::now();

        statistics.detachMilliseconds =
            elapsedMilliseconds(
                detachStart,
                detachEnd);

        statistics.cuttingMilliseconds =
            elapsedMilliseconds(
                cuttingStart,
                cuttingEnd);

        assignAlignedMaskStatistics(
            statistics,
            maskStatistics);

        statistics.totalMilliseconds =
            elapsedMilliseconds(
                totalStart,
                Clock::now());

        assert(statistics.classifiedCellCount ==
               statistics.outsideCellCount +
               statistics.insideCellCount +
               statistics.intersectingCellCount);

        assert(statistics.pointQueryCount == statistics.centerSampleCount);
        assert(statistics.pointQueryCount ==
               statistics.accessorRootCacheHitCount +
               statistics.accessorRootCacheMissCount);

        return changed ? result : object;
    }
    const Clock::time_point toolIndexStart = Clock::now();
    const VoxelForestQuery toolQuery(tool.forest(), tool.grid());
    VoxelForestConstAccessor toolAccessor(tool.forest());
    const CellTransformContext context = createCellTransformContext(object, tool);
    const bool toolEmpty = tool.isEmpty();
    const Clock::time_point toolIndexEnd = Clock::now();

    statistics.toolIndexMilliseconds = elapsedMilliseconds(toolIndexStart, toolIndexEnd);

    if (toolEmpty)
    {
        statistics.totalMilliseconds = elapsedMilliseconds(totalStart, Clock::now());
        return object;
    }

    const Clock::time_point materialCollectionStart = Clock::now();
    const std::vector<VoxelCellAddress> materialCells = collectMaterialCells(object);
    const Clock::time_point materialCollectionEnd = Clock::now();

    statistics.materialCollectionMilliseconds = elapsedMilliseconds(materialCollectionStart, materialCollectionEnd);
    statistics.inputMaterialCellCount = static_cast<std::uint64_t>(materialCells.size());

    if (materialCells.empty())
    {
        statistics.totalMilliseconds = elapsedMilliseconds(totalStart, Clock::now());
        return object;
    }

    VoxelShape result = object;

    const Clock::time_point detachStart = Clock::now();
    VoxelForest& resultForest = result.editForest();
    const Clock::time_point detachEnd = Clock::now();

    statistics.detachMilliseconds = elapsedMilliseconds(detachStart, detachEnd);

    const VoxelLevel targetLevel = object.grid().maximumLevel();
    const Clock::time_point cuttingStart = Clock::now();

    for (std::size_t materialIndex = 0; materialIndex < materialCells.size(); ++materialIndex)
    {
        const VoxelCellAddress& address = materialCells[materialIndex];

        assert(address.level <= targetLevel);

        VoxelTreeEditor editor(resultForest, address);
        const CutCellResult cellResult = cutMaterialCellWithVoxelToolStatistics(editor, object, tool, context, toolQuery, toolAccessor, address, targetLevel, statistics);

        if (cellResult.changed && changes)
        {
            changes->addModifiedRoot(rootCellIndex(address));
        }

        if (cellResult.state == VoxelState::Empty)
        {
            resultForest.pruneEmptyBranch(address);
        }
    }

    const Clock::time_point cuttingEnd = Clock::now();
    const VoxelForestConstAccessorStatistics& accessorStatistics = toolAccessor.statistics();

    statistics.cuttingMilliseconds = elapsedMilliseconds(cuttingStart, cuttingEnd);
    statistics.accessorRootCacheHitCount = accessorStatistics.rootCacheHitCount;
    statistics.accessorRootCacheMissCount = accessorStatistics.rootCacheMissCount;
    statistics.accessorPathReuseCount = accessorStatistics.pathReuseCount;
    statistics.accessorReusedPathLevelCount = accessorStatistics.reusedPathLevelCount;
    statistics.accessorNodeVisitCount = accessorStatistics.nodeVisitCount;
    statistics.totalMilliseconds = elapsedMilliseconds(totalStart, Clock::now());

    assert(statistics.classifiedCellCount == statistics.outsideCellCount + statistics.insideCellCount + statistics.intersectingCellCount);
    assert(statistics.pointQueryCount == statistics.centerSampleCount);
    assert(statistics.pointQueryCount == statistics.accessorRootCacheHitCount + statistics.accessorRootCacheMissCount);

    return result;
}

}
}
}

#endif