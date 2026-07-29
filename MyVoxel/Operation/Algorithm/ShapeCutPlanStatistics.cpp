#include "BooleanAlgorithmConfig.h"

#if MYVOXEL_BOOLEAN_STATISTICS_ENABLED

#include "ShapeCutPlanStatistics.h"

#include <atomic>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <thread>
#include <utility>

#include "BooleanAlgorithmCommon.h"
#include "MyVoxel/Core/Tree/VoxelTreeConstCursor.h"

namespace
{

const std::uint32_t InvalidCutPlanGroupIndex = (std::numeric_limits<std::uint32_t>::max)(); // 无效切削计划子节点组索引。

}

namespace MyVoxel
{
namespace Operation
{
namespace Algorithm
{

// 检查八个子计划是否全部使用指定动作。
static bool allChildPlansUseAction(const RootCutPlan& plan, std::uint32_t groupIndex, CutPlanAction action)
{
    assert(static_cast<std::size_t>(groupIndex) < plan.nodeGroups.size());

    const CutPlanNodeGroup& group = plan.nodeGroups[static_cast<std::size_t>(groupIndex)];

    for (std::size_t childIndex = 0; childIndex < group.children.size(); ++childIndex)
    {
        if (group.children[childIndex].action != action)
        {
            return false;
        }
    }

    return true;
}

// 检查八个子计划执行后是否全部得到指定节点状态。
static bool allChildPlansResultInState(const RootCutPlan& plan, std::uint32_t groupIndex, VoxelState state)
{
    assert(static_cast<std::size_t>(groupIndex) < plan.nodeGroups.size());

    const CutPlanNodeGroup& group = plan.nodeGroups[static_cast<std::size_t>(groupIndex)];

    for (std::size_t childIndex = 0; childIndex < group.children.size(); ++childIndex)
    {
        if (group.children[childIndex].resultState != state)
        {
            return false;
        }
    }

    return true;
}

// 返回连续刀具区域分类并记录统计。
static CellRelation classifyObjectCellWithShapeToolStatistics(const Geometry::ShapeInstance& tool, const ShapeToolContext& context, const VoxelCellAddress& address, const MyMath::Vector3& centerToolSpace, BooleanOperationStatistics& statistics)
{
    ++statistics.classifiedCellCount;

    const Geometry::ShapeRelation relation = tool.shape().classifyLocalBounds(shapeToolCellBounds(context, address.level, centerToolSpace));

    switch (relation)
    {
    case Geometry::ShapeRelation::Outside:
        return CellRelation::Outside;

    case Geometry::ShapeRelation::Intersecting:
        return CellRelation::Intersecting;

    case Geometry::ShapeRelation::Inside:
        return CellRelation::Inside;
    }

    assert(false);
    return CellRelation::Outside;
}

// 检查最高层体素中心是否位于连续刀具内部并记录点查询。
static bool shapeToolContainsObjectCellCenterStatistics(const Geometry::ShapeInstance& tool, const MyMath::Vector3& centerToolSpace, BooleanOperationStatistics& statistics)
{
    ++statistics.pointQueryCount;
    return tool.shape().containsLocalPoint(centerToolSpace);
}

// 递归生成当前节点的带统计切削计划。
static CutPlanNode buildShapeCutPlanStatistics(const VoxelTreeConstCursor& cursor, const Geometry::ShapeInstance& tool, const ShapeToolContext& context, const VoxelCellAddress& address, const MyMath::Vector3& centerToolSpace, VoxelLevel targetLevel, RootCutPlan& plan, BooleanOperationStatistics& statistics)
{
    ++statistics.visitedCellCount;
    ++statistics.cutPlanNodeCount;

    const VoxelState currentState = cursor.state();

    if (currentState == VoxelState::Empty)
    {
        ++statistics.emptySkippedCellCount;
        return CutPlanNode(CutPlanAction::Keep, VoxelState::Empty, InvalidCutPlanGroupIndex);
    }

    if (address.level == targetLevel)
    {
        assert(currentState == VoxelState::Material);

        ++statistics.centerSampleCount;

        if (shapeToolContainsObjectCellCenterStatistics(tool, centerToolSpace, statistics))
        {
            ++statistics.centerRemovedCellCount;
            return CutPlanNode(CutPlanAction::Remove, VoxelState::Empty, InvalidCutPlanGroupIndex);
        }

        return CutPlanNode(CutPlanAction::Keep, VoxelState::Material, InvalidCutPlanGroupIndex);
    }

    const CellRelation relation = classifyObjectCellWithShapeToolStatistics(tool, context, address, centerToolSpace, statistics);

    if (relation == CellRelation::Outside)
    {
        ++statistics.outsideCellCount;
        return CutPlanNode(CutPlanAction::Keep, currentState, InvalidCutPlanGroupIndex);
    }

    if (relation == CellRelation::Inside)
    {
        ++statistics.insideCellCount;
        ++statistics.removedBranchCount;
        return CutPlanNode(CutPlanAction::Remove, VoxelState::Empty, InvalidCutPlanGroupIndex);
    }

    ++statistics.intersectingCellCount;

    assert(address.level < targetLevel);

    const std::size_t originalGroupCount = plan.nodeGroups.size();
    const std::uint32_t groupIndex = plan.allocateNodeGroup();
    const ShapeToolLevelContext& levelContext = context.levels[static_cast<std::size_t>(address.level)];

    for (int cornerIndex = 0; cornerIndex < VoxelCornerCount; ++cornerIndex)
    {
        const VoxelCorner corner = static_cast<VoxelCorner>(cornerIndex);
        const VoxelCellAddress childAddress = childCellAddress(address, corner);
        const MyMath::Vector3 childCenterToolSpace = centerToolSpace + levelContext.childCenterOffsets[cornerIndex];
        const VoxelTreeConstCursor childCursor = cursor.child(corner);
        const CutPlanNode childPlan = buildShapeCutPlanStatistics(childCursor, tool, context, childAddress, childCenterToolSpace, targetLevel, plan, statistics);

        plan.nodeGroups[static_cast<std::size_t>(groupIndex)].children[static_cast<std::size_t>(cornerIndex)] = childPlan;
    }

    if (allChildPlansUseAction(plan, groupIndex, CutPlanAction::Keep))
    {
        plan.nodeGroups.resize(originalGroupCount);
        return CutPlanNode(CutPlanAction::Keep, currentState, InvalidCutPlanGroupIndex);
    }

    if (allChildPlansResultInState(plan, groupIndex, VoxelState::Empty))
    {
        plan.nodeGroups.resize(originalGroupCount);
        return CutPlanNode(CutPlanAction::Remove, VoxelState::Empty, InvalidCutPlanGroupIndex);
    }

    return CutPlanNode(CutPlanAction::Subdivide, VoxelState::Subdivided, groupIndex);
}

void buildShapeCutPlansStatistics(const VoxelShape& object, const Geometry::ShapeInstance& tool, const ShapeToolContext& context, const std::vector<VoxelCellAddress>& rootCells, unsigned int workerCount, std::vector<RootCutPlan>& plans, std::vector<BooleanOperationStatistics>& localStatistics)
{
    assert(!rootCells.empty());
    assert(workerCount > 0);
    assert(plans.size() == rootCells.size());
    assert(localStatistics.size() == rootCells.size());

    std::atomic<std::size_t> nextRootIndex(0);

    const auto worker =
        [&]()
        {
            while (true)
            {
                const std::size_t rootIndex = nextRootIndex.fetch_add(1);

                if (rootIndex >= rootCells.size())
                {
                    return;
                }

                RootCutPlan currentPlan;
                BooleanOperationStatistics currentStatistics;
                const VoxelCellAddress& rootAddress = rootCells[rootIndex];
                const MyMath::Vector3 centerToolSpace = context.objectToTool.transformPoint(cellCenter(object, rootAddress));
                const VoxelTreeConstCursor cursor(object.forest(), rootAddress);

                currentPlan.root = buildShapeCutPlanStatistics(cursor, tool, context, rootAddress, centerToolSpace, object.grid().maximumLevel(), currentPlan, currentStatistics);
                currentStatistics.cutPlanGroupCount = static_cast<std::uint64_t>(currentPlan.nodeGroups.size());

                plans[rootIndex] = std::move(currentPlan);
                localStatistics[rootIndex] = currentStatistics;
            }
        };

    if (workerCount == 1)
    {
        worker();
        return;
    }

    std::vector<std::thread> workers;
    workers.reserve(static_cast<std::size_t>(workerCount - 1));

    for (unsigned int workerIndex = 1; workerIndex < workerCount; ++workerIndex)
    {
        workers.push_back(std::thread(worker));
    }

    worker();

    for (std::size_t workerIndex = 0; workerIndex < workers.size(); ++workerIndex)
    {
        workers[workerIndex].join();
    }
}

}
}
}

#endif