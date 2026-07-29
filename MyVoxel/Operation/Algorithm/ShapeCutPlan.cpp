#include "ShapeCutPlan.h"

#include <atomic>
#include <cassert>
#include <limits>
#include <thread>
#include <utility>

#include "BooleanAlgorithmCommon.h"
#include "MyVoxel/Core/Tree/VoxelTreeConstCursor.h"

namespace
{

const unsigned int MaximumAutomaticWorkerCount = 8; // 自动并行最多使用8个线程，保留原版实测上限。
const std::uint32_t InvalidCutPlanGroupIndex = (std::numeric_limits<std::uint32_t>::max)(); // 无效切削计划子节点组索引。

}

namespace MyVoxel
{
namespace Operation
{
namespace Algorithm
{

CutPlanNode::CutPlanNode()
    : action(CutPlanAction::Keep)
    , resultState(VoxelState::Empty)
    , childGroupIndex(InvalidCutPlanGroupIndex)
{
}

CutPlanNode::CutPlanNode(CutPlanAction actionValue, VoxelState resultStateValue, std::uint32_t childGroupIndexValue)
    : action(actionValue)
    , resultState(resultStateValue)
    , childGroupIndex(childGroupIndexValue)
{
}

RootCutPlan::RootCutPlan()
{
}

std::uint32_t RootCutPlan::allocateNodeGroup()
{
    assert(nodeGroups.size() < static_cast<std::size_t>(InvalidCutPlanGroupIndex));

    nodeGroups.push_back(CutPlanNodeGroup());
    return static_cast<std::uint32_t>(nodeGroups.size() - 1);
}

RootPlanApplication::RootPlanApplication(VoxelForest& forest, const VoxelCellAddress& rootAddress, const RootCutPlan& planValue)
    : editor(forest, rootAddress)
    , plan(&planValue)
{
    assert(rootAddress.level == BaseVoxelLevel);
}

// 返回左操作数体素与连续刀具材料区域之间的保守关系。
static CellRelation classifyObjectCellWithShapeTool(const Geometry::ShapeInstance& tool,
                                                    const ShapeToolContext& context,
                                                    const VoxelCellAddress& address,
                                                    const MyMath::Vector3& centerToolSpace)
{
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

// 递归读取原始体素树并生成当前节点的连续Shape切削计划。
static CutPlanNode buildShapeCutPlan(const VoxelTreeConstCursor& cursor,
                                     const Geometry::ShapeInstance& tool,
                                     const ShapeToolContext& context,
                                     const VoxelCellAddress& address,
                                     const MyMath::Vector3& centerToolSpace,
                                     VoxelLevel targetLevel,
                                     RootCutPlan& plan)
{
    const VoxelState currentState = cursor.state();

    if (currentState == VoxelState::Empty)
    {
        return CutPlanNode(CutPlanAction::Keep, VoxelState::Empty, InvalidCutPlanGroupIndex);
    }

    if (address.level == targetLevel)
    {
        assert(currentState == VoxelState::Material);

        return tool.shape().containsLocalPoint(centerToolSpace) ?
            CutPlanNode(CutPlanAction::Remove, VoxelState::Empty, InvalidCutPlanGroupIndex) :
            CutPlanNode(CutPlanAction::Keep, VoxelState::Material, InvalidCutPlanGroupIndex);
    }

    const CellRelation relation = classifyObjectCellWithShapeTool(tool, context, address, centerToolSpace);

    if (relation == CellRelation::Outside)
    {
        return CutPlanNode(CutPlanAction::Keep, currentState, InvalidCutPlanGroupIndex);
    }

    if (relation == CellRelation::Inside)
    {
        return CutPlanNode(CutPlanAction::Remove, VoxelState::Empty, InvalidCutPlanGroupIndex);
    }

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
        const CutPlanNode childPlan = buildShapeCutPlan(childCursor, tool, context, childAddress, childCenterToolSpace, targetLevel, plan);

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

unsigned int resolveParallelWorkerCount(const BooleanOperationOptions& options, std::size_t taskCount)
{
    if (taskCount == 0)
    {
        return 0;
    }

    if (taskCount < options.minimumParallelRootCount)
    {
        return 1;
    }

    unsigned int workerCount = options.workerCount;

    if (workerCount == 0)
    {
        workerCount = std::thread::hardware_concurrency();

        if (workerCount == 0)
        {
            workerCount = 1;
        }

        if (workerCount > MaximumAutomaticWorkerCount)
        {
            workerCount = MaximumAutomaticWorkerCount;
        }
    }

    if (static_cast<std::size_t>(workerCount) > taskCount)
    {
        workerCount = static_cast<unsigned int>(taskCount);
    }

    return workerCount > 0 ? workerCount : 1;
}

void buildShapeCutPlans(const VoxelShape& object,
                        const Geometry::ShapeInstance& tool,
                        const ShapeToolContext& context,
                        const std::vector<VoxelCellAddress>& rootCells,
                        unsigned int workerCount,
                        std::vector<RootCutPlan>& plans)
{
    assert(!rootCells.empty());
    assert(workerCount > 0);
    assert(plans.size() == rootCells.size());

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
                const VoxelCellAddress& rootAddress = rootCells[rootIndex];
                const MyMath::Vector3 centerToolSpace = context.objectToTool.transformPoint(cellCenter(object, rootAddress));
                const VoxelTreeConstCursor cursor(object.forest(), rootAddress);

                currentPlan.root = buildShapeCutPlan(cursor, tool, context, rootAddress, centerToolSpace, object.grid().maximumLevel(), currentPlan);
                plans[rootIndex] = std::move(currentPlan);
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

bool hasShapeCutPlanModification(const std::vector<RootCutPlan>& plans)
{
    for (std::size_t planIndex = 0; planIndex < plans.size(); ++planIndex)
    {
        if (plans[planIndex].root.action != CutPlanAction::Keep)
        {
            return true;
        }
    }

    return false;
}

// 递归应用一个连续Shape切削计划节点。
static VoxelState applyShapeCutPlan(VoxelTreeEditor& editor, const RootCutPlan& plan, const CutPlanNode& node)
{
    if (node.action == CutPlanAction::Keep)
    {
        assert(editor.state() == node.resultState);
        return node.resultState;
    }

    if (node.action == CutPlanAction::Remove)
    {
        assert(node.resultState == VoxelState::Empty);

        editor.setState(VoxelState::Empty);
        return VoxelState::Empty;
    }

    assert(node.action == CutPlanAction::Subdivide);
    assert(node.resultState == VoxelState::Subdivided);
    assert(node.childGroupIndex != InvalidCutPlanGroupIndex);
    assert(static_cast<std::size_t>(node.childGroupIndex) < plan.nodeGroups.size());

    const VoxelState currentState = editor.state();

    if (currentState == VoxelState::Material)
    {
        editor.split();
    }
    else
    {
        assert(currentState == VoxelState::Subdivided);
    }

    const CutPlanNodeGroup& group = plan.nodeGroups[static_cast<std::size_t>(node.childGroupIndex)];

    for (int cornerIndex = 0; cornerIndex < VoxelCornerCount; ++cornerIndex)
    {
        const CutPlanNode& childPlan = group.children[static_cast<std::size_t>(cornerIndex)];

        if (childPlan.action == CutPlanAction::Keep)
        {
            continue;
        }

        const VoxelCorner corner = static_cast<VoxelCorner>(cornerIndex);
        VoxelTreeEditor childEditor = editor.child(corner);

        applyShapeCutPlan(childEditor, plan, childPlan);
    }

    assert(editor.state() == VoxelState::Subdivided);
    return VoxelState::Subdivided;
}

void applyShapeCutPlans(std::vector<RootPlanApplication>& applications, unsigned int workerCount)
{
    if (applications.empty())
    {
        assert(workerCount == 0);
        return;
    }

    assert(workerCount > 0);
    assert(static_cast<std::size_t>(workerCount) <= applications.size());

    std::atomic<std::size_t> nextApplicationIndex(0);

    const auto worker =
        [&]()
        {
            while (true)
            {
                const std::size_t applicationIndex = nextApplicationIndex.fetch_add(1);

                if (applicationIndex >= applications.size())
                {
                    return;
                }

                RootPlanApplication& application = applications[applicationIndex];

                assert(application.plan);
                assert(application.plan->root.action == CutPlanAction::Subdivide);

                applyShapeCutPlan(application.editor, *application.plan, application.plan->root);
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