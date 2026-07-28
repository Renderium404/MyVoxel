#include "BooleanOperation.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <thread>
#include <utility>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "../Core/VoxelNodeForestConstAccessor.h"
#include "../Core/VoxelNodeForestConstCursor.h"
#include "../Core/VoxelNodeForestEditor.h"
#include "../Core/VoxelNodeForestQuery.h"
#include "../Shape/ShapeBounds.h"
#include "../Shape/ShapeRegionRelation.h"

namespace
{

using Clock = std::chrono::steady_clock;

const double BroadPhaseBoundsToleranceScale = 1.0e-9; // 宽相包围盒按第0层边长的十亿分之一向外扩展，避免边界舍入遗漏相邻根节点。
const unsigned int MaximumAutomaticPlanningWorkerCount = 8; // 自动并行最多使用8个线程，该数量来自当前12逻辑线程设备的实测最优区间。
const std::uint32_t InvalidCutPlanGroupIndex = (std::numeric_limits<std::uint32_t>::max)(); // 无效切削计划子节点组索引。

// 保存普通体素右操作数使用的固定变换数据。
struct CellTransformContext
{
    MyMath::Matrix4 transform; // 左操作数局部坐标到右操作数局部坐标的变换。
    double extentScaleX = 1.0; // 单位半边长在右操作数X方向产生的包围盒半宽比例。
    double extentScaleY = 1.0; // 单位半边长在右操作数Y方向产生的包围盒半宽比例。
    double extentScaleZ = 1.0; // 单位半边长在右操作数Z方向产生的包围盒半宽比例。
};

// 保存连续Shape在指定体素层级使用的固定递归数据。
struct ShapeToolLevelContext
{
    double halfExtentX = 0.0; // 当前层级体素映射到Shape局部空间后的X方向包围盒半宽。
    double halfExtentY = 0.0; // 当前层级体素映射到Shape局部空间后的Y方向包围盒半宽。
    double halfExtentZ = 0.0; // 当前层级体素映射到Shape局部空间后的Z方向包围盒半宽。
    MyMath::Vector3 childCenterOffsets[MyVoxel::VoxelCornerCount]; // 八个子节点中心相对父节点中心的Shape局部空间偏移。
};

// 保存连续Shape切削使用的宽相范围和全部层级递归数据。
struct ShapeToolContext
{
    MyMath::Matrix4 objectToTool; // 左操作数局部坐标到连续Shape局部坐标的变换。
    std::vector<ShapeToolLevelContext> levels; // 各体素层级预计算数据。
    MyVoxel::VoxelCellIndex minimumRootIndex; // 连续Shape包围盒覆盖的最小第0层索引。
    MyVoxel::VoxelCellIndex maximumRootIndex; // 连续Shape包围盒覆盖的最大第0层索引。
    std::uint64_t rootCandidateCount = 0; // 宽相索引范围包含的第0层候选数量。
};

// 表示左操作数体素与右操作数材料区域之间的关系。
enum class CellShapeRelation
{
    Outside,
    Intersecting,
    Inside
};

// 表示一个切削计划节点需要执行的操作。
enum class CutPlanAction : std::uint8_t
{
    Keep,
    Remove,
    Subdivide
};

// 表示一个连续Shape切削计划节点。
struct CutPlanNode
{
    CutPlanNode() = default;

    CutPlanNode(CutPlanAction actionValue, MyVoxel::VoxelState resultStateValue, std::uint32_t childGroupIndexValue = InvalidCutPlanGroupIndex)
        : action(actionValue)
        , resultState(resultStateValue)
        , childGroupIndex(childGroupIndexValue)
    {
    }

    CutPlanAction action = CutPlanAction::Keep; // 当前计划节点的处理动作。
    MyVoxel::VoxelState resultState = MyVoxel::VoxelState::Empty; // 应用当前计划后该节点的最终状态。
    std::uint32_t childGroupIndex = InvalidCutPlanGroupIndex; // Subdivide动作连接的八子节点组索引。
};

// 表示一个切削计划八子节点组。
struct CutPlanNodeGroup
{
    std::array<CutPlanNode, MyVoxel::VoxelCornerCount> children; // 八个角点对应的切削计划节点。
};

// 保存一个第0层根节点的完整切削计划和线程局部统计。
struct RootCutPlan
{
    // 分配一个切削计划子节点组并返回其索引。
    std::uint32_t allocateNodeGroup()
    {
        assert(nodeGroups.size() < static_cast<std::size_t>(InvalidCutPlanGroupIndex));
        nodeGroups.push_back(CutPlanNodeGroup());
        return static_cast<std::uint32_t>(nodeGroups.size() - 1);
    }

    CutPlanNode root; // 当前第0层根节点的切削计划。
    std::vector<CutPlanNodeGroup> nodeGroups; // 当前根计划使用的全部八子节点组。
    MyVoxel::BooleanOperationStatistics statistics; // 当前根节点分析阶段产生的线程局部统计。
};
// 保存一个已经完成根级写时复制的计划应用任务。
struct RootPlanApplication
{
    RootPlanApplication(MyVoxel::VoxelRootTree& treeValue, const RootCutPlan& planValue)
        : tree(&treeValue)
        , plan(&planValue)
    {
    }

    MyVoxel::VoxelRootTree* tree; // 当前任务独占修改的根树。
    const RootCutPlan* plan; // 当前根树对应的只读切削计划。
};

// 保存一次体素刀具递归处理后的节点状态和材料变化结果。
struct CutCellResult
{
    CutCellResult(MyVoxel::VoxelState stateValue, bool changedValue)
        : state(stateValue)
        , changed(changedValue)
    {
    }

    MyVoxel::VoxelState state; // 当前节点处理后的最终状态。
    bool changed; // 当前节点覆盖区域内是否实际删除了材料。
};




/// 公共基础计算

// 返回两个时间点之间的毫秒数。
double elapsedMilliseconds(const Clock::time_point& start, const Clock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}
// 返回指定体素地址所属的第0层根节点索引。
MyVoxel::VoxelCellIndex rootCellIndex(MyVoxel::VoxelCellAddress address)
{
    while (MyVoxel::hasParentCell(address))
    {
        address = MyVoxel::parentCellAddress(address);
    }

    return address.index;
}


// 返回指定体素在左操作数局部坐标中的中心点。
MyMath::Vector3 cellCenter(const MyVoxel::VoxelShape& object, const MyVoxel::VoxelCellAddress& address)
{
    const double edgeLength = object.voxelEdgeLength(address.level);
    const double centerOffset = edgeLength * 0.5; // 体素中心距离体素最小角为半个体素边长。
    return MyMath::Vector3(static_cast<double>(address.index.x) * edgeLength + centerOffset, static_cast<double>(address.index.y) * edgeLength + centerOffset, static_cast<double>(address.index.z) * edgeLength + centerOffset);
}

// 返回左操作数体素映射到右操作数局部坐标后的保守轴对齐包围盒。
MyVoxel::ShapeBounds transformedCellBounds(const MyVoxel::VoxelShape& object, const CellTransformContext& context, const MyVoxel::VoxelCellAddress& address)
{
    const double halfEdgeLength = object.voxelEdgeLength(address.level) * 0.5; // 原始体素中心到任一面的距离。
    const MyMath::Vector3 transformedCenter = context.transform.transformPoint(cellCenter(object, address));
    const double halfExtentX = halfEdgeLength * context.extentScaleX;
    const double halfExtentY = halfEdgeLength * context.extentScaleY;
    const double halfExtentZ = halfEdgeLength * context.extentScaleZ;
    return MyVoxel::ShapeBounds(transformedCenter.x() - halfExtentX, transformedCenter.y() - halfExtentY, transformedCenter.z() - halfExtentZ, transformedCenter.x() + halfExtentX, transformedCenter.y() + halfExtentY, transformedCenter.z() + halfExtentZ);
}

// 创建左操作数局部坐标到右操作数局部坐标的固定变换数据。
CellTransformContext createCellTransformContext(const MyMath::Matrix4& objectTransform, const MyMath::Matrix4& toolTransform)
{
    assert(objectTransform.isRigidTransform());
    assert(toolTransform.isRigidTransform());

    MyMath::Matrix4 worldToTool;
    const bool inverted = toolTransform.inverted(worldToTool);

    assert(inverted);

    CellTransformContext context;
    context.transform = worldToTool * objectTransform;
    context.extentScaleX = std::abs(context.transform(0, 0)) + std::abs(context.transform(0, 1)) + std::abs(context.transform(0, 2));
    context.extentScaleY = std::abs(context.transform(1, 0)) + std::abs(context.transform(1, 1)) + std::abs(context.transform(1, 2));
    context.extentScaleZ = std::abs(context.transform(2, 0)) + std::abs(context.transform(2, 1)) + std::abs(context.transform(2, 2));
    return context;
}

// 收集左操作数当前全部材料节点地址，避免修改森林时破坏遍历过程。
std::vector<MyVoxel::VoxelCellAddress> collectMaterialCells(const MyVoxel::VoxelShape& object)
{
    std::vector<MyVoxel::VoxelCellAddress> addresses;

    object.forest().forEachMaterialCell(
        [&](const MyVoxel::VoxelCellAddress& address)
        {
            addresses.push_back(address);
        });

    return addresses;
}

// 收集指定第0层索引范围内实际存在的根节点。
std::vector<MyVoxel::VoxelCellAddress> collectRootCellsInRange(const MyVoxel::VoxelShape& object, const MyVoxel::VoxelCellIndex& minimumRootIndex, const MyVoxel::VoxelCellIndex& maximumRootIndex, std::size_t* existingRootCount)
{
    std::vector<MyVoxel::VoxelCellAddress> addresses;

    const std::size_t rootCount = object.forest().forEachRootCellInRange(
        minimumRootIndex,
        maximumRootIndex,
        [&](const MyVoxel::VoxelCellAddress& address)
        {
            addresses.push_back(address);
        });

    if (existingRootCount)
    {
        *existingRootCount = rootCount;
    }

    return addresses;
}

// 将体素森林查询关系转换为布尔运算内部关系。
CellShapeRelation convertRelation(MyVoxel::VoxelRegionRelation relation)
{
    switch (relation)
    {
    case MyVoxel::VoxelRegionRelation::Outside:
        return CellShapeRelation::Outside;

    case MyVoxel::VoxelRegionRelation::Intersecting:
        return CellShapeRelation::Intersecting;

    case MyVoxel::VoxelRegionRelation::Inside:
        return CellShapeRelation::Inside;
    }

    assert(false);
    return CellShapeRelation::Outside;
}

// 将连续Shape查询关系转换为布尔运算内部关系。
CellShapeRelation convertRelation(MyVoxel::ShapeRegionRelation relation)
{
    switch (relation)
    {
    case MyVoxel::ShapeRegionRelation::Outside:
        return CellShapeRelation::Outside;

    case MyVoxel::ShapeRegionRelation::Intersecting:
        return CellShapeRelation::Intersecting;

    case MyVoxel::ShapeRegionRelation::Inside:
        return CellShapeRelation::Inside;
    }

    assert(false);
    return CellShapeRelation::Outside;
}

// 将一次体素森林区域查询统计累加到布尔运算统计中。
void accumulateQueryStatistics(MyVoxel::BooleanOperationStatistics& target, const MyVoxel::VoxelNodeForestQueryStatistics& source)
{
    target.queryRootCandidateCount += source.rootCandidateCount;
    target.queryExistingRootCount += source.existingRootCount;
    target.queryNodeBoundsTestCount += source.nodeBoundsTestCount;
    target.queryVisitedNodeCount += source.visitedNodeCount;
}

/// 连续Shape宽相与增量数据

// 判断角点是否使用X方向最大侧。
bool cornerUsesMaximumX(MyVoxel::VoxelCorner corner)
{
    return (static_cast<unsigned int>(corner) & 1U) != 0U;
}

// 判断角点是否使用Y方向最大侧。
bool cornerUsesMaximumY(MyVoxel::VoxelCorner corner)
{
    return (static_cast<unsigned int>(corner) & 2U) != 0U;
}

// 判断角点是否使用Z方向最大侧。
bool cornerUsesMaximumZ(MyVoxel::VoxelCorner corner)
{
    return (static_cast<unsigned int>(corner) & 4U) != 0U;
}

// 将连续坐标转换为第0层索引，并在超出索引类型范围时饱和到边界值。
MyVoxel::VoxelIndex clampedRootIndex(double coordinate, double baseVoxelEdgeLength)
{
    assert(std::isfinite(coordinate));
    assert(std::isfinite(baseVoxelEdgeLength) && baseVoxelEdgeLength > 0.0);

    const double indexValue = std::floor(coordinate / baseVoxelEdgeLength);
    const double minimumValue = static_cast<double>((std::numeric_limits<MyVoxel::VoxelIndex>::min)());
    const double maximumValue = static_cast<double>((std::numeric_limits<MyVoxel::VoxelIndex>::max)());

    if (indexValue <= minimumValue)
    {
        return (std::numeric_limits<MyVoxel::VoxelIndex>::min)();
    }

    if (indexValue >= maximumValue)
    {
        return (std::numeric_limits<MyVoxel::VoxelIndex>::max)();
    }

    return static_cast<MyVoxel::VoxelIndex>(indexValue);
}

// 返回包含两端点的索引范围长度。
std::uint64_t inclusiveRangeLength(MyVoxel::VoxelIndex minimumValue, MyVoxel::VoxelIndex maximumValue)
{
    assert(minimumValue <= maximumValue);
    return static_cast<std::uint64_t>(static_cast<std::int64_t>(maximumValue) - static_cast<std::int64_t>(minimumValue)) + 1;
}

// 执行无符号64位饱和乘法。
std::uint64_t saturatedMultiply(std::uint64_t first, std::uint64_t second)
{
    if (first == 0 || second == 0)
    {
        return 0;
    }

    const std::uint64_t maximumValue = (std::numeric_limits<std::uint64_t>::max)();

    if (first > maximumValue / second)
    {
        return maximumValue;
    }

    return first * second;
}

// 返回三维第0层索引范围包含的候选单元数量。
std::uint64_t rootRangeCandidateCount(const MyVoxel::VoxelCellIndex& minimumIndex, const MyVoxel::VoxelCellIndex& maximumIndex)
{
    const std::uint64_t xCount = inclusiveRangeLength(minimumIndex.x, maximumIndex.x);
    const std::uint64_t yCount = inclusiveRangeLength(minimumIndex.y, maximumIndex.y);
    const std::uint64_t zCount = inclusiveRangeLength(minimumIndex.z, maximumIndex.z);
    return saturatedMultiply(saturatedMultiply(xCount, yCount), zCount);
}

// 将局部轴对齐包围盒转换到目标坐标系并返回其目标空间轴对齐包围盒。
MyVoxel::ShapeBounds transformedShapeBounds(const MyVoxel::ShapeBounds& bounds, const MyMath::Matrix4& transform)
{
    assert(bounds.isValid());
    assert(transform.isRigidTransform());

    double minimumX = (std::numeric_limits<double>::max)();
    double minimumY = (std::numeric_limits<double>::max)();
    double minimumZ = (std::numeric_limits<double>::max)();
    double maximumX = -(std::numeric_limits<double>::max)();
    double maximumY = -(std::numeric_limits<double>::max)();
    double maximumZ = -(std::numeric_limits<double>::max)();

    for (int cornerIndex = 0; cornerIndex < MyVoxel::VoxelCornerCount; ++cornerIndex)
    {
        const bool maximumSideX = (cornerIndex & 1) != 0;
        const bool maximumSideY = (cornerIndex & 2) != 0;
        const bool maximumSideZ = (cornerIndex & 4) != 0;
        const MyMath::Vector3 corner(maximumSideX ? bounds.maximumX : bounds.minimumX, maximumSideY ? bounds.maximumY : bounds.minimumY, maximumSideZ ? bounds.maximumZ : bounds.minimumZ);
        const MyMath::Vector3 transformedCorner = transform.transformPoint(corner);

        minimumX = std::min(minimumX, transformedCorner.x());
        minimumY = std::min(minimumY, transformedCorner.y());
        minimumZ = std::min(minimumZ, transformedCorner.z());
        maximumX = std::max(maximumX, transformedCorner.x());
        maximumY = std::max(maximumY, transformedCorner.y());
        maximumZ = std::max(maximumZ, transformedCorner.z());
    }

    return MyVoxel::ShapeBounds(minimumX, minimumY, minimumZ, maximumX, maximumY, maximumZ);
}

// 创建连续Shape切削使用的宽相范围、层级包围盒和子节点增量偏移。
ShapeToolContext createShapeToolContext(const MyVoxel::VoxelShape& object, const MyVoxel::Shape& tool)
{
    assert(object.transform().isRigidTransform());
    assert(tool.transform().isRigidTransform());

    const MyVoxel::ShapeBounds toolLocalBounds = tool.localBounds();

    assert(toolLocalBounds.isValid());
    assert(toolLocalBounds.hasVolume());

    MyMath::Matrix4 worldToObject;
    MyMath::Matrix4 worldToTool;

    const bool objectInverted = object.transform().inverted(worldToObject);
    const bool toolInverted = tool.transform().inverted(worldToTool);

    assert(objectInverted);
    assert(toolInverted);

    ShapeToolContext context;
    context.objectToTool = worldToTool * object.transform();

    const MyMath::Matrix4 toolToObject = worldToObject * tool.transform();
    const MyVoxel::ShapeBounds toolObjectBounds = transformedShapeBounds(toolLocalBounds, toolToObject);
    const double baseVoxelEdgeLength = object.baseVoxelEdgeLength();
    const double boundsTolerance = baseVoxelEdgeLength * BroadPhaseBoundsToleranceScale;

    context.minimumRootIndex = MyVoxel::VoxelCellIndex(clampedRootIndex(toolObjectBounds.minimumX - boundsTolerance, baseVoxelEdgeLength), clampedRootIndex(toolObjectBounds.minimumY - boundsTolerance, baseVoxelEdgeLength), clampedRootIndex(toolObjectBounds.minimumZ - boundsTolerance, baseVoxelEdgeLength));
    context.maximumRootIndex = MyVoxel::VoxelCellIndex(clampedRootIndex(toolObjectBounds.maximumX + boundsTolerance, baseVoxelEdgeLength), clampedRootIndex(toolObjectBounds.maximumY + boundsTolerance, baseVoxelEdgeLength), clampedRootIndex(toolObjectBounds.maximumZ + boundsTolerance, baseVoxelEdgeLength));
    context.rootCandidateCount = rootRangeCandidateCount(context.minimumRootIndex, context.maximumRootIndex);

    const MyMath::Vector3 transformedAxisX = context.objectToTool.transformVector(MyMath::Vector3::unitX());
    const MyMath::Vector3 transformedAxisY = context.objectToTool.transformVector(MyMath::Vector3::unitY());
    const MyMath::Vector3 transformedAxisZ = context.objectToTool.transformVector(MyMath::Vector3::unitZ());
    const double extentScaleX = std::abs(transformedAxisX.x()) + std::abs(transformedAxisY.x()) + std::abs(transformedAxisZ.x());
    const double extentScaleY = std::abs(transformedAxisX.y()) + std::abs(transformedAxisY.y()) + std::abs(transformedAxisZ.y());
    const double extentScaleZ = std::abs(transformedAxisX.z()) + std::abs(transformedAxisY.z()) + std::abs(transformedAxisZ.z());
    const int maximumLevel = static_cast<int>(object.maximumLevel());

    context.levels.resize(static_cast<std::size_t>(maximumLevel + 1));

    for (int levelIndex = 0; levelIndex <= maximumLevel; ++levelIndex)
    {
        const MyVoxel::VoxelLevel level = static_cast<MyVoxel::VoxelLevel>(levelIndex);
        const double edgeLength = object.voxelEdgeLength(level);
        const double halfEdgeLength = edgeLength * 0.5;
        ShapeToolLevelContext& levelContext = context.levels[static_cast<std::size_t>(levelIndex)];

        levelContext.halfExtentX = halfEdgeLength * extentScaleX;
        levelContext.halfExtentY = halfEdgeLength * extentScaleY;
        levelContext.halfExtentZ = halfEdgeLength * extentScaleZ;

        if (levelIndex >= maximumLevel)
        {
            continue;
        }

        const double childCenterOffset = edgeLength * 0.25; // 子节点中心相对父节点中心沿每个工件轴偏移父节点边长的四分之一。

        for (int cornerIndex = 0; cornerIndex < MyVoxel::VoxelCornerCount; ++cornerIndex)
        {
            const MyVoxel::VoxelCorner corner = static_cast<MyVoxel::VoxelCorner>(cornerIndex);
            const double signX = cornerUsesMaximumX(corner) ? 1.0 : -1.0;
            const double signY = cornerUsesMaximumY(corner) ? 1.0 : -1.0;
            const double signZ = cornerUsesMaximumZ(corner) ? 1.0 : -1.0;

            levelContext.childCenterOffsets[cornerIndex] = transformedAxisX * (childCenterOffset * signX) + transformedAxisY * (childCenterOffset * signY) + transformedAxisZ * (childCenterOffset * signZ);
        }
    }

    return context;
}

// 返回连续Shape局部空间中当前体素中心和预计算半宽对应的包围盒。
MyVoxel::ShapeBounds shapeToolCellBounds(const ShapeToolContext& context, MyVoxel::VoxelLevel level, const MyMath::Vector3& centerToolSpace)
{
    assert(static_cast<std::size_t>(level) < context.levels.size());

    const ShapeToolLevelContext& levelContext = context.levels[static_cast<std::size_t>(level)];
    return MyVoxel::ShapeBounds(centerToolSpace.x() - levelContext.halfExtentX, centerToolSpace.y() - levelContext.halfExtentY, centerToolSpace.z() - levelContext.halfExtentZ, centerToolSpace.x() + levelContext.halfExtentX, centerToolSpace.y() + levelContext.halfExtentY, centerToolSpace.z() + levelContext.halfExtentZ);
}

/// 普通体素右操作数路径

// 返回左操作数体素与体素右操作数材料区域之间的保守空间关系。
CellShapeRelation classifyObjectCellWithVoxelTool(const MyVoxel::VoxelShape& object, const CellTransformContext& context, const MyVoxel::VoxelNodeForestQuery& toolQuery, const MyVoxel::VoxelCellAddress& address)
{
    const MyVoxel::ShapeBounds bounds = transformedCellBounds(object, context, address);
    const MyVoxel::VoxelLocalBox queryBounds(bounds.minimumX, bounds.minimumY, bounds.minimumZ, bounds.maximumX, bounds.maximumY, bounds.maximumZ);
    return convertRelation(toolQuery.classify(queryBounds));
}

// 检查左操作数体素中心映射后是否位于体素右操作数材料中。
bool voxelToolContainsObjectCellCenter(const MyVoxel::VoxelShape& object, const MyVoxel::VoxelShape& tool, const CellTransformContext& context, MyVoxel::VoxelNodeForestConstAccessor& toolAccessor, const MyVoxel::VoxelCellAddress& address)
{
    const MyMath::Vector3 toolPoint = context.transform.transformPoint(cellCenter(object, address));
    const MyVoxel::VoxelCellAddress toolAddress = tool.localAddressAt(toolPoint, tool.maximumLevel());
    return toolAccessor.state(toolAddress) == MyVoxel::VoxelState::Material;
}

// 递归对指定材料节点执行体素右操作数布尔减，并返回处理后的节点状态。
// 递归对指定材料节点执行体素右操作数布尔减，并返回节点状态和材料变化结果。
CutCellResult cutMaterialCellWithVoxelTool(MyVoxel::VoxelNodeForestEditor& editor, const MyVoxel::VoxelShape& object, const MyVoxel::VoxelShape& tool, const CellTransformContext& context, const MyVoxel::VoxelNodeForestQuery& toolQuery, MyVoxel::VoxelNodeForestConstAccessor& toolAccessor, const MyVoxel::VoxelCellAddress& address, MyVoxel::VoxelLevel targetLevel)
{
    if (address.level == targetLevel)
    {
        if (voxelToolContainsObjectCellCenter(object, tool, context, toolAccessor, address))
        {
            editor.setState(MyVoxel::VoxelState::Empty);
            return CutCellResult(MyVoxel::VoxelState::Empty, true);
        }

        return CutCellResult(MyVoxel::VoxelState::Material, false);
    }

    const CellShapeRelation relation = classifyObjectCellWithVoxelTool(object, context, toolQuery, address);

    if (relation == CellShapeRelation::Outside)
    {
        return CutCellResult(MyVoxel::VoxelState::Material, false);
    }

    if (relation == CellShapeRelation::Inside)
    {
        editor.setState(MyVoxel::VoxelState::Empty);
        return CutCellResult(MyVoxel::VoxelState::Empty, true);
    }

    assert(address.level < targetLevel);

    editor.split();

    bool changed = false;

    for (int i = 0; i < MyVoxel::VoxelCornerCount; ++i)
    {
        const MyVoxel::VoxelCorner corner = static_cast<MyVoxel::VoxelCorner>(i);
        const MyVoxel::VoxelCellAddress childAddress = MyVoxel::childCellAddress(address, corner);
        MyVoxel::VoxelNodeForestEditor childEditor = editor.child(corner);
        const CutCellResult childResult = cutMaterialCellWithVoxelTool(childEditor, object, tool, context, toolQuery, toolAccessor, childAddress, targetLevel);

        changed = childResult.changed || changed;
    }

    return CutCellResult(editor.merge(), changed);
}




// 执行不带统计的体素右操作数布尔减。
// 执行不带统计的体素右操作数布尔减，并在changes非空时记录修改根节点。
MyVoxel::VoxelShape cutWithVoxelToolImpl(const MyVoxel::VoxelShape& object, const MyVoxel::VoxelShape& tool, MyVoxel::VoxelChangeSet* changes)
{
    if (changes)
    {
        changes->clear();
    }

    assert(object.transform().isRigidTransform());
    assert(tool.transform().isRigidTransform());

    if (tool.forest().rootCount() == 0)
    {
        return object;
    }

    const std::vector<MyVoxel::VoxelCellAddress> materialCells = collectMaterialCells(object);

    if (materialCells.empty())
    {
        return object;
    }

    const MyVoxel::VoxelNodeForestQuery toolQuery(tool.forest(), tool.baseVoxelEdgeLength());
    MyVoxel::VoxelNodeForestConstAccessor toolAccessor(tool.forest());
    const CellTransformContext transformContext = createCellTransformContext(object.transform(), tool.transform());
    MyVoxel::VoxelShape result = object;
    MyVoxel::VoxelNodeForest& resultForest = result.forest();
    const MyVoxel::VoxelLevel targetLevel = object.maximumLevel();

    for (std::size_t i = 0; i < materialCells.size(); ++i)
    {
        assert(materialCells[i].level <= targetLevel);

        MyVoxel::VoxelNodeForestEditor editor(resultForest, materialCells[i]);
        const CutCellResult cellResult = cutMaterialCellWithVoxelTool(editor, object, tool, transformContext, toolQuery, toolAccessor, materialCells[i], targetLevel);

        if (cellResult.changed && changes)
        {
            changes->addModifiedRoot(rootCellIndex(materialCells[i]));
        }

        if (cellResult.state == MyVoxel::VoxelState::Empty)
        {
            resultForest.pruneEmptyBranch(materialCells[i]);
        }
    }

    return result;
}
/// 带统计体素右操作数路径

// 返回左操作数体素与体素右操作数材料区域之间的保守关系，并记录查询统计。
CellShapeRelation classifyObjectCellWithVoxelToolStatistics(const MyVoxel::VoxelShape& object, const CellTransformContext& context, const MyVoxel::VoxelNodeForestQuery& toolQuery, const MyVoxel::VoxelCellAddress& address, MyVoxel::BooleanOperationStatistics& statistics)
{
    ++statistics.classifiedCellCount;

    const MyVoxel::ShapeBounds bounds = transformedCellBounds(object, context, address);
    const MyVoxel::VoxelLocalBox queryBounds(bounds.minimumX, bounds.minimumY, bounds.minimumZ, bounds.maximumX, bounds.maximumY, bounds.maximumZ);
    MyVoxel::VoxelNodeForestQueryStatistics queryStatistics;
    const MyVoxel::VoxelRegionRelation relation = toolQuery.classify(queryBounds, queryStatistics);

    accumulateQueryStatistics(statistics, queryStatistics);
    return convertRelation(relation);
}

// 检查左操作数体素中心是否位于体素右操作数材料中，并记录点查询统计。
bool voxelToolContainsObjectCellCenterStatistics(const MyVoxel::VoxelShape& object, const MyVoxel::VoxelShape& tool, const CellTransformContext& context, MyVoxel::VoxelNodeForestConstAccessor& toolAccessor, const MyVoxel::VoxelCellAddress& address, MyVoxel::BooleanOperationStatistics& statistics)
{
    ++statistics.pointQueryCount;

    const MyMath::Vector3 toolPoint = context.transform.transformPoint(cellCenter(object, address));
    const MyVoxel::VoxelCellAddress toolAddress = tool.localAddressAt(toolPoint, tool.maximumLevel());
    return toolAccessor.state(toolAddress) == MyVoxel::VoxelState::Material;
}

// 递归对指定材料节点执行带统计的体素右操作数布尔减，并返回处理后的节点状态。
// 递归对指定材料节点执行带统计的体素右操作数布尔减，并返回节点状态和材料变化结果。
CutCellResult cutMaterialCellWithVoxelToolStatistics(MyVoxel::VoxelNodeForestEditor& editor, const MyVoxel::VoxelShape& object, const MyVoxel::VoxelShape& tool, const CellTransformContext& context, const MyVoxel::VoxelNodeForestQuery& toolQuery, MyVoxel::VoxelNodeForestConstAccessor& toolAccessor, const MyVoxel::VoxelCellAddress& address, MyVoxel::VoxelLevel targetLevel, MyVoxel::BooleanOperationStatistics& statistics)
{
    ++statistics.visitedCellCount;

    if (address.level == targetLevel)
    {
        ++statistics.centerSampleCount;

        if (voxelToolContainsObjectCellCenterStatistics(object, tool, context, toolAccessor, address, statistics))
        {
            ++statistics.centerRemovedCellCount;
            editor.setState(MyVoxel::VoxelState::Empty);
            return CutCellResult(MyVoxel::VoxelState::Empty, true);
        }

        return CutCellResult(MyVoxel::VoxelState::Material, false);
    }

    const CellShapeRelation relation = classifyObjectCellWithVoxelToolStatistics(object, context, toolQuery, address, statistics);

    if (relation == CellShapeRelation::Outside)
    {
        ++statistics.outsideCellCount;
        return CutCellResult(MyVoxel::VoxelState::Material, false);
    }

    if (relation == CellShapeRelation::Inside)
    {
        ++statistics.insideCellCount;
        ++statistics.removedBranchCount;
        editor.setState(MyVoxel::VoxelState::Empty);
        return CutCellResult(MyVoxel::VoxelState::Empty, true);
    }

    ++statistics.intersectingCellCount;

    assert(address.level < targetLevel);

    editor.split();

    bool changed = false;

    for (int i = 0; i < MyVoxel::VoxelCornerCount; ++i)
    {
        const MyVoxel::VoxelCorner corner = static_cast<MyVoxel::VoxelCorner>(i);
        const MyVoxel::VoxelCellAddress childAddress = MyVoxel::childCellAddress(address, corner);
        MyVoxel::VoxelNodeForestEditor childEditor = editor.child(corner);
        const CutCellResult childResult = cutMaterialCellWithVoxelToolStatistics(childEditor, object, tool, context, toolQuery, toolAccessor, childAddress, targetLevel, statistics);

        changed = childResult.changed || changed;
    }

    ++statistics.mergeAttemptCount;

    const MyVoxel::VoxelState mergedState = editor.merge();

    if (mergedState != MyVoxel::VoxelState::Subdivided)
    {
        ++statistics.mergeSuccessCount;
    }

    return CutCellResult(mergedState, changed);
}
// 执行带计时和统计的体素右操作数布尔减。
// 执行带计时和统计的体素右操作数布尔减，并在changes非空时记录修改根节点。
MyVoxel::VoxelShape cutWithVoxelToolStatisticsImpl(const MyVoxel::VoxelShape& object, const MyVoxel::VoxelShape& tool, MyVoxel::BooleanOperationStatistics& statistics, MyVoxel::VoxelChangeSet* changes)
{
    statistics.reset();

    if (changes)
    {
        changes->clear();
    }

    const Clock::time_point totalStart = Clock::now();

    assert(object.transform().isRigidTransform());
    assert(tool.transform().isRigidTransform());

    const Clock::time_point toolIndexStart = Clock::now();
    const MyVoxel::VoxelNodeForestQuery toolQuery(tool.forest(), tool.baseVoxelEdgeLength());
    MyVoxel::VoxelNodeForestConstAccessor toolAccessor(tool.forest());
    const CellTransformContext transformContext = createCellTransformContext(object.transform(), tool.transform());
    const bool toolEmpty = tool.forest().rootCount() == 0;
    const Clock::time_point toolIndexEnd = Clock::now();

    statistics.toolIndexMilliseconds = elapsedMilliseconds(toolIndexStart, toolIndexEnd);

    if (toolEmpty)
    {
        statistics.totalMilliseconds = elapsedMilliseconds(totalStart, Clock::now());
        return object;
    }

    const Clock::time_point materialCollectionStart = Clock::now();
    const std::vector<MyVoxel::VoxelCellAddress> materialCells = collectMaterialCells(object);
    const Clock::time_point materialCollectionEnd = Clock::now();

    statistics.materialCollectionMilliseconds = elapsedMilliseconds(materialCollectionStart, materialCollectionEnd);
    statistics.inputMaterialCellCount = static_cast<std::uint64_t>(materialCells.size());

    if (materialCells.empty())
    {
        statistics.totalMilliseconds = elapsedMilliseconds(totalStart, Clock::now());
        return object;
    }

    MyVoxel::VoxelShape result = object;

    const Clock::time_point detachStart = Clock::now();
    MyVoxel::VoxelNodeForest& resultForest = result.forest();
    const Clock::time_point detachEnd = Clock::now();

    statistics.detachMilliseconds = elapsedMilliseconds(detachStart, detachEnd);

    const MyVoxel::VoxelLevel targetLevel = object.maximumLevel();
    const Clock::time_point cuttingStart = Clock::now();

    for (std::size_t i = 0; i < materialCells.size(); ++i)
    {
        assert(materialCells[i].level <= targetLevel);

        MyVoxel::VoxelNodeForestEditor editor(resultForest, materialCells[i]);
        const CutCellResult cellResult = cutMaterialCellWithVoxelToolStatistics(editor, object, tool, transformContext, toolQuery, toolAccessor, materialCells[i], targetLevel, statistics);

        if (cellResult.changed && changes)
        {
            changes->addModifiedRoot(rootCellIndex(materialCells[i]));
        }

        if (cellResult.state == MyVoxel::VoxelState::Empty)
        {
            resultForest.pruneEmptyBranch(materialCells[i]);
        }
    }

    const Clock::time_point cuttingEnd = Clock::now();

    statistics.cuttingMilliseconds = elapsedMilliseconds(cuttingStart, cuttingEnd);

    const MyVoxel::VoxelNodeForestConstAccessorStatistics& accessorStatistics = toolAccessor.statistics();
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
/// 连续Shape并行切削计划

// 返回左操作数体素与连续右操作数材料区域之间的保守空间关系。
CellShapeRelation classifyObjectCellWithShapeTool(const MyVoxel::Shape& tool, const ShapeToolContext& context, const MyVoxel::VoxelCellAddress& address, const MyMath::Vector3& centerToolSpace)
{
    return convertRelation(tool.classifyLocalBounds(shapeToolCellBounds(context, address.level, centerToolSpace)));
}

// 返回连续Shape区域分类结果，并记录分类统计。
CellShapeRelation classifyObjectCellWithShapeToolStatistics(const MyVoxel::Shape& tool, const ShapeToolContext& context, const MyVoxel::VoxelCellAddress& address, const MyMath::Vector3& centerToolSpace, MyVoxel::BooleanOperationStatistics& statistics)
{
    ++statistics.classifiedCellCount;
    return classifyObjectCellWithShapeTool(tool, context, address, centerToolSpace);
}

// 检查最高层体素中心是否位于连续Shape材料中，并记录点查询统计。
bool shapeToolContainsObjectCellCenterStatistics(const MyVoxel::Shape& tool, const MyMath::Vector3& centerToolSpace, MyVoxel::BooleanOperationStatistics& statistics)
{
    ++statistics.pointQueryCount;
    return tool.containsLocalPoint(centerToolSpace);
}

// 检查八个子计划是否全部使用指定动作。
bool allChildPlansUseAction(const RootCutPlan& plan, std::uint32_t groupIndex, CutPlanAction action)
{
    assert(static_cast<std::size_t>(groupIndex) < plan.nodeGroups.size());

    const CutPlanNodeGroup& group = plan.nodeGroups[static_cast<std::size_t>(groupIndex)];

    for (std::size_t i = 0; i < group.children.size(); ++i)
    {
        if (group.children[i].action != action)
        {
            return false;
        }
    }

    return true;
}

// 检查八个子计划执行后是否全部得到指定节点状态。
bool allChildPlansResultInState(const RootCutPlan& plan, std::uint32_t groupIndex, MyVoxel::VoxelState state)
{
    assert(static_cast<std::size_t>(groupIndex) < plan.nodeGroups.size());

    const CutPlanNodeGroup& group = plan.nodeGroups[static_cast<std::size_t>(groupIndex)];

    for (std::size_t i = 0; i < group.children.size(); ++i)
    {
        if (group.children[i].resultState != state)
        {
            return false;
        }
    }

    return true;
}
// 递归读取原始体素树并生成当前节点的连续Shape切削计划。
// 递归读取原始体素树并生成当前节点的连续Shape切削计划。
CutPlanNode buildShapeCutPlan(const MyVoxel::VoxelNodeForestConstCursor& cursor, const MyVoxel::Shape& tool, const ShapeToolContext& context, const MyVoxel::VoxelCellAddress& address, const MyMath::Vector3& centerToolSpace, MyVoxel::VoxelLevel targetLevel, RootCutPlan& plan, MyVoxel::BooleanOperationStatistics* statistics)
{
    if (statistics)
    {
        ++statistics->visitedCellCount;
        ++statistics->cutPlanNodeCount;
    }

    const MyVoxel::VoxelState currentState = cursor.state();

    // 已有空节点保持为空，不需要执行连续几何查询。
    if (currentState == MyVoxel::VoxelState::Empty)
    {
        if (statistics)
        {
            ++statistics->emptySkippedCellCount;
        }

        return CutPlanNode(CutPlanAction::Keep, MyVoxel::VoxelState::Empty);
    }

    // 最高层材料节点通过中心采样决定最终状态。
    if (address.level == targetLevel)
    {
        assert(currentState == MyVoxel::VoxelState::Material);

        if (statistics)
        {
            ++statistics->centerSampleCount;

            if (shapeToolContainsObjectCellCenterStatistics(tool, centerToolSpace, *statistics))
            {
                ++statistics->centerRemovedCellCount;
                return CutPlanNode(CutPlanAction::Remove, MyVoxel::VoxelState::Empty);
            }
        }
        else if (tool.containsLocalPoint(centerToolSpace))
        {
            return CutPlanNode(CutPlanAction::Remove, MyVoxel::VoxelState::Empty);
        }

        return CutPlanNode(CutPlanAction::Keep, MyVoxel::VoxelState::Material);
    }

    const CellShapeRelation relation = statistics ?
        classifyObjectCellWithShapeToolStatistics(tool, context, address, centerToolSpace, *statistics) :
        classifyObjectCellWithShapeTool(tool, context, address, centerToolSpace);

    if (relation == CellShapeRelation::Outside)
    {
        if (statistics)
        {
            ++statistics->outsideCellCount;
        }

        return CutPlanNode(CutPlanAction::Keep, currentState);
    }

    if (relation == CellShapeRelation::Inside)
    {
        if (statistics)
        {
            ++statistics->insideCellCount;
            ++statistics->removedBranchCount;
        }

        return CutPlanNode(CutPlanAction::Remove, MyVoxel::VoxelState::Empty);
    }

    if (statistics)
    {
        ++statistics->intersectingCellCount;
    }

    assert(address.level < targetLevel);

    const std::size_t originalGroupCount = plan.nodeGroups.size();
    const std::uint32_t groupIndex = plan.allocateNodeGroup();
    const ShapeToolLevelContext& levelContext = context.levels[static_cast<std::size_t>(address.level)];

    for (int i = 0; i < MyVoxel::VoxelCornerCount; ++i)
    {
        const MyVoxel::VoxelCorner corner = static_cast<MyVoxel::VoxelCorner>(i);
        const MyVoxel::VoxelCellAddress childAddress = MyVoxel::childCellAddress(address, corner);
        const MyMath::Vector3 childCenterToolSpace = centerToolSpace + levelContext.childCenterOffsets[i];
        const MyVoxel::VoxelNodeForestConstCursor childCursor = cursor.child(corner);

        // 递归可能使nodeGroups扩容，因此不能跨递归持有其中元素的引用。
        const CutPlanNode childPlan = buildShapeCutPlan(childCursor, tool, context, childAddress, childCenterToolSpace, targetLevel, plan, statistics);
        plan.nodeGroups[static_cast<std::size_t>(groupIndex)].children[static_cast<std::size_t>(i)] = childPlan;
    }

    // 八个子计划全部保持原状态时，父节点也无需修改。
    if (allChildPlansUseAction(plan, groupIndex, CutPlanAction::Keep))
    {
        plan.nodeGroups.resize(originalGroupCount);
        return CutPlanNode(CutPlanAction::Keep, currentState);
    }

    // 只要八个子节点最终全部为空，就直接删除当前完整分支。
    if (allChildPlansResultInState(plan, groupIndex, MyVoxel::VoxelState::Empty))
    {
        plan.nodeGroups.resize(originalGroupCount);
        return CutPlanNode(CutPlanAction::Remove, MyVoxel::VoxelState::Empty);
    }

    return CutPlanNode(CutPlanAction::Subdivide, MyVoxel::VoxelState::Subdivided, groupIndex);
}

// 返回指定数量独立任务实际使用的工作线程数量。
unsigned int resolveParallelWorkerCount(const MyVoxel::BooleanOperationOptions& options, std::size_t taskCount)
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

        if (workerCount > MaximumAutomaticPlanningWorkerCount)
        {
            workerCount = MaximumAutomaticPlanningWorkerCount;
        }
    }

    if (static_cast<std::size_t>(workerCount) > taskCount)
    {
        workerCount = static_cast<unsigned int>(taskCount);
    }

    return workerCount > 0 ? workerCount : 1;
}



// 为全部候选根节点生成切削计划。
void buildShapeCutPlans(const MyVoxel::VoxelShape& object, const MyVoxel::Shape& tool, const ShapeToolContext& context, const std::vector<MyVoxel::VoxelCellAddress>& rootCells, unsigned int workerCount, bool collectStatistics, std::vector<RootCutPlan>& plans)
{
    assert(!rootCells.empty());
    assert(workerCount > 0);
    assert(plans.size() == rootCells.size());

    std::atomic<std::size_t> nextRootIndex(0);

    const auto worker = [&]()
    {
        while (true)
        {
            const std::size_t rootIndex = nextRootIndex.fetch_add(1);

            if (rootIndex >= rootCells.size())
            {
                return;
            }

            RootCutPlan currentPlan;
            MyVoxel::BooleanOperationStatistics* localStatistics = collectStatistics ? &currentPlan.statistics : nullptr;
            const MyVoxel::VoxelCellAddress& rootAddress = rootCells[rootIndex];
            const MyMath::Vector3 centerToolSpace = context.objectToTool.transformPoint(cellCenter(object, rootAddress));
            const MyVoxel::VoxelNodeForestConstCursor cursor(object.forest(), rootAddress);

            currentPlan.root = buildShapeCutPlan(cursor, tool, context, rootAddress, centerToolSpace, object.maximumLevel(), currentPlan, localStatistics);

            if (localStatistics)
            {
                localStatistics->cutPlanGroupCount = static_cast<std::uint64_t>(currentPlan.nodeGroups.size());
            }

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

    // 调用线程也参与任务处理，因此只额外创建workerCount-1个工作线程。
    for (unsigned int i = 1; i < workerCount; ++i)
    {
        workers.push_back(std::thread(worker));
    }

    worker();

    for (std::size_t i = 0; i < workers.size(); ++i)
    {
        workers[i].join();
    }
}

// 检查切削计划是否包含实际修改。
bool hasShapeCutPlanModification(const std::vector<RootCutPlan>& plans)
{
    for (std::size_t i = 0; i < plans.size(); ++i)
    {
        if (plans[i].root.action != CutPlanAction::Keep)
        {
            return true;
        }
    }

    return false;
}

// 递归应用一个连续Shape切削计划节点。
// 递归应用一个连续Shape切削计划节点。
MyVoxel::VoxelState applyShapeCutPlan(MyVoxel::VoxelNodeForestEditor& editor, const RootCutPlan& plan, const CutPlanNode& node)
{
    if (node.action == CutPlanAction::Keep)
    {
        assert(editor.state() == node.resultState);
        return node.resultState;
    }

    if (node.action == CutPlanAction::Remove)
    {
        assert(node.resultState == MyVoxel::VoxelState::Empty);

        editor.setState(MyVoxel::VoxelState::Empty);
        return MyVoxel::VoxelState::Empty;
    }

    assert(node.action == CutPlanAction::Subdivide);
    assert(node.resultState == MyVoxel::VoxelState::Subdivided);
    assert(node.childGroupIndex != InvalidCutPlanGroupIndex);
    assert(static_cast<std::size_t>(node.childGroupIndex) < plan.nodeGroups.size());

    const MyVoxel::VoxelState currentState = editor.state();

    if (currentState == MyVoxel::VoxelState::Material)
    {
        editor.split();
    }
    else
    {
        assert(currentState == MyVoxel::VoxelState::Subdivided);
    }

    const CutPlanNodeGroup& group = plan.nodeGroups[static_cast<std::size_t>(node.childGroupIndex)];

    for (int i = 0; i < MyVoxel::VoxelCornerCount; ++i)
    {
        const CutPlanNode& childPlan = group.children[static_cast<std::size_t>(i)];

        // Keep表示完整保留当前子节点及其现有子树，不需要创建编辑器或继续递归。
        if (childPlan.action == CutPlanAction::Keep)
        {
            continue;
        }

        const MyVoxel::VoxelCorner corner = static_cast<MyVoxel::VoxelCorner>(i);
        MyVoxel::VoxelNodeForestEditor childEditor = editor.child(corner);
        applyShapeCutPlan(childEditor, plan, childPlan);
    }

    // 计划生成阶段已将最终全空的子节点组压缩为Remove，因此Subdivide计划执行后必然仍为混合状态。
    assert(editor.state() == MyVoxel::VoxelState::Subdivided);
    return MyVoxel::VoxelState::Subdivided;
}

// 并行应用全部已经完成根级写时复制的切削计划。
void applyShapeCutPlans(const std::vector<RootPlanApplication>& applications, unsigned int workerCount)
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

                const RootPlanApplication& application = applications[applicationIndex];

                assert(application.tree);
                assert(application.plan);
                assert(application.plan->root.action == CutPlanAction::Subdivide);

                MyVoxel::VoxelNodeForestEditor editor(*application.tree);
                applyShapeCutPlan(editor, *application.plan, application.plan->root);
            }
        };

    if (workerCount == 1)
    {
        worker();
        return;
    }

    std::vector<std::thread> workers;
    workers.reserve(static_cast<std::size_t>(workerCount - 1));

    // 调用线程也参与计划应用，因此只额外创建workerCount-1个工作线程。
    for (unsigned int i = 1; i < workerCount; ++i)
    {
        workers.push_back(std::thread(worker));
    }

    worker();

    for (std::size_t i = 0; i < workers.size(); ++i)
    {
        workers[i].join();
    }
}

// 执行连续Shape右操作数布尔减，并在statistics非空时记录统计数据。
// 执行连续Shape右操作数布尔减，并在changes和statistics非空时记录对应数据。
MyVoxel::VoxelShape cutWithShapeToolImpl(const MyVoxel::VoxelShape& object, const MyVoxel::Shape& tool, const MyVoxel::BooleanOperationOptions& options, MyVoxel::VoxelChangeSet* changes, MyVoxel::BooleanOperationStatistics* statistics)
{
    if (changes)
    {
        changes->clear();
    }

    if (statistics)
    {
        statistics->reset();
    }

    const Clock::time_point totalStart = Clock::now();

    assert(object.transform().isRigidTransform());
    assert(tool.transform().isRigidTransform());
    assert(tool.localBounds().hasVolume());

    const Clock::time_point toolIndexStart = Clock::now();
    const ShapeToolContext context = createShapeToolContext(object, tool);
    const Clock::time_point toolIndexEnd = Clock::now();

    if (statistics)
    {
        statistics->toolIndexMilliseconds = elapsedMilliseconds(toolIndexStart, toolIndexEnd);
        statistics->broadPhaseRootCandidateCount = context.rootCandidateCount;
    }

    const Clock::time_point materialCollectionStart = Clock::now();
    std::size_t existingRootCount = 0;
    const std::vector<MyVoxel::VoxelCellAddress> rootCells = collectRootCellsInRange(object, context.minimumRootIndex, context.maximumRootIndex, &existingRootCount);
    const Clock::time_point materialCollectionEnd = Clock::now();

    if (statistics)
    {
        statistics->materialCollectionMilliseconds = elapsedMilliseconds(materialCollectionStart, materialCollectionEnd);
        statistics->broadPhaseExistingRootCount = static_cast<std::uint64_t>(existingRootCount);
        statistics->inputMaterialCellCount = static_cast<std::uint64_t>(rootCells.size());
    }

    if (rootCells.empty())
    {
        if (statistics)
        {
            statistics->totalMilliseconds = elapsedMilliseconds(totalStart, Clock::now());
        }

        return object;
    }

    const unsigned int workerCount = resolveParallelWorkerCount(options, rootCells.size());
    std::vector<RootCutPlan> plans(rootCells.size());
    const Clock::time_point planningStart = Clock::now();

    buildShapeCutPlans(object, tool, context, rootCells, workerCount, statistics != nullptr, plans);

    const Clock::time_point planningEnd = Clock::now();

    if (statistics)
    {
        statistics->planningMilliseconds = elapsedMilliseconds(planningStart, planningEnd);
        statistics->planningWorkerCount = workerCount;

        for (std::size_t i = 0; i < plans.size(); ++i)
        {
            statistics->accumulate(plans[i].statistics);
        }
    }

    if (!hasShapeCutPlanModification(plans))
    {
        if (statistics)
        {
            statistics->cuttingMilliseconds = statistics->planningMilliseconds;
            statistics->totalMilliseconds = elapsedMilliseconds(totalStart, Clock::now());
        }

        return object;
    }

    MyVoxel::VoxelShape result = object;

    const Clock::time_point detachStart = Clock::now();
    MyVoxel::VoxelNodeForest& resultForest = result.forest();
    const Clock::time_point detachEnd = Clock::now();

    if (statistics)
    {
        statistics->detachMilliseconds = elapsedMilliseconds(detachStart, detachEnd);
    }

    std::vector<RootPlanApplication> applications;
    applications.reserve(rootCells.size());

    std::uint64_t removedRootTreeCount = 0;
    const Clock::time_point rootPreparationStart = Clock::now();

    for (std::size_t i = 0; i < rootCells.size(); ++i)
    {
        const CutPlanNode& rootPlan = plans[i].root;

        if (rootPlan.action == CutPlanAction::Keep)
        {
            continue;
        }

        // 完整删除根节点时直接删除共享指针，不复制对应根树。
        if (rootPlan.action == CutPlanAction::Remove)
        {
            const bool removed = resultForest.eraseRootTree(rootCells[i].index);

            assert(removed);

            if (removed)
            {
                ++removedRootTreeCount;

                if (changes)
                {
                    changes->addModifiedRoot(rootCells[i].index);
                }
            }

            continue;
        }

        assert(rootPlan.action == CutPlanAction::Subdivide);

        // 根树分离必须在主线程完成，工作线程不允许修改根节点映射。
        MyVoxel::VoxelRootTree* editableTree = resultForest.detachRootTree(rootCells[i].index);

        assert(editableTree);

        if (changes)
        {
            changes->addModifiedRoot(rootCells[i].index);
        }

        applications.push_back(RootPlanApplication(*editableTree, plans[i]));
    }

    const Clock::time_point rootPreparationEnd = Clock::now();
    const unsigned int applicationWorkerCount = resolveParallelWorkerCount(options, applications.size());
    const Clock::time_point applicationStart = Clock::now();

    applyShapeCutPlans(applications, applicationWorkerCount);

    const Clock::time_point applicationEnd = Clock::now();

    if (statistics)
    {
        statistics->rootPreparationMilliseconds = elapsedMilliseconds(rootPreparationStart, rootPreparationEnd);
        statistics->planApplicationMilliseconds = elapsedMilliseconds(applicationStart, applicationEnd);
        statistics->planApplicationWorkerCount = applicationWorkerCount;
        statistics->preparedRootTreeCount = static_cast<std::uint64_t>(applications.size());
        statistics->removedRootTreeCount = removedRootTreeCount;
        statistics->cuttingMilliseconds = statistics->planningMilliseconds + statistics->rootPreparationMilliseconds + statistics->planApplicationMilliseconds;
        statistics->totalMilliseconds = elapsedMilliseconds(totalStart, Clock::now());

        assert(statistics->broadPhaseExistingRootCount == statistics->inputMaterialCellCount);
        assert(statistics->classifiedCellCount == statistics->outsideCellCount + statistics->insideCellCount + statistics->intersectingCellCount);
        assert(statistics->pointQueryCount == statistics->centerSampleCount);
        assert(statistics->queryRootCandidateCount == 0);
        assert(statistics->queryExistingRootCount == 0);
        assert(statistics->queryNodeBoundsTestCount == 0);
        assert(statistics->queryVisitedNodeCount == 0);
        assert(statistics->accessorRootCacheHitCount == 0);
        assert(statistics->accessorRootCacheMissCount == 0);
        assert(statistics->accessorPathReuseCount == 0);
        assert(statistics->accessorReusedPathLevelCount == 0);
        assert(statistics->accessorNodeVisitCount == 0);
    }

    return result;
}
}

namespace MyVoxel
{

void BooleanOperationStatistics::reset()
{
    *this = BooleanOperationStatistics();
}

void BooleanOperationStatistics::accumulate(const BooleanOperationStatistics& other)
{
    /// 耗时统计

    totalMilliseconds += other.totalMilliseconds;
    toolIndexMilliseconds += other.toolIndexMilliseconds;
    materialCollectionMilliseconds += other.materialCollectionMilliseconds;
    detachMilliseconds += other.detachMilliseconds;
    planningMilliseconds += other.planningMilliseconds;
    rootPreparationMilliseconds += other.rootPreparationMilliseconds;
    planApplicationMilliseconds += other.planApplicationMilliseconds;
    cuttingMilliseconds += other.cuttingMilliseconds;

    /// 连续Shape并行统计

    planningWorkerCount += other.planningWorkerCount;
    planApplicationWorkerCount += other.planApplicationWorkerCount;
    cutPlanNodeCount += other.cutPlanNodeCount;
    cutPlanGroupCount += other.cutPlanGroupCount;
    preparedRootTreeCount += other.preparedRootTreeCount;
    removedRootTreeCount += other.removedRootTreeCount;

    /// 连续Shape宽相统计

    broadPhaseRootCandidateCount += other.broadPhaseRootCandidateCount;
    broadPhaseExistingRootCount += other.broadPhaseExistingRootCount;

    /// 左操作数递归统计

    toolMaterialCellCount += other.toolMaterialCellCount;
    inputMaterialCellCount += other.inputMaterialCellCount;
    visitedCellCount += other.visitedCellCount;
    emptySkippedCellCount += other.emptySkippedCellCount;
    classifiedCellCount += other.classifiedCellCount;
    outsideCellCount += other.outsideCellCount;
    insideCellCount += other.insideCellCount;
    intersectingCellCount += other.intersectingCellCount;
    centerSampleCount += other.centerSampleCount;
    pointQueryCount += other.pointQueryCount;
    centerRemovedCellCount += other.centerRemovedCellCount;
    removedBranchCount += other.removedBranchCount;
    mergeAttemptCount += other.mergeAttemptCount;
    mergeSuccessCount += other.mergeSuccessCount;

    /// 体素右操作数区域查询统计

    rootBucketVisitCount += other.rootBucketVisitCount;
    materialBoxTestCount += other.materialBoxTestCount;
    queryRootCandidateCount += other.queryRootCandidateCount;
    queryExistingRootCount += other.queryExistingRootCount;
    queryNodeBoundsTestCount += other.queryNodeBoundsTestCount;
    queryVisitedNodeCount += other.queryVisitedNodeCount;

    /// 体素右操作数点访问器统计

    accessorRootCacheHitCount += other.accessorRootCacheHitCount;
    accessorRootCacheMissCount += other.accessorRootCacheMissCount;
    accessorPathReuseCount += other.accessorPathReuseCount;
    accessorReusedPathLevelCount += other.accessorReusedPathLevelCount;
    accessorNodeVisitCount += other.accessorNodeVisitCount;
}

VoxelShape cut(const VoxelShape& object, const VoxelShape& tool)
{
    return cutWithVoxelToolImpl(object, tool, nullptr);
}

VoxelShape cut(const VoxelShape& object, const VoxelShape& tool, VoxelChangeSet& changes)
{
    return cutWithVoxelToolImpl(object, tool, &changes);
}

VoxelShape cut(const VoxelShape& object, const VoxelShape& tool, BooleanOperationStatistics& statistics)
{
    return cutWithVoxelToolStatisticsImpl(object, tool, statistics, nullptr);
}

VoxelShape cut(const VoxelShape& object, const VoxelShape& tool, VoxelChangeSet& changes, BooleanOperationStatistics& statistics)
{
    return cutWithVoxelToolStatisticsImpl(object, tool, statistics, &changes);
}

VoxelShape cut(const VoxelShape& object, const Shape& tool)
{
    return cutWithShapeToolImpl(object, tool, BooleanOperationOptions(), nullptr, nullptr);
}

VoxelShape cut(const VoxelShape& object, const Shape& tool, VoxelChangeSet& changes)
{
    return cutWithShapeToolImpl(object, tool, BooleanOperationOptions(), &changes, nullptr);
}

VoxelShape cut(const VoxelShape& object, const Shape& tool, BooleanOperationStatistics& statistics)
{
    return cutWithShapeToolImpl(object, tool, BooleanOperationOptions(), nullptr, &statistics);
}

VoxelShape cut(const VoxelShape& object, const Shape& tool, VoxelChangeSet& changes, BooleanOperationStatistics& statistics)
{
    return cutWithShapeToolImpl(object, tool, BooleanOperationOptions(), &changes, &statistics);
}

VoxelShape cut(const VoxelShape& object, const Shape& tool, const BooleanOperationOptions& options)
{
    return cutWithShapeToolImpl(object, tool, options, nullptr, nullptr);
}

VoxelShape cut(const VoxelShape& object, const Shape& tool, const BooleanOperationOptions& options, VoxelChangeSet& changes)
{
    return cutWithShapeToolImpl(object, tool, options, &changes, nullptr);
}

VoxelShape cut(const VoxelShape& object, const Shape& tool, const BooleanOperationOptions& options, BooleanOperationStatistics& statistics)
{
    return cutWithShapeToolImpl(object, tool, options, nullptr, &statistics);
}

VoxelShape cut(const VoxelShape& object, const Shape& tool, const BooleanOperationOptions& options, VoxelChangeSet& changes, BooleanOperationStatistics& statistics)
{
    return cutWithShapeToolImpl(object, tool, options, &changes, &statistics);
}
}