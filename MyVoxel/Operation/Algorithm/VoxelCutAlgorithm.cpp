#include "BooleanCutAlgorithm.h"

#include <cassert>
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

namespace
{

// 保存工件局部坐标到体素刀具局部坐标的固定变换数据。
struct CellTransformContext
{
    MyMath::Matrix4 objectToTool; // 工件局部坐标到刀具局部坐标的变换。
};

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

// 将连续坐标转换为指定体素网格索引。
MyVoxel::VoxelIndex coordinateIndex(double coordinate, double origin, double edgeLength)
{
    assert(std::isfinite(coordinate));
    assert(std::isfinite(origin));
    assert(std::isfinite(edgeLength) && edgeLength > 0.0);

    const double indexValue = std::floor((coordinate - origin) / edgeLength);
    const double minimumValue = static_cast<double>((std::numeric_limits<MyVoxel::VoxelIndex>::min)());
    const double maximumValue = static_cast<double>((std::numeric_limits<MyVoxel::VoxelIndex>::max)());

    assert(indexValue >= minimumValue && indexValue <= maximumValue);
    return static_cast<MyVoxel::VoxelIndex>(indexValue);
}

// 返回指定点在VoxelGrid目标层级对应的体素地址。
MyVoxel::VoxelCellAddress gridAddressAtPoint(const MyVoxel::VoxelGrid& grid, const MyMath::Vector3& point, MyVoxel::VoxelLevel level)
{
    assert(grid.isValid());
    assert(point.isFinite());
    assert(level <= grid.maximumLevel());

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

// 将体素森林区域关系转换为布尔运算内部关系。
MyVoxel::Operation::Algorithm::CellRelation convertRelation(MyVoxel::VoxelRegionRelation relation)
{
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

// 返回工件体素与体素刀具材料区域之间的保守关系。
MyVoxel::Operation::Algorithm::CellRelation classifyObjectCellWithVoxelTool(const MyVoxel::VoxelShape& object, const CellTransformContext& context, const MyVoxel::VoxelForestQuery& toolQuery, const MyVoxel::VoxelCellAddress& address)
{
    return convertRelation(toolQuery.classify(transformedCellBounds(object, context, address)));
}

// 检查工件体素中心映射后是否位于体素刀具材料中。
bool voxelToolContainsObjectCellCenter(const MyVoxel::VoxelShape& object, const MyVoxel::VoxelShape& tool, const CellTransformContext& context, MyVoxel::VoxelForestConstAccessor& toolAccessor, const MyVoxel::VoxelCellAddress& address)
{
    const MyMath::Vector3 toolPoint = context.objectToTool.transformPoint(MyVoxel::Operation::Algorithm::cellCenter(object, address));
    const MyVoxel::VoxelCellAddress toolAddress = gridAddressAtPoint(tool.grid(), toolPoint, tool.grid().maximumLevel());
    return toolAccessor.state(toolAddress) == MyVoxel::VoxelState::Material;
}

// 递归对指定材料节点执行体素刀具布尔减。
MyVoxel::Operation::Algorithm::CutCellResult cutMaterialCellWithVoxelTool(MyVoxel::VoxelTreeEditor editor, const MyVoxel::VoxelShape& object, const MyVoxel::VoxelShape& tool, const CellTransformContext& context, const MyVoxel::VoxelForestQuery& toolQuery, MyVoxel::VoxelForestConstAccessor& toolAccessor, const MyVoxel::VoxelCellAddress& address, MyVoxel::VoxelLevel targetLevel)
{
    if (address.level == targetLevel)
    {
        if (voxelToolContainsObjectCellCenter(object, tool, context, toolAccessor, address))
        {
            editor.setState(MyVoxel::VoxelState::Empty);
            return MyVoxel::Operation::Algorithm::CutCellResult(MyVoxel::VoxelState::Empty, true);
        }

        return MyVoxel::Operation::Algorithm::CutCellResult(MyVoxel::VoxelState::Material, false);
    }

    const MyVoxel::Operation::Algorithm::CellRelation relation = classifyObjectCellWithVoxelTool(object, context, toolQuery, address);

    if (relation == MyVoxel::Operation::Algorithm::CellRelation::Outside)
    {
        return MyVoxel::Operation::Algorithm::CutCellResult(MyVoxel::VoxelState::Material, false);
    }

    if (relation == MyVoxel::Operation::Algorithm::CellRelation::Inside)
    {
        editor.setState(MyVoxel::VoxelState::Empty);
        return MyVoxel::Operation::Algorithm::CutCellResult(MyVoxel::VoxelState::Empty, true);
    }

    assert(address.level < targetLevel);

    editor.split();

    bool changed = false;

    for (int cornerIndex = 0; cornerIndex < MyVoxel::VoxelCornerCount; ++cornerIndex)
    {
        const MyVoxel::VoxelCorner corner = static_cast<MyVoxel::VoxelCorner>(cornerIndex);
        const MyVoxel::VoxelCellAddress childAddress = MyVoxel::childCellAddress(address, corner);
        const MyVoxel::Operation::Algorithm::CutCellResult childResult = cutMaterialCellWithVoxelTool(editor.child(corner), object, tool, context, toolQuery, toolAccessor, childAddress, targetLevel);

        changed = childResult.changed || changed;
    }

    return MyVoxel::Operation::Algorithm::CutCellResult(editor.merge(), changed);
}

}

namespace MyVoxel
{
namespace Operation
{
namespace Algorithm
{

VoxelShape cutWithVoxelTool(const VoxelShape& object, const VoxelShape& tool, VoxelChangeSet* changes)
{
    if (changes)
    {
        changes->clear();
    }

    assert(object.grid().isValid());
    assert(tool.grid().isValid());
    assert(object.transform().isRigidTransform());
    assert(tool.transform().isRigidTransform());

    if (object.isEmpty() || tool.isEmpty())
    {
        return object;
    }

    const std::vector<VoxelCellAddress> materialCells = collectMaterialCells(object);

    if (materialCells.empty())
    {
        return object;
    }

    const VoxelForestQuery toolQuery(tool.forest(), tool.grid());
    VoxelForestConstAccessor toolAccessor(tool.forest());
    const CellTransformContext context = createCellTransformContext(object, tool);
    VoxelShape result = object;
    VoxelForest& resultForest = result.editForest();
    const VoxelLevel targetLevel = object.grid().maximumLevel();

    for (std::size_t materialIndex = 0; materialIndex < materialCells.size(); ++materialIndex)
    {
        const VoxelCellAddress& address = materialCells[materialIndex];

        assert(address.level <= targetLevel);

        VoxelTreeEditor editor(resultForest, address);
        const CutCellResult cellResult = cutMaterialCellWithVoxelTool(editor, object, tool, context, toolQuery, toolAccessor, address, targetLevel);

        if (cellResult.changed && changes)
        {
            changes->addModifiedRoot(rootCellIndex(address));
        }

        if (cellResult.state == VoxelState::Empty)
        {
            resultForest.pruneEmptyBranch(address);
        }
    }

    return result;
}

}
}
}