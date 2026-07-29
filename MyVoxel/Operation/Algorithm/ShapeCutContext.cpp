#include "ShapeCutContext.h"

#include <cassert>
#include <cmath>
#include <cstddef>

#include "BooleanAlgorithmCommon.h"

namespace
{

const double BroadPhaseBoundsToleranceScale = 1.0e-9; // 宽相范围按第0层边长的十亿分之一向外扩展。
const double HalfScale = 0.5; // 体素半边长为完整边长的一半。
const double ChildCenterOffsetScale = 0.25; // 子节点中心相对父节点中心偏移父节点边长的四分之一。

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

}

namespace MyVoxel
{
namespace Operation
{
namespace Algorithm
{

ShapeToolLevelContext::ShapeToolLevelContext()
    : halfExtentX(0.0)
    , halfExtentY(0.0)
    , halfExtentZ(0.0)
{
}

ShapeToolContext createShapeToolContext(const VoxelShape& object, const Geometry::ShapeInstance& tool)
{
    assert(object.transform().isRigidTransform());
    assert(tool.isValid());
    assert(tool.localToWorld().isRigidTransform());

    MyMath::Matrix4 worldToObject;
    const bool objectInverted = object.transform().inverted(worldToObject);

    assert(objectInverted);

    ShapeToolContext context;
    context.objectToTool = tool.worldToLocal() * object.transform();

    const MyMath::Matrix4 toolToObject = worldToObject * tool.localToWorld();
    const Bounds3 toolObjectBounds = tool.shape().localBounds().transformed(toolToObject);
    const double baseEdgeLength = cellEdgeLength(object.grid(), BaseVoxelLevel);
    const double tolerance = baseEdgeLength * BroadPhaseBoundsToleranceScale;
    const Bounds3 expandedBounds(
        MyMath::Vector3(toolObjectBounds.minimum().x() - tolerance,
                        toolObjectBounds.minimum().y() - tolerance,
                        toolObjectBounds.minimum().z() - tolerance),
        MyMath::Vector3(toolObjectBounds.maximum().x() + tolerance,
                        toolObjectBounds.maximum().y() + tolerance,
                        toolObjectBounds.maximum().z() + tolerance));

    const VoxelCellRange rootRange = object.grid().cellRange(expandedBounds, BaseVoxelLevel);

    context.minimumRootIndex = rootRange.minimum;
    context.maximumRootIndex = rootRange.maximum;

    const MyMath::Vector3 transformedAxisX = context.objectToTool.transformVector(MyMath::Vector3::unitX());
    const MyMath::Vector3 transformedAxisY = context.objectToTool.transformVector(MyMath::Vector3::unitY());
    const MyMath::Vector3 transformedAxisZ = context.objectToTool.transformVector(MyMath::Vector3::unitZ());

    const double extentScaleX = std::fabs(transformedAxisX.x()) + std::fabs(transformedAxisY.x()) + std::fabs(transformedAxisZ.x());
    const double extentScaleY = std::fabs(transformedAxisX.y()) + std::fabs(transformedAxisY.y()) + std::fabs(transformedAxisZ.y());
    const double extentScaleZ = std::fabs(transformedAxisX.z()) + std::fabs(transformedAxisY.z()) + std::fabs(transformedAxisZ.z());
    const int maximumLevel = static_cast<int>(object.grid().maximumLevel());

    context.levels.resize(static_cast<std::size_t>(maximumLevel + 1));

    for (int levelIndex = 0; levelIndex <= maximumLevel; ++levelIndex)
    {
        const VoxelLevel level = static_cast<VoxelLevel>(levelIndex);
        const double edgeLength = cellEdgeLength(object.grid(), level);
        const double halfEdgeLength = edgeLength * HalfScale;
        ShapeToolLevelContext& levelContext = context.levels[static_cast<std::size_t>(levelIndex)];

        levelContext.halfExtentX = halfEdgeLength * extentScaleX;
        levelContext.halfExtentY = halfEdgeLength * extentScaleY;
        levelContext.halfExtentZ = halfEdgeLength * extentScaleZ;

        if (levelIndex >= maximumLevel)
        {
            continue;
        }

        const double childCenterOffset = edgeLength * ChildCenterOffsetScale;

        for (int cornerIndex = 0; cornerIndex < VoxelCornerCount; ++cornerIndex)
        {
            const VoxelCorner corner = static_cast<VoxelCorner>(cornerIndex);
            const double signX = cornerUsesMaximumX(corner) ? 1.0 : -1.0;
            const double signY = cornerUsesMaximumY(corner) ? 1.0 : -1.0;
            const double signZ = cornerUsesMaximumZ(corner) ? 1.0 : -1.0;

            levelContext.childCenterOffsets[cornerIndex] =
                transformedAxisX * (childCenterOffset * signX) +
                transformedAxisY * (childCenterOffset * signY) +
                transformedAxisZ * (childCenterOffset * signZ);
        }
    }

    return context;
}

Bounds3 shapeToolCellBounds(const ShapeToolContext& context, VoxelLevel level, const MyMath::Vector3& centerToolSpace)
{
    assert(static_cast<std::size_t>(level) < context.levels.size());

    const ShapeToolLevelContext& levelContext = context.levels[static_cast<std::size_t>(level)];

    return Bounds3(
        MyMath::Vector3(centerToolSpace.x() - levelContext.halfExtentX,
                        centerToolSpace.y() - levelContext.halfExtentY,
                        centerToolSpace.z() - levelContext.halfExtentZ),
        MyMath::Vector3(centerToolSpace.x() + levelContext.halfExtentX,
                        centerToolSpace.y() + levelContext.halfExtentY,
                        centerToolSpace.z() + levelContext.halfExtentZ));
}

}
}
}