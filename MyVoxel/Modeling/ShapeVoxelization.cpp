#include "ShapeVoxelization.h"

#include <cstddef>
#include <cstdint>
#include <limits>

#include "MyMath/Vector3.h"

#include "MyVoxel/Base/Bounds3.h"
#include "MyVoxel/Core/Tree/VoxelTreeEditor.h"
#include "MyVoxel/Foundation/Diagnostic.h"
#include "MyVoxel/Geometry/ShapeRelation.h"

namespace
{

const double CellCenterScale = 0.5; // 体素中心位于包围盒两个端点的中间位置。
const unsigned int MaskLeafCoveredLevelCount = 2; // 一个64位掩码叶块固定覆盖当前节点下面两级，共4×4×4个最高层体素。

// 检查当前层是否适合直接使用覆盖下面两级的掩码叶块。
bool shouldUseMaskLeaf(const MyVoxel::VoxelCellAddress& address, MyVoxel::VoxelLevel maximumLevel)
{
    const unsigned int currentLevelValue = static_cast<unsigned int>(address.level);
    const unsigned int maximumLevelValue = static_cast<unsigned int>(maximumLevel);

    if (address.level == MyVoxel::BaseVoxelLevel || currentLevelValue > maximumLevelValue)
    {
        return false;
    }

    return maximumLevelValue - currentLevelValue == MaskLeafCoveredLevelCount;
}

// 执行无符号64位饱和乘法，溢出时返回最大值。
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

// 返回指定体素包围盒的中心点。
MyMath::Vector3 boundsCenter(const MyVoxel::Bounds3& bounds)
{
    const MyMath::Vector3& minimum = bounds.minimum();
    const MyMath::Vector3& maximum = bounds.maximum();

    return MyMath::Vector3(
        (minimum.x() + maximum.x()) * CellCenterScale,
        (minimum.y() + maximum.y()) * CellCenterScale,
        (minimum.z() + maximum.z()) * CellCenterScale);
}

// 按已经获得的连续几何关系递归生成当前体素节点。
MyVoxel::VoxelState voxelizeCell(MyVoxel::VoxelTreeEditor editor,
                                 const MyVoxel::Geometry::Shape& shape,
                                 const MyVoxel::VoxelGrid& grid,
                                 const MyVoxel::VoxelCellAddress& address,
                                 MyVoxel::Geometry::ShapeRelation relation,
                                 MyVoxel::Modeling::VoxelizationStatistics& statistics)
{
    ++statistics.visitedCellCount;

    if (relation == MyVoxel::Geometry::ShapeRelation::Outside)
    {
        ++statistics.outsideCellCount;
        editor.setState(MyVoxel::VoxelState::Empty);
        return MyVoxel::VoxelState::Empty;
    }

    if (relation == MyVoxel::Geometry::ShapeRelation::Inside)
    {
        ++statistics.insideCellCount;
        editor.setState(MyVoxel::VoxelState::Material);
        return MyVoxel::VoxelState::Material;
    }

    MYVOXEL_ASSERT_MESSAGE(relation == MyVoxel::Geometry::ShapeRelation::Intersecting, "Shape voxelization requires a valid ShapeRelation.");

    ++statistics.intersectingCellCount;

    const MyVoxel::Bounds3 cellBounds = grid.cellBounds(address);

    if (address.level == grid.maximumLevel())
    {
        ++statistics.centerSampleCount;

        const MyVoxel::VoxelState sampledState =
            shape.containsLocalPoint(boundsCenter(cellBounds)) ?
            MyVoxel::VoxelState::Material :
            MyVoxel::VoxelState::Empty;

        editor.setState(sampledState);
        return sampledState;
    }

    MYVOXEL_ASSERT_MESSAGE(address.level < grid.maximumLevel(), "Shape voxelization address level exceeds the VoxelGrid maximum level.");

    bool subdivisionCreated = false;

    if (shouldUseMaskLeaf(address, grid.maximumLevel()))
    {
        subdivisionCreated = editor.makeMaskLeaf();
    }
    else
    {
        subdivisionCreated = editor.split();
    }

    if (subdivisionCreated)
    {
        ++statistics.splitCount;
    }

    MYVOXEL_ASSERT_MESSAGE(editor.canAccessChildren(), "Shape voxelization failed to create an accessible child structure.");

    for (std::size_t cornerIndex = 0; cornerIndex < MyVoxel::VoxelCornerCount; ++cornerIndex)
    {
        const MyVoxel::VoxelCorner corner = static_cast<MyVoxel::VoxelCorner>(cornerIndex);
        const MyVoxel::VoxelCellAddress childAddress = MyVoxel::childCellAddress(address, corner);
        const MyVoxel::Bounds3 childBounds = grid.cellBounds(childAddress);
        const MyVoxel::Geometry::ShapeRelation childRelation = shape.classifyLocalBounds(childBounds);

        voxelizeCell(editor.child(corner), shape, grid, childAddress, childRelation, statistics);
    }

    const MyVoxel::VoxelState mergedState = editor.merge();

    if (mergedState != MyVoxel::VoxelState::Subdivided)
    {
        ++statistics.mergeSuccessCount;
    }

    return mergedState;
}



// 执行连续Shape到VoxelShape的统一局部体素化。
MyVoxel::VoxelShape voxelizeShapeImpl(const MyVoxel::Geometry::Shape& shape,
                                      const MyVoxel::VoxelGrid& grid,
                                      MyVoxel::Modeling::VoxelizationStatistics& statistics)
{
    MYVOXEL_ASSERT_MESSAGE(shape.isValid(), "Shape voxelization requires a valid Geometry::Shape.");
    MYVOXEL_ASSERT_MESSAGE(grid.isValid(), "Shape voxelization requires a valid VoxelGrid.");

    statistics.reset();

    const MyVoxel::Bounds3 shapeBounds = shape.localBounds();

    MYVOXEL_ASSERT_MESSAGE(shapeBounds.isValid(), "Shape voxelization requires valid local bounds.");
    MYVOXEL_ASSERT_MESSAGE(shapeBounds.hasVolume(), "Shape voxelization requires local bounds with volume.");

    MyVoxel::VoxelShape result(grid);
    MyVoxel::VoxelForest& forest = result.editForest();

    const MyVoxel::VoxelCellRange rootRange = grid.cellRange(shapeBounds, MyVoxel::BaseVoxelLevel);
    const std::uint64_t xyCandidateCount = saturatedMultiply(rootRange.countX(), rootRange.countY());

    statistics.rootCandidateCount = saturatedMultiply(xyCandidateCount, rootRange.countZ());

    for (std::int64_t z = static_cast<std::int64_t>(rootRange.minimum.z); z <= static_cast<std::int64_t>(rootRange.maximum.z); ++z)
    {
        for (std::int64_t y = static_cast<std::int64_t>(rootRange.minimum.y); y <= static_cast<std::int64_t>(rootRange.maximum.y); ++y)
        {
            for (std::int64_t x = static_cast<std::int64_t>(rootRange.minimum.x); x <= static_cast<std::int64_t>(rootRange.maximum.x); ++x)
            {
                const MyVoxel::VoxelCellIndex rootIndex(static_cast<MyVoxel::VoxelIndex>(x),
                                                        static_cast<MyVoxel::VoxelIndex>(y),
                                                        static_cast<MyVoxel::VoxelIndex>(z));

                const MyVoxel::VoxelCellAddress rootAddress(rootIndex, MyVoxel::BaseVoxelLevel);
                const MyVoxel::Bounds3 rootBounds = grid.cellBounds(rootAddress);
                const MyVoxel::Geometry::ShapeRelation rootRelation = shape.classifyLocalBounds(rootBounds);

                if (rootRelation == MyVoxel::Geometry::ShapeRelation::Outside)
                {
                    ++statistics.visitedCellCount;
                    ++statistics.outsideCellCount;
                    continue;
                }

                const bool created = forest.setState(rootAddress, MyVoxel::VoxelState::Material);

                MYVOXEL_ASSERT_MESSAGE(created, "Shape voxelization candidate root must initially be absent.");

                MyVoxel::VoxelTreeEditor editor(forest, rootAddress);
                const MyVoxel::VoxelState rootState = voxelizeCell(editor, shape, grid, rootAddress, rootRelation, statistics);

                if (rootState == MyVoxel::VoxelState::Empty)
                {
                    const bool erased = forest.setState(rootAddress, MyVoxel::VoxelState::Empty);
                    MYVOXEL_ASSERT_MESSAGE(erased, "An empty voxelized root tree must be removable.");
                }
                else
                {
                    ++statistics.createdRootCount;
                }
            }
        }
    }

    MYVOXEL_ASSERT_MESSAGE(result.rootCount() == static_cast<std::size_t>(statistics.createdRootCount), "Voxelization root statistics must match the generated VoxelShape.");
    return result;
}

// 执行ShapeInstance到VoxelShape的统一体素化，并保留实例局部到世界变换。
MyVoxel::VoxelShape voxelizeInstanceImpl(const MyVoxel::Geometry::ShapeInstance& instance,
                                         const MyVoxel::VoxelGrid& grid,
                                         MyVoxel::Modeling::VoxelizationStatistics& statistics)
{
    MYVOXEL_ASSERT_MESSAGE(instance.isValid(), "ShapeInstance voxelization requires a valid instance.");

    MyVoxel::VoxelShape result = voxelizeShapeImpl(instance.shape(), grid, statistics);
    result.setTransform(instance.localToWorld());
    return result;
}






}

namespace MyVoxel
{
namespace Modeling
{

VoxelizationStatistics::VoxelizationStatistics()
    : rootCandidateCount(0)
    , createdRootCount(0)
    , visitedCellCount(0)
    , outsideCellCount(0)
    , insideCellCount(0)
    , intersectingCellCount(0)
    , centerSampleCount(0)
    , splitCount(0)
    , mergeSuccessCount(0)
{
}

void VoxelizationStatistics::reset()
{
    rootCandidateCount = 0;
    createdRootCount = 0;
    visitedCellCount = 0;
    outsideCellCount = 0;
    insideCellCount = 0;
    intersectingCellCount = 0;
    centerSampleCount = 0;
    splitCount = 0;
    mergeSuccessCount = 0;
}

void VoxelizationStatistics::accumulate(const VoxelizationStatistics& other)
{
    rootCandidateCount += other.rootCandidateCount;
    createdRootCount += other.createdRootCount;
    visitedCellCount += other.visitedCellCount;
    outsideCellCount += other.outsideCellCount;
    insideCellCount += other.insideCellCount;
    intersectingCellCount += other.intersectingCellCount;
    centerSampleCount += other.centerSampleCount;
    splitCount += other.splitCount;
    mergeSuccessCount += other.mergeSuccessCount;
}

/// 局部Shape体素化

VoxelShape voxelize(const Geometry::Shape& shape, const VoxelGrid& grid)
{
    VoxelizationStatistics statistics;
    return voxelizeShapeImpl(shape, grid, statistics);
}

VoxelShape voxelize(const Geometry::Shape& shape, const VoxelGrid& grid, VoxelizationStatistics& statistics)
{
    return voxelizeShapeImpl(shape, grid, statistics);
}

VoxelShape voxelize(const Geometry::Shape& shape, double baseVoxelEdgeLength, VoxelLevel maximumLevel)
{
    return voxelize(shape, VoxelGrid(baseVoxelEdgeLength, maximumLevel));
}

VoxelShape voxelize(const Geometry::Shape& shape, double baseVoxelEdgeLength, VoxelLevel maximumLevel, VoxelizationStatistics& statistics)
{
    return voxelize(shape, VoxelGrid(baseVoxelEdgeLength, maximumLevel), statistics);
}

/// Shape实例体素化

VoxelShape voxelize(const Geometry::ShapeInstance& instance, const VoxelGrid& grid)
{
    VoxelizationStatistics statistics;
    return voxelizeInstanceImpl(instance, grid, statistics);
}

VoxelShape voxelize(const Geometry::ShapeInstance& instance, const VoxelGrid& grid, VoxelizationStatistics& statistics)
{
    return voxelizeInstanceImpl(instance, grid, statistics);
}

VoxelShape voxelize(const Geometry::ShapeInstance& instance, double baseVoxelEdgeLength, VoxelLevel maximumLevel)
{
    return voxelize(instance, VoxelGrid(baseVoxelEdgeLength, maximumLevel));
}

VoxelShape voxelize(const Geometry::ShapeInstance& instance, double baseVoxelEdgeLength, VoxelLevel maximumLevel, VoxelizationStatistics& statistics)
{
    return voxelize(instance, VoxelGrid(baseVoxelEdgeLength, maximumLevel), statistics);
}

}
}