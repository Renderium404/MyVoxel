#include "VoxelPackedForestQuery.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include "MyMath/Vector3.h"

namespace
{

const double MiddleScale = 0.5; // 子节点分割面位于父节点最小点和最大点的中间位置。

// 判断角点是否位于X轴最大侧。
bool cornerUsesMaximumX(MyVoxel::VoxelCorner corner)
{
    return (static_cast<unsigned int>(corner) & 1U) != 0U;
}

// 判断角点是否位于Y轴最大侧。
bool cornerUsesMaximumY(MyVoxel::VoxelCorner corner)
{
    return (static_cast<unsigned int>(corner) & 2U) != 0U;
}

// 判断角点是否位于Z轴最大侧。
bool cornerUsesMaximumZ(MyVoxel::VoxelCorner corner)
{
    return (static_cast<unsigned int>(corner) & 4U) != 0U;
}
// 将连续坐标按旧版零原点规则转换为第0层体素索引。
MyVoxel::VoxelIndex legacyCoordinateIndex(double coordinate, double edgeLength)
{
    assert(std::isfinite(coordinate));
    assert(std::isfinite(edgeLength) && edgeLength > 0.0);

    const double indexValue = std::floor(coordinate / edgeLength);
    const double minimumValue = static_cast<double>((std::numeric_limits<MyVoxel::VoxelIndex>::min)());
    const double maximumValue = static_cast<double>((std::numeric_limits<MyVoxel::VoxelIndex>::max)());

    assert(indexValue >= minimumValue && indexValue <= maximumValue);
    return static_cast<MyVoxel::VoxelIndex>(indexValue);
}
}

namespace MyVoxel
{

VoxelPackedForestQuery::VoxelPackedForestQuery(
    const VoxelPackedForest& forest,
    const VoxelGrid& grid,
    VoxelForestQueryCoordinateMode coordinateMode)
    : m_forest(&forest)
    , m_grid(&grid)
    , m_coordinateMode(coordinateMode)
    , m_baseVoxelEdgeLength(0.0)
{
    assert(grid.isValid());

    const VoxelCellAddress zeroRootAddress(VoxelCellIndex(0, 0, 0), BaseVoxelLevel);
    const Bounds3 zeroRootBounds = grid.cellBounds(zeroRootAddress);

    m_baseVoxelEdgeLength = zeroRootBounds.maximum().x() - zeroRootBounds.minimum().x();

    assert(std::isfinite(m_baseVoxelEdgeLength) && m_baseVoxelEdgeLength > 0.0);
}

VoxelRegionRelation VoxelPackedForestQuery::classify(const Bounds3& bounds) const
{
    return classifyImpl(bounds, nullptr);
}

VoxelRegionRelation VoxelPackedForestQuery::classify(const Bounds3& bounds, VoxelForestQueryStatistics& statistics) const
{
    statistics.reset();
    return classifyImpl(bounds, &statistics);
}

VoxelRegionRelation VoxelPackedForestQuery::classifyImpl(const Bounds3& bounds, VoxelForestQueryStatistics* statistics) const
{
    assert(m_forest);
    assert(m_grid);
    assert(m_grid->isValid());
    assert(bounds.isValid());

    const VoxelCellRange rootRange = queryRootRange(bounds);
    bool possibleIntersection = false;

    for (std::int64_t z = static_cast<std::int64_t>(rootRange.minimum.z); z <= static_cast<std::int64_t>(rootRange.maximum.z); ++z)
    {
        for (std::int64_t y = static_cast<std::int64_t>(rootRange.minimum.y); y <= static_cast<std::int64_t>(rootRange.maximum.y); ++y)
        {
            for (std::int64_t x = static_cast<std::int64_t>(rootRange.minimum.x); x <= static_cast<std::int64_t>(rootRange.maximum.x); ++x)
            {
                if (statistics)
                {
                    ++statistics->rootCandidateCount;
                }

                const VoxelCellIndex rootIndex(
                    static_cast<VoxelIndex>(x),
                    static_cast<VoxelIndex>(y),
                    static_cast<VoxelIndex>(z));

                const VoxelPackedForest::RootMap::const_iterator iterator = m_forest->m_roots.find(rootIndex);

                if (iterator == m_forest->m_roots.end())
                {
                    continue;
                }

                assert(iterator->second);

                if (statistics)
                {
                    ++statistics->existingRootCount;
                }

                const Bounds3 rootBounds = queryRootBounds(rootIndex);

                if (!intersects(bounds, rootBounds))
                {
                    continue;
                }

                const VoxelPackedTreeConstCursor cursor(*iterator->second);
                const VoxelRegionRelation relation = classifyNode(bounds, rootBounds, cursor, statistics);

                if (relation == VoxelRegionRelation::Inside)
                {
                    return VoxelRegionRelation::Inside;
                }

                if (relation == VoxelRegionRelation::Intersecting)
                {
                    possibleIntersection = true;
                }
            }
        }
    }

    return possibleIntersection ? VoxelRegionRelation::Intersecting : VoxelRegionRelation::Outside;
}

VoxelRegionRelation VoxelPackedForestQuery::classifyNode(
    const Bounds3& bounds,
    const Bounds3& nodeBounds,
    const VoxelPackedTreeConstCursor& cursor,
    VoxelForestQueryStatistics* statistics) const
{
    assert(bounds.isValid());
    assert(nodeBounds.isValid());

    if (statistics)
    {
        ++statistics->nodeBoundsTestCount;
    }

    if (!intersects(bounds, nodeBounds))
    {
        return VoxelRegionRelation::Outside;
    }

    if (statistics)
    {
        ++statistics->visitedNodeCount;
    }

    const VoxelState state = cursor.state();

    if (state == VoxelState::Empty)
    {
        if (statistics)
        {
            ++statistics->emptyNodeCount;
        }

        return VoxelRegionRelation::Outside;
    }

    if (state == VoxelState::Material)
    {
        if (statistics)
        {
            ++statistics->materialNodeCount;
        }

        return contains(nodeBounds, bounds) ? VoxelRegionRelation::Inside : VoxelRegionRelation::Intersecting;
    }

    assert(state == VoxelState::Subdivided);

    if (statistics)
    {
        ++statistics->subdividedNodeCount;
    }

    const MyMath::Vector3& nodeMinimum = nodeBounds.minimum();
    const MyMath::Vector3& nodeMaximum = nodeBounds.maximum();
    const MyMath::Vector3& queryMinimum = bounds.minimum();
    const MyMath::Vector3& queryMaximum = bounds.maximum();

    const double middleX = (nodeMinimum.x() + nodeMaximum.x()) * MiddleScale;
    const double middleY = (nodeMinimum.y() + nodeMaximum.y()) * MiddleScale;
    const double middleZ = (nodeMinimum.z() + nodeMaximum.z()) * MiddleScale;

    const int minimumXSide = queryMinimum.x() <= middleX ? 0 : 1;
    const int maximumXSide = queryMaximum.x() >= middleX ? 1 : 0;
    const int minimumYSide = queryMinimum.y() <= middleY ? 0 : 1;
    const int maximumYSide = queryMaximum.y() >= middleY ? 1 : 0;
    const int minimumZSide = queryMinimum.z() <= middleZ ? 0 : 1;
    const int maximumZSide = queryMaximum.z() >= middleZ ? 1 : 0;

    bool possibleIntersection = false;

    for (int zSide = minimumZSide; zSide <= maximumZSide; ++zSide)
    {
        for (int ySide = minimumYSide; ySide <= maximumYSide; ++ySide)
        {
            for (int xSide = minimumXSide; xSide <= maximumXSide; ++xSide)
            {
                const unsigned int cornerValue =
                    static_cast<unsigned int>(
                        xSide |
                        (ySide << 1) |
                        (zSide << 2));

                const VoxelCorner corner = static_cast<VoxelCorner>(cornerValue);
                const Bounds3 currentChildBounds = childBounds(nodeBounds, corner);
                const VoxelRegionRelation relation =
                    classifyNode(
                        bounds,
                        currentChildBounds,
                        cursor.child(corner),
                        statistics);

                if (relation == VoxelRegionRelation::Inside)
                {
                    return VoxelRegionRelation::Inside;
                }

                if (relation == VoxelRegionRelation::Intersecting)
                {
                    possibleIntersection = true;
                }
            }
        }
    }

    return possibleIntersection ? VoxelRegionRelation::Intersecting : VoxelRegionRelation::Outside;
}

VoxelCellRange VoxelPackedForestQuery::queryRootRange(const Bounds3& bounds) const
{
    assert(m_grid);
    assert(bounds.isValid());

    VoxelCellRange range = m_grid->cellRange(bounds, BaseVoxelLevel);

    if (m_coordinateMode == VoxelForestQueryCoordinateMode::GridAware)
    {
        return range;
    }

    assert(m_coordinateMode == VoxelForestQueryCoordinateMode::LegacyZeroOrigin);

    const MyMath::Vector3& minimum = bounds.minimum();
    const MyMath::Vector3& maximum = bounds.maximum();

    range.minimum = VoxelCellIndex(
        legacyCoordinateIndex(minimum.x(), m_baseVoxelEdgeLength),
        legacyCoordinateIndex(minimum.y(), m_baseVoxelEdgeLength),
        legacyCoordinateIndex(minimum.z(), m_baseVoxelEdgeLength));

    range.maximum = VoxelCellIndex(
        legacyCoordinateIndex(maximum.x(), m_baseVoxelEdgeLength),
        legacyCoordinateIndex(maximum.y(), m_baseVoxelEdgeLength),
        legacyCoordinateIndex(maximum.z(), m_baseVoxelEdgeLength));

    return range;
}

Bounds3 VoxelPackedForestQuery::queryRootBounds(const VoxelCellIndex& rootIndex) const
{
    assert(m_grid);

    if (m_coordinateMode == VoxelForestQueryCoordinateMode::GridAware)
    {
        return m_grid->cellBounds(VoxelCellAddress(rootIndex, BaseVoxelLevel));
    }

    assert(m_coordinateMode == VoxelForestQueryCoordinateMode::LegacyZeroOrigin);

    const double minimumX = static_cast<double>(rootIndex.x) * m_baseVoxelEdgeLength;
    const double minimumY = static_cast<double>(rootIndex.y) * m_baseVoxelEdgeLength;
    const double minimumZ = static_cast<double>(rootIndex.z) * m_baseVoxelEdgeLength;

    return Bounds3(
        MyMath::Vector3(minimumX, minimumY, minimumZ),
        MyMath::Vector3(
            minimumX + m_baseVoxelEdgeLength,
            minimumY + m_baseVoxelEdgeLength,
            minimumZ + m_baseVoxelEdgeLength));
}



bool VoxelPackedForestQuery::intersects(const Bounds3& first, const Bounds3& second)
{
    assert(first.isValid());
    assert(second.isValid());

    const MyMath::Vector3& firstMinimum = first.minimum();
    const MyMath::Vector3& firstMaximum = first.maximum();
    const MyMath::Vector3& secondMinimum = second.minimum();
    const MyMath::Vector3& secondMaximum = second.maximum();

    return firstMaximum.x() >= secondMinimum.x() &&
           firstMinimum.x() <= secondMaximum.x() &&
           firstMaximum.y() >= secondMinimum.y() &&
           firstMinimum.y() <= secondMaximum.y() &&
           firstMaximum.z() >= secondMinimum.z() &&
           firstMinimum.z() <= secondMaximum.z();
}

bool VoxelPackedForestQuery::contains(const Bounds3& outer, const Bounds3& inner)
{
    assert(outer.isValid());
    assert(inner.isValid());

    const MyMath::Vector3& outerMinimum = outer.minimum();
    const MyMath::Vector3& outerMaximum = outer.maximum();
    const MyMath::Vector3& innerMinimum = inner.minimum();
    const MyMath::Vector3& innerMaximum = inner.maximum();

    return outerMinimum.x() <= innerMinimum.x() &&
           outerMinimum.y() <= innerMinimum.y() &&
           outerMinimum.z() <= innerMinimum.z() &&
           outerMaximum.x() >= innerMaximum.x() &&
           outerMaximum.y() >= innerMaximum.y() &&
           outerMaximum.z() >= innerMaximum.z();
}

Bounds3 VoxelPackedForestQuery::childBounds(const Bounds3& parentBounds, VoxelCorner corner)
{
    assert(parentBounds.isValid());

    const MyMath::Vector3& minimum = parentBounds.minimum();
    const MyMath::Vector3& maximum = parentBounds.maximum();

    const double middleX = (minimum.x() + maximum.x()) * MiddleScale;
    const double middleY = (minimum.y() + maximum.y()) * MiddleScale;
    const double middleZ = (minimum.z() + maximum.z()) * MiddleScale;

    const bool maximumX = cornerUsesMaximumX(corner);
    const bool maximumY = cornerUsesMaximumY(corner);
    const bool maximumZ = cornerUsesMaximumZ(corner);

    return Bounds3(
        MyMath::Vector3(
            maximumX ? middleX : minimum.x(),
            maximumY ? middleY : minimum.y(),
            maximumZ ? middleZ : minimum.z()),
        MyMath::Vector3(
            maximumX ? maximum.x() : middleX,
            maximumY ? maximum.y() : middleY,
            maximumZ ? maximum.z() : middleZ));
}

}