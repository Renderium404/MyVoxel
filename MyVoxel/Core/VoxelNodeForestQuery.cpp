#include "VoxelNodeForestQuery.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include "VoxelRootTree.h"
#include "VoxelNode.h"
#include "VoxelNodeForest.h"
#include "VoxelNodeGroup.h"

namespace
{

// 检查局部包围盒是否包含有限且有序的坐标范围。
bool isValidBox(const MyVoxel::VoxelLocalBox& bounds)
{
    return std::isfinite(bounds.minimumX) && std::isfinite(bounds.minimumY) && std::isfinite(bounds.minimumZ) &&
           std::isfinite(bounds.maximumX) && std::isfinite(bounds.maximumY) && std::isfinite(bounds.maximumZ) &&
           bounds.minimumX <= bounds.maximumX && bounds.minimumY <= bounds.maximumY && bounds.minimumZ <= bounds.maximumZ;
}

// 将连续坐标转换为指定体素边长下的空间索引。
MyVoxel::VoxelIndex coordinateIndex(double coordinate, double edgeLength)
{
    assert(std::isfinite(coordinate));
    assert(std::isfinite(edgeLength) && edgeLength > 0.0);

    const double value = std::floor(coordinate / edgeLength);
    const double minimum = static_cast<double>((std::numeric_limits<MyVoxel::VoxelIndex>::min)());
    const double maximum = static_cast<double>((std::numeric_limits<MyVoxel::VoxelIndex>::max)());

    assert(value >= minimum && value <= maximum);
    return static_cast<MyVoxel::VoxelIndex>(value);
}

// 检查outer是否完整包含inner。
bool containsBox(const MyVoxel::VoxelLocalBox& outer, const MyVoxel::VoxelLocalBox& inner)
{
    return inner.minimumX >= outer.minimumX && inner.maximumX <= outer.maximumX &&
           inner.minimumY >= outer.minimumY && inner.maximumY <= outer.maximumY &&
           inner.minimumZ >= outer.minimumZ && inner.maximumZ <= outer.maximumZ;
}

}

namespace MyVoxel
{

VoxelLocalBox::VoxelLocalBox(double minimumXValue, double minimumYValue, double minimumZValue, double maximumXValue, double maximumYValue, double maximumZValue)
    : minimumX(minimumXValue)
    , minimumY(minimumYValue)
    , minimumZ(minimumZValue)
    , maximumX(maximumXValue)
    , maximumY(maximumYValue)
    , maximumZ(maximumZValue)
{
    assert(isValidBox(*this));
}

void VoxelNodeForestQueryStatistics::reset()
{
    *this = VoxelNodeForestQueryStatistics();
}

void VoxelNodeForestQueryStatistics::accumulate(const VoxelNodeForestQueryStatistics& other)
{
    rootCandidateCount += other.rootCandidateCount;
    existingRootCount += other.existingRootCount;
    nodeBoundsTestCount += other.nodeBoundsTestCount;
    visitedNodeCount += other.visitedNodeCount;
    emptyNodeCount += other.emptyNodeCount;
    materialNodeCount += other.materialNodeCount;
    subdividedNodeCount += other.subdividedNodeCount;
}

VoxelNodeForestQuery::VoxelNodeForestQuery(const VoxelNodeForest& forest, double baseVoxelEdgeLength)
    : m_forest(&forest)
    , m_baseVoxelEdgeLength(baseVoxelEdgeLength)
{
    assert(std::isfinite(baseVoxelEdgeLength) && baseVoxelEdgeLength > 0.0);
}

VoxelRegionRelation VoxelNodeForestQuery::classify(const VoxelLocalBox& bounds) const
{
    return classifyImpl(bounds, nullptr);
}

VoxelRegionRelation VoxelNodeForestQuery::classify(const VoxelLocalBox& bounds, VoxelNodeForestQueryStatistics& statistics) const
{
    statistics.reset();
    return classifyImpl(bounds, &statistics);
}

VoxelRegionRelation VoxelNodeForestQuery::classifyImpl(const VoxelLocalBox& bounds, VoxelNodeForestQueryStatistics* statistics) const
{
    assert(m_forest);
    assert(isValidBox(bounds));

    const VoxelIndex minimumX = coordinateIndex(bounds.minimumX, m_baseVoxelEdgeLength);
    const VoxelIndex minimumY = coordinateIndex(bounds.minimumY, m_baseVoxelEdgeLength);
    const VoxelIndex minimumZ = coordinateIndex(bounds.minimumZ, m_baseVoxelEdgeLength);
    const VoxelIndex maximumX = coordinateIndex(bounds.maximumX, m_baseVoxelEdgeLength);
    const VoxelIndex maximumY = coordinateIndex(bounds.maximumY, m_baseVoxelEdgeLength);
    const VoxelIndex maximumZ = coordinateIndex(bounds.maximumZ, m_baseVoxelEdgeLength);
    bool possibleIntersection = false;

    for (std::int64_t z = minimumZ; z <= maximumZ; ++z)
    {
        for (std::int64_t y = minimumY; y <= maximumY; ++y)
        {
            for (std::int64_t x = minimumX; x <= maximumX; ++x)
            {
                if (statistics)
                {
                    ++statistics->rootCandidateCount;
                }

                const VoxelCellIndex rootIndex(static_cast<VoxelIndex>(x), static_cast<VoxelIndex>(y), static_cast<VoxelIndex>(z));
                const VoxelNodeForest::RootMap::const_iterator iterator = m_forest->m_roots.find(rootIndex);

                if (iterator == m_forest->m_roots.end())
                {
                    continue;
                }

                if (statistics)
                {
                    ++statistics->existingRootCount;
                }

                const double minimumRootX = static_cast<double>(rootIndex.x) * m_baseVoxelEdgeLength;
                const double minimumRootY = static_cast<double>(rootIndex.y) * m_baseVoxelEdgeLength;
                const double minimumRootZ = static_cast<double>(rootIndex.z) * m_baseVoxelEdgeLength;
                const VoxelLocalBox rootBounds(minimumRootX, minimumRootY, minimumRootZ, minimumRootX + m_baseVoxelEdgeLength, minimumRootY + m_baseVoxelEdgeLength, minimumRootZ + m_baseVoxelEdgeLength);
                assert(iterator->second);

                const VoxelRootTree& tree = *iterator->second;
                const VoxelRegionRelation relation = classifyNode(bounds, rootBounds, tree.root, tree.nodePool, statistics);

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

VoxelRegionRelation VoxelNodeForestQuery::classifyNode(const VoxelLocalBox& bounds, const VoxelLocalBox& nodeBounds, const VoxelNode& node, const VoxelNodePool& nodePool, VoxelNodeForestQueryStatistics* statistics) const
{
    if (statistics)
    {
        ++statistics->nodeBoundsTestCount;
        ++statistics->visitedNodeCount;
    }

    if (node.state() == VoxelState::Empty)
    {
        if (statistics)
        {
            ++statistics->emptyNodeCount;
        }

        return VoxelRegionRelation::Outside;
    }

    if (node.state() == VoxelState::Material)
    {
        if (statistics)
        {
            ++statistics->materialNodeCount;
        }

        return containsBox(nodeBounds, bounds) ? VoxelRegionRelation::Inside : VoxelRegionRelation::Intersecting;
    }

    assert(node.state() == VoxelState::Subdivided);

    if (statistics)
    {
        ++statistics->subdividedNodeCount;
    }

    const VoxelNodeGroup& group = nodePool.nodeGroup(node.childGroupIndex());
    const double middleX = (nodeBounds.minimumX + nodeBounds.maximumX) * 0.5;
    const double middleY = (nodeBounds.minimumY + nodeBounds.maximumY) * 0.5;
    const double middleZ = (nodeBounds.minimumZ + nodeBounds.maximumZ) * 0.5;
    const int minimumXSide = bounds.minimumX <= middleX ? 0 : 1;
    const int maximumXSide = bounds.maximumX >= middleX ? 1 : 0;
    const int minimumYSide = bounds.minimumY <= middleY ? 0 : 1;
    const int maximumYSide = bounds.maximumY >= middleY ? 1 : 0;
    const int minimumZSide = bounds.minimumZ <= middleZ ? 0 : 1;
    const int maximumZSide = bounds.maximumZ >= middleZ ? 1 : 0;
    bool possibleIntersection = false;

    for (int zSide = minimumZSide; zSide <= maximumZSide; ++zSide)
    {
        for (int ySide = minimumYSide; ySide <= maximumYSide; ++ySide)
        {
            for (int xSide = minimumXSide; xSide <= maximumXSide; ++xSide)
            {
                const unsigned int cornerValue = static_cast<unsigned int>(xSide | (ySide << 1) | (zSide << 2));
                const VoxelCorner corner = static_cast<VoxelCorner>(cornerValue);
                const VoxelLocalBox childBounds(
                    xSide == 0 ? nodeBounds.minimumX : middleX,
                    ySide == 0 ? nodeBounds.minimumY : middleY,
                    zSide == 0 ? nodeBounds.minimumZ : middleZ,
                    xSide == 0 ? middleX : nodeBounds.maximumX,
                    ySide == 0 ? middleY : nodeBounds.maximumY,
                    zSide == 0 ? middleZ : nodeBounds.maximumZ);

                const VoxelRegionRelation relation = classifyNode(bounds, childBounds, group.child(corner), nodePool, statistics);

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
}