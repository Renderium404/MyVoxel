#include "VoxelForestQuery.h"

#include <algorithm>
#include <cstddef>
#include <limits>

#include "MyMath/Vector3.h"
#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

const double HalfScale = 0.5; // 父节点包围盒中点计算使用的固定比例。

// 检查两个半开包围盒是否存在三个方向均为正长度的重叠区域。
bool hasVolumeIntersection(const MyVoxel::Bounds3& first, const MyVoxel::Bounds3& second)
{
    return first.maximum().x() > second.minimum().x() && first.minimum().x() < second.maximum().x() &&
           first.maximum().y() > second.minimum().y() && first.minimum().y() < second.maximum().y() &&
           first.maximum().z() > second.minimum().z() && first.minimum().z() < second.maximum().z();
}

// 检查outer是否完整包含inner。
bool containsBounds(const MyVoxel::Bounds3& outer, const MyVoxel::Bounds3& inner)
{
    return inner.minimum().x() >= outer.minimum().x() && inner.maximum().x() <= outer.maximum().x() &&
           inner.minimum().y() >= outer.minimum().y() && inner.maximum().y() <= outer.maximum().y() &&
           inner.minimum().z() >= outer.minimum().z() && inner.maximum().z() <= outer.maximum().z();
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

}

namespace MyVoxel
{

VoxelForestQueryStatistics::VoxelForestQueryStatistics()
    : rootCandidateCount(0)
    , existingRootCount(0)
    , nodeBoundsTestCount(0)
    , visitedNodeCount(0)
    , emptyNodeCount(0)
    , materialNodeCount(0)
    , subdividedNodeCount(0)
{
}

void VoxelForestQueryStatistics::reset()
{
    rootCandidateCount = 0;
    existingRootCount = 0;
    nodeBoundsTestCount = 0;
    visitedNodeCount = 0;
    emptyNodeCount = 0;
    materialNodeCount = 0;
    subdividedNodeCount = 0;
}

void VoxelForestQueryStatistics::accumulate(const VoxelForestQueryStatistics& other)
{
    rootCandidateCount += other.rootCandidateCount;
    existingRootCount += other.existingRootCount;
    nodeBoundsTestCount += other.nodeBoundsTestCount;
    visitedNodeCount += other.visitedNodeCount;
    emptyNodeCount += other.emptyNodeCount;
    materialNodeCount += other.materialNodeCount;
    subdividedNodeCount += other.subdividedNodeCount;
}

VoxelForestQuery::VoxelForestQuery(const VoxelForest& forest, const VoxelGrid& grid)
    : m_forest(&forest)
    , m_grid(grid)
{
    MYVOXEL_ASSERT_MESSAGE(m_grid.isValid(), "VoxelForestQuery requires a valid VoxelGrid.");
}

VoxelRegionRelation VoxelForestQuery::classify(const Bounds3& bounds) const
{
    return classifyImpl(bounds, nullptr);
}

VoxelRegionRelation VoxelForestQuery::classify(const Bounds3& bounds, VoxelForestQueryStatistics& statistics) const
{
    statistics.reset();
    return classifyImpl(bounds, &statistics);
}

VoxelRegionRelation VoxelForestQuery::classifyImpl(const Bounds3& bounds, VoxelForestQueryStatistics* statistics) const
{
    MYVOXEL_ASSERT_MESSAGE(m_forest, "VoxelForestQuery requires a valid VoxelForest.");
    MYVOXEL_ASSERT_MESSAGE(bounds.isValid(), "VoxelForestQuery bounds must be valid.");
    MYVOXEL_ASSERT_MESSAGE(bounds.hasVolume(), "VoxelForestQuery bounds must have volume.");

    const VoxelCellRange rootRange = m_grid.cellRange(bounds, BaseVoxelLevel);

    if (statistics)
    {
        const std::uint64_t xyCandidateCount = saturatedMultiply(rootRange.countX(), rootRange.countY());
        statistics->rootCandidateCount = saturatedMultiply(xyCandidateCount, rootRange.countZ());
    }

    if (rootRange.minimum == rootRange.maximum)
    {
        const VoxelTree* tree = m_forest->findTree(rootRange.minimum);

        if (!tree)
        {
            return VoxelRegionRelation::Outside;
        }

        if (statistics)
        {
            statistics->existingRootCount = 1;
        }

        const VoxelCellAddress rootAddress(rootRange.minimum, BaseVoxelLevel);
        return classifyNode(bounds, m_grid.cellBounds(rootAddress), tree->root, tree->nodePool, statistics);
    }

    bool possibleIntersection = false;

    const std::size_t existingTreeCount = m_forest->visitTreesInRange(
        rootRange.minimum,
        rootRange.maximum,
        [this, &bounds, &possibleIntersection, statistics](const VoxelCellIndex& rootIndex, const VoxelTree& tree)
        {
            const VoxelCellAddress rootAddress(rootIndex, BaseVoxelLevel);
            const VoxelRegionRelation relation = classifyNode(bounds, m_grid.cellBounds(rootAddress), tree.root, tree.nodePool, statistics);

            if (relation != VoxelRegionRelation::Outside)
            {
                possibleIntersection = true;
            }
        });

    if (statistics)
    {
        statistics->existingRootCount = static_cast<std::uint64_t>(existingTreeCount);
    }

    // 跨越多个根体素时，即使多个材料节点共同覆盖目标区域，也保守返回Intersecting。
    return possibleIntersection ? VoxelRegionRelation::Intersecting : VoxelRegionRelation::Outside;
}

VoxelRegionRelation VoxelForestQuery::classifyNode(const Bounds3& bounds, const Bounds3& nodeBounds, const VoxelNode& node, const VoxelNodePool& nodePool, VoxelForestQueryStatistics* statistics) const
{
    MYVOXEL_ASSERT_MESSAGE(bounds.isValid() && bounds.hasVolume(), "VoxelForestQuery target bounds must be valid and have volume.");
    MYVOXEL_ASSERT_MESSAGE(nodeBounds.isValid() && nodeBounds.hasVolume(), "VoxelForestQuery node bounds must be valid and have volume.");
    MYVOXEL_ASSERT_MESSAGE(node.isValid(), "VoxelForestQuery encountered an invalid node.");

    if (statistics)
    {
        ++statistics->nodeBoundsTestCount;
        ++statistics->visitedNodeCount;
    }

    if (!hasVolumeIntersection(bounds, nodeBounds))
    {
        return VoxelRegionRelation::Outside;
    }

    if (node.isEmpty())
    {
        if (statistics)
        {
            ++statistics->emptyNodeCount;
        }

        return VoxelRegionRelation::Outside;
    }

    if (node.isMaterial())
    {
        if (statistics)
        {
            ++statistics->materialNodeCount;
        }

        return containsBounds(nodeBounds, bounds) ? VoxelRegionRelation::Inside : VoxelRegionRelation::Intersecting;
    }

    MYVOXEL_ASSERT_MESSAGE(node.isSubdivided(), "VoxelForestQuery requires a valid node state.");

    if (statistics)
    {
        ++statistics->subdividedNodeCount;
    }

    const MyMath::Vector3& minimum = nodeBounds.minimum();
    const MyMath::Vector3& maximum = nodeBounds.maximum();
    const double middleX = (minimum.x() + maximum.x()) * HalfScale;
    const double middleY = (minimum.y() + maximum.y()) * HalfScale;
    const double middleZ = (minimum.z() + maximum.z()) * HalfScale;

    // 半开体素区间下，位于中分面上的最小边界归入正方向，最大边界归入负方向。
    const int minimumXSide = bounds.minimum().x() < middleX ? 0 : 1;
    const int maximumXSide = bounds.maximum().x() > middleX ? 1 : 0;
    const int minimumYSide = bounds.minimum().y() < middleY ? 0 : 1;
    const int maximumYSide = bounds.maximum().y() > middleY ? 1 : 0;
    const int minimumZSide = bounds.minimum().z() < middleZ ? 0 : 1;
    const int maximumZSide = bounds.maximum().z() > middleZ ? 1 : 0;

    const VoxelNodeGroup& group = nodePool.nodeGroup(node.childGroupIndex());
    bool possibleIntersection = false;

    for (int zSide = minimumZSide; zSide <= maximumZSide; ++zSide)
    {
        for (int ySide = minimumYSide; ySide <= maximumYSide; ++ySide)
        {
            for (int xSide = minimumXSide; xSide <= maximumXSide; ++xSide)
            {
                const unsigned int cornerValue = static_cast<unsigned int>(xSide | (ySide << 1) | (zSide << 2));
                const VoxelCorner corner = static_cast<VoxelCorner>(cornerValue);

                const MyMath::Vector3 childMinimum(
                    xSide == 0 ? minimum.x() : middleX,
                    ySide == 0 ? minimum.y() : middleY,
                    zSide == 0 ? minimum.z() : middleZ);

                const MyMath::Vector3 childMaximum(
                    xSide == 0 ? middleX : maximum.x(),
                    ySide == 0 ? middleY : maximum.y(),
                    zSide == 0 ? middleZ : maximum.z());

                const Bounds3 childBounds(childMinimum, childMaximum);
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