#include "VoxelNodeForestConstAccessor.h"

#include <algorithm>
#include <cassert>

namespace MyVoxel
{

VoxelNodeForestConstAccessor::VoxelNodeForestConstAccessor(const VoxelNodeForest& forest)
    : m_forest(&forest)
    , m_cachedTree(nullptr)
    , m_hasCachedRoot(false)
{
}

VoxelState VoxelNodeForestConstAccessor::state(const VoxelCellAddress& address)
{
    assert(m_forest);

    const VoxelCellAddress rootAddress = VoxelNodeForest::rootCellAddress(address);
    const bool rootCacheHit = m_hasCachedRoot && sameIndex(m_cachedRootIndex, rootAddress.index);

    if (rootCacheHit)
    {
        ++m_statistics.rootCacheHitCount;
    }
    else
    {
        ++m_statistics.rootCacheMissCount;

        m_cachedRootIndex = rootAddress.index;
        m_cachedTree = m_forest->findRootTree(rootAddress.index);
        m_hasCachedRoot = true;
        m_cachedCorners.clear();
        m_cachedNodes.clear();

        if (m_cachedTree)
        {
            m_cachedNodes.push_back(&m_cachedTree->root);
        }
    }

    if (!m_cachedTree)
    {
        return VoxelState::Empty;
    }

    std::vector<VoxelCorner> targetCorners;
    VoxelNodeForest::buildCornerPath(address, targetCorners);

    std::size_t reusedLevelCount = 0;
    const std::size_t reusableCornerCount = std::min(targetCorners.size(), m_cachedCorners.size());

    while (reusedLevelCount < reusableCornerCount &&
           reusedLevelCount + 1 < m_cachedNodes.size() &&
           targetCorners[reusedLevelCount] == m_cachedCorners[reusedLevelCount])
    {
        ++reusedLevelCount;
    }

    if (reusedLevelCount > 0)
    {
        ++m_statistics.pathReuseCount;
        m_statistics.reusedPathLevelCount += static_cast<std::uint64_t>(reusedLevelCount);
    }

    m_cachedCorners.resize(reusedLevelCount);
    m_cachedNodes.resize(reusedLevelCount + 1);

    const VoxelNode* node = m_cachedNodes.back();

    for (std::size_t levelIndex = reusedLevelCount; levelIndex < targetCorners.size(); ++levelIndex)
    {
        ++m_statistics.nodeVisitCount;

        if (node->state() != VoxelState::Subdivided)
        {
            return node->state();
        }

        const VoxelCorner corner = targetCorners[levelIndex];
        const VoxelNodeGroup& group = m_cachedTree->nodePool.nodeGroup(node->childGroupIndex());

        node = &group.child(corner);
        m_cachedCorners.push_back(corner);
        m_cachedNodes.push_back(node);
    }

    ++m_statistics.nodeVisitCount;
    return node->state();
}

void VoxelNodeForestConstAccessor::clear()
{
    m_cachedTree = nullptr;
    m_hasCachedRoot = false;
    m_cachedCorners.clear();
    m_cachedNodes.clear();
    m_statistics = VoxelNodeForestConstAccessorStatistics();
}

const VoxelNodeForestConstAccessorStatistics& VoxelNodeForestConstAccessor::statistics() const
{
    return m_statistics;
}

bool VoxelNodeForestConstAccessor::sameIndex(const VoxelCellIndex& first, const VoxelCellIndex& second)
{
    return first.x == second.x && first.y == second.y && first.z == second.z;
}

}