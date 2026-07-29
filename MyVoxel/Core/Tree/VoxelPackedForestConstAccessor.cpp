#include "VoxelPackedForestConstAccessor.h"

#include <algorithm>
#include <cassert>

namespace MyVoxel
{

VoxelPackedForestConstAccessor::VoxelPackedForestConstAccessor(const VoxelPackedForest& forest)
    : m_forest(&forest)
    , m_cachedTree(nullptr)
    , m_hasCachedRoot(false)
{
}

VoxelState VoxelPackedForestConstAccessor::state(const VoxelCellAddress& address)
{
    assert(m_forest);

    const VoxelCellAddress rootAddress = VoxelPackedForest::rootCellAddress(address);
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
        m_cachedCursors.clear();

        if (m_cachedTree)
        {
            m_cachedCursors.push_back(VoxelPackedTreeConstCursor(*m_cachedTree));
        }
    }

    if (!m_cachedTree)
    {
        return VoxelState::Empty;
    }

    std::vector<VoxelCorner> targetCorners;
    VoxelPackedForest::buildCornerPath(address, targetCorners);

    std::size_t reusedLevelCount = 0;
    const std::size_t reusableCornerCount = std::min(targetCorners.size(), m_cachedCorners.size());

    while (reusedLevelCount < reusableCornerCount &&
           reusedLevelCount + 1 < m_cachedCursors.size() &&
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
    m_cachedCursors.erase(m_cachedCursors.begin() + reusedLevelCount + 1, m_cachedCursors.end());

    VoxelPackedTreeConstCursor cursor = m_cachedCursors.back();

    for (std::size_t levelIndex = reusedLevelCount; levelIndex < targetCorners.size(); ++levelIndex)
    {
        ++m_statistics.nodeVisitCount;

        const VoxelState currentState = cursor.state();

        if (currentState != VoxelState::Subdivided)
        {
            return currentState;
        }

        const VoxelCorner corner = targetCorners[levelIndex];

        cursor = cursor.child(corner);
        m_cachedCorners.push_back(corner);
        m_cachedCursors.push_back(cursor);
    }

    ++m_statistics.nodeVisitCount;
    return cursor.state();
}

void VoxelPackedForestConstAccessor::clear()
{
    m_cachedTree = nullptr;
    m_hasCachedRoot = false;
    m_cachedCorners.clear();
    m_cachedCursors.clear();
    m_statistics.reset();
}

const VoxelPackedForestConstAccessorStatistics& VoxelPackedForestConstAccessor::statistics() const
{
    return m_statistics;
}

bool VoxelPackedForestConstAccessor::sameIndex(const VoxelCellIndex& first, const VoxelCellIndex& second)
{
    return first.x == second.x && first.y == second.y && first.z == second.z;
}

}