#include "VoxelForestConstAccessor.h"

#include <algorithm>
#include <cstddef>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

VoxelForestConstAccessorStatistics::VoxelForestConstAccessorStatistics()
    : rootCacheHitCount(0)
    , rootCacheMissCount(0)
    , pathReuseCount(0)
    , reusedPathLevelCount(0)
    , nodeVisitCount(0)
{
}

void VoxelForestConstAccessorStatistics::clear()
{
    rootCacheHitCount = 0;
    rootCacheMissCount = 0;
    pathReuseCount = 0;
    reusedPathLevelCount = 0;
    nodeVisitCount = 0;
}

VoxelForestConstAccessor::VoxelForestConstAccessor(const VoxelForest& forest)
    : m_forest(&forest)
    , m_cachedTree(nullptr)
    , m_cachedRootIndex()
    , m_hasCachedRoot(false)
{
}

/// 节点查询

VoxelState VoxelForestConstAccessor::state(const VoxelCellAddress& address)
{
    MYVOXEL_ASSERT_MESSAGE(m_forest, "VoxelForestConstAccessor requires a valid VoxelForest.");

    const VoxelCellAddress rootAddress = rootCellAddress(address);
    const bool rootCacheHit = m_hasCachedRoot && m_cachedRootIndex == rootAddress.index;

    if (rootCacheHit)
    {
        ++m_statistics.rootCacheHitCount;
    }
    else
    {
        ++m_statistics.rootCacheMissCount;

        m_cachedRootIndex = rootAddress.index;
        m_cachedTree = m_forest->findTree(rootAddress.index);
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

    MYVOXEL_ASSERT_MESSAGE(m_cachedNodes.size() == m_cachedCorners.size() + 1, "VoxelForestConstAccessor cached node path is inconsistent.");

    std::vector<VoxelCorner> targetCorners;
    VoxelForest::buildCornerPath(address, targetCorners);

    std::size_t reusedLevelCount = 0;
    const std::size_t reusableCornerCount = (std::min)(targetCorners.size(), m_cachedCorners.size());

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

        MYVOXEL_ASSERT_MESSAGE(node && node->isValid(), "VoxelForestConstAccessor encountered an invalid node.");

        if (!node->isSubdivided())
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

    MYVOXEL_ASSERT_MESSAGE(node && node->isValid(), "VoxelForestConstAccessor target node must be valid.");
    return node->state();
}

/// 缓存与统计

void VoxelForestConstAccessor::clear()
{
    m_cachedTree = nullptr;
    m_cachedRootIndex = VoxelCellIndex();
    m_hasCachedRoot = false;
    m_cachedCorners.clear();
    m_cachedNodes.clear();
    m_statistics.clear();
}

const VoxelForestConstAccessorStatistics& VoxelForestConstAccessor::statistics() const
{
    return m_statistics;
}

}