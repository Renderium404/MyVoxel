#include "VoxelizationWorkspace.h"

#include <utility>

#include "MyVoxel/Core/VoxelShapeSession.h"
#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{
namespace Modeling
{

VoxelizationWorkspace::VoxelizationWorkspace()
    : m_initialized(false)
    , m_exactRootReuseCount(0)
    , m_fallbackRootReuseCount(0)
    , m_createdRootTreeCount(0)
{
}

/// 当前结果

const VoxelShape& VoxelizationWorkspace::result() const
{
    MYVOXEL_ASSERT_MESSAGE(m_initialized, "VoxelizationWorkspace does not contain an initialized result.");
    MYVOXEL_ASSERT_MESSAGE(m_result.isValid(), "VoxelizationWorkspace result must be valid.");
    return m_result;
}

bool VoxelizationWorkspace::isInitialized() const
{
    return m_initialized;
}

/// 资源管理

void VoxelizationWorkspace::reset()
{
    if (!m_initialized)
    {
        return;
    }

    if (m_result.isDataShared())
    {
        // 外部仍持有当前结果时不能移动其根树，重新建立相同场参数的空结果以保持快照语义。
        const VoxelGrid resultGrid = m_result.grid();
        const float resultBackgroundDistance = m_result.backgroundDistance();
        const MyMath::Matrix4 resultTransform = m_result.transform();
        m_result = VoxelShape(resultGrid, resultBackgroundDistance);
        m_result.setTransform(resultTransform);
        return;
    }

    recycleResultRoots();
    MYVOXEL_ASSERT_MESSAGE(m_result.isEmpty(), "VoxelizationWorkspace reset result must be empty.");
    MYVOXEL_ASSERT_MESSAGE(m_result.features().isEmpty() && m_result.hasCompleteFeatures(), "VoxelizationWorkspace reset result must contain a complete empty FeatureSet.");
}

void VoxelizationWorkspace::release()
{
    m_cachedRootTrees.clear();
    m_result = VoxelShape();
    m_initialized = false;
    resetStatistics();
}

/// 缓存统计

std::size_t VoxelizationWorkspace::cachedRootSlotCount() const
{
    return m_cachedRootTrees.size();
}

std::size_t VoxelizationWorkspace::retainedRootSlotCount() const
{
    const std::size_t activeRootCount = m_initialized ? m_result.rootCount() : 0;
    return activeRootCount + m_cachedRootTrees.size();
}

std::size_t VoxelizationWorkspace::retainedChunkCount() const
{
    std::size_t chunkCount = 0;

    for (std::size_t rootPosition = 0; rootPosition < m_cachedRootTrees.size(); ++rootPosition)
    {
        const VoxelTree& tree = m_cachedRootTrees[rootPosition].second;
        chunkCount += tree.nodeChunkCount() + tree.leafChunkCount();
    }

    if (!m_initialized)
    {
        return chunkCount;
    }

    m_result.forest().forEachRootCell(
        [&](const VoxelCellAddress& rootAddress)
        {
            const VoxelTree* tree = m_result.forest().getTree(rootAddress.index);
            MYVOXEL_ASSERT(tree);
            chunkCount += tree->nodeChunkCount() + tree->leafChunkCount();
        });

    return chunkCount;
}

std::size_t VoxelizationWorkspace::retainedStorageCapacityBytes() const
{
    std::size_t capacityBytes = 0;

    for (std::size_t rootPosition = 0; rootPosition < m_cachedRootTrees.size(); ++rootPosition)
    {
        capacityBytes += m_cachedRootTrees[rootPosition].second.storageCapacityBytes();
    }

    if (!m_initialized)
    {
        return capacityBytes;
    }

    m_result.forest().forEachRootCell(
        [&](const VoxelCellAddress& rootAddress)
        {
            const VoxelTree* tree = m_result.forest().getTree(rootAddress.index);
            MYVOXEL_ASSERT(tree);
            capacityBytes += tree->storageCapacityBytes();
        });

    return capacityBytes;
}

/// 运行统计

std::size_t VoxelizationWorkspace::exactRootReuseCount() const
{
    return m_exactRootReuseCount;
}

std::size_t VoxelizationWorkspace::fallbackRootReuseCount() const
{
    return m_fallbackRootReuseCount;
}

std::size_t VoxelizationWorkspace::createdRootTreeCount() const
{
    return m_createdRootTreeCount;
}

void VoxelizationWorkspace::resetStatistics()
{
    m_exactRootReuseCount = 0;
    m_fallbackRootReuseCount = 0;
    m_createdRootTreeCount = 0;
}

/// 内部辅助

void VoxelizationWorkspace::prepare(const VoxelShape& reference)
{
    MYVOXEL_ASSERT_MESSAGE(reference.isValid(), "VoxelizationWorkspace requires a valid reference VoxelShape.");

    // reference可能直接引用m_result，因此必须在修改m_result之前保存完整参考场参数。
    const VoxelGrid referenceGrid = reference.grid();
    const float referenceBackgroundDistance = reference.backgroundDistance();
    const MyMath::Matrix4 referenceTransform = reference.transform();

    const bool resultShared = m_initialized && m_result.isDataShared();
    const bool sameGrid = m_initialized && m_result.grid().isEqualTo(referenceGrid, 0.0);
    const bool sameBackgroundDistance = m_initialized && m_result.backgroundDistance() == referenceBackgroundDistance;

    if (m_initialized && !resultShared)
    {
        recycleResultRoots();
    }

    if (!m_initialized || resultShared || !sameGrid || !sameBackgroundDistance)
    {
        m_result = VoxelShape(referenceGrid, referenceBackgroundDistance);
    }
    else
    {
        MYVOXEL_ASSERT_MESSAGE(m_result.isEmpty(), "Reusable VoxelizationWorkspace result must be empty before voxelization.");
        MYVOXEL_ASSERT_MESSAGE(m_result.features().isEmpty() && m_result.hasCompleteFeatures(), "Reusable VoxelizationWorkspace result must not retain features from the previous voxelization.");
    }

    m_result.setTransform(referenceTransform);
    m_initialized = true;
}

VoxelTree VoxelizationWorkspace::acquireRootTree(const VoxelCellIndex& rootIndex)
{
    MYVOXEL_ASSERT_MESSAGE(m_initialized && m_result.isValid(), "VoxelizationWorkspace must be prepared before acquiring a root tree.");
    const float backgroundDistance = m_result.backgroundDistance();

    // 优先复用上一轮属于相同根索引的BlockPool。
    for (std::size_t rootPosition = 0; rootPosition < m_cachedRootTrees.size(); ++rootPosition)
    {
        if (m_cachedRootTrees[rootPosition].first != rootIndex)
        {
            continue;
        }

        VoxelTree tree(std::move(m_cachedRootTrees[rootPosition].second));

        if (rootPosition + 1 < m_cachedRootTrees.size())
        {
            m_cachedRootTrees[rootPosition] = std::move(m_cachedRootTrees.back());
        }

        m_cachedRootTrees.pop_back();
        tree.reset(VoxelState::Empty, backgroundDistance);
        ++m_exactRootReuseCount;

        MYVOXEL_ASSERT_MESSAGE(tree.isValid(), "Acquired cached root tree must be valid.");
        MYVOXEL_ASSERT_MESSAGE(tree.state() == VoxelState::Empty && tree.value() == backgroundDistance, "Acquired cached root tree must represent current +B background.");
        return tree;
    }

    if (m_cachedRootTrees.empty())
    {
        ++m_createdRootTreeCount;
        return VoxelTree(VoxelState::Empty, backgroundDistance);
    }

    // 新出现的根无法预先确定存储需求，优先复用容量最大的BlockPool以减少大型缓存闲置。
    std::size_t largestPosition = 0;

    for (std::size_t rootPosition = 1; rootPosition < m_cachedRootTrees.size(); ++rootPosition)
    {
        if (m_cachedRootTrees[rootPosition].second.storageCapacityBytes() > m_cachedRootTrees[largestPosition].second.storageCapacityBytes())
        {
            largestPosition = rootPosition;
        }
    }

    VoxelTree tree(std::move(m_cachedRootTrees[largestPosition].second));

    if (largestPosition + 1 < m_cachedRootTrees.size())
    {
        m_cachedRootTrees[largestPosition] = std::move(m_cachedRootTrees.back());
    }

    m_cachedRootTrees.pop_back();
    tree.reset(VoxelState::Empty, backgroundDistance);
    ++m_fallbackRootReuseCount;

    MYVOXEL_ASSERT_MESSAGE(tree.isValid(), "Acquired cached root tree must be valid.");
    MYVOXEL_ASSERT_MESSAGE(tree.state() == VoxelState::Empty && tree.value() == backgroundDistance, "Acquired cached root tree must represent current +B background.");
    return tree;
}

void VoxelizationWorkspace::recycleRootTree(const VoxelCellIndex& rootIndex, VoxelTree&& tree)
{
    MYVOXEL_ASSERT_MESSAGE(m_initialized && m_result.isValid(), "VoxelizationWorkspace must be initialized before recycling root trees.");

    if (tree.storageCapacityBytes() == 0)
    {
        return;
    }

    const float backgroundDistance = m_result.backgroundDistance();
    tree.reset(VoxelState::Empty, backgroundDistance);

    MYVOXEL_ASSERT_MESSAGE(tree.isValid(), "Recycled root tree must remain valid.");
    MYVOXEL_ASSERT_MESSAGE(tree.state() == VoxelState::Empty && tree.value() == backgroundDistance, "Recycled root tree must represent current +B background.");
    MYVOXEL_ASSERT_MESSAGE(tree.allocatedGroupCount() == 0 && tree.allocatedLeafCount() == 0, "Recycled root tree must not retain active groups or leaves.");

    m_cachedRootTrees.push_back(VoxelForest::TreeEntry(rootIndex, std::move(tree)));
}

void VoxelizationWorkspace::recycleResultRoots()
{
    MYVOXEL_ASSERT_MESSAGE(!m_result.isDataShared(), "Shared voxelization result cannot be recycled in place.");

    std::vector<VoxelForest::TreeEntry> extractedRoots;

    {
        VoxelShapeSession session = m_result.session(BaseVoxelLevel);

        if (!session.isEmpty())
        {
            session.moveTreesTo(extractedRoots);
        }

        // moveTreesTo会把Feature标记为Unavailable；工作区空结果随后恢复为确认完整的空FeatureSet。
        session.clearFeatures();
    }

    for (std::size_t rootPosition = 0; rootPosition < extractedRoots.size(); ++rootPosition)
    {
        recycleRootTree(extractedRoots[rootPosition].first, std::move(extractedRoots[rootPosition].second));
    }

    MYVOXEL_ASSERT_MESSAGE(m_result.isEmpty(), "Voxelization result forest must be empty after root extraction.");
    MYVOXEL_ASSERT_MESSAGE(m_result.features().isEmpty() && m_result.hasCompleteFeatures(), "Voxelization result features must be empty after recycling.");
}

}
}
