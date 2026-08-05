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
        // 外部仍持有当前结果时不能回收其根树，否则会触发节点池复制或破坏快照语义。
        const VoxelGrid resultGrid = m_result.grid();
        const MyMath::Matrix4 resultTransform = m_result.transform();

        m_result = VoxelShape(resultGrid);
        m_result.setTransform(resultTransform);
        return;
    }

    recycleResultRoots();

    MYVOXEL_ASSERT_MESSAGE(m_result.isEmpty(), "VoxelizationWorkspace reset result must be empty.");
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
        chunkCount += m_cachedRootTrees[rootPosition].second.chunkCount();
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

            chunkCount += tree->chunkCount();
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

    // reference可能直接引用m_result，因此必须在修改m_result之前保存参考空间参数。
    const VoxelGrid referenceGrid = reference.grid();
    const MyMath::Matrix4 referenceTransform = reference.transform();

    const bool resultShared = m_initialized && m_result.isDataShared();
    const bool sameGrid = m_initialized && m_result.grid().isEqualTo(referenceGrid, 0.0);

    if (m_initialized && !resultShared)
    {
        recycleResultRoots();
    }

    if (!m_initialized || resultShared || !sameGrid)
    {
        m_result = VoxelShape(referenceGrid);
    }
    else
    {
        MYVOXEL_ASSERT_MESSAGE(m_result.isEmpty(), "Reusable VoxelizationWorkspace result must be empty before voxelization.");
    }

    m_result.setTransform(referenceTransform);
    m_initialized = true;
}

VoxelTree VoxelizationWorkspace::acquireRootTree(const VoxelCellIndex& rootIndex)
{
    // 优先复用上一轮属于相同根索引的节点池。
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
        ++m_exactRootReuseCount;

        MYVOXEL_ASSERT_MESSAGE(tree.isValid(), "Acquired cached root tree must be valid.");
        MYVOXEL_ASSERT_MESSAGE(tree.state() == VoxelState::Empty, "Acquired cached root tree must be empty.");
        return tree;
    }

    if (m_cachedRootTrees.empty())
    {
        ++m_createdRootTreeCount;
        return VoxelTree(VoxelState::Empty);
    }

    // 新出现的根无法预先确定节点需求，优先使用最大缓存池可以避免大型池闲置时较小池继续扩容。
    std::size_t largestPosition = 0;

    for (std::size_t rootPosition = 1; rootPosition < m_cachedRootTrees.size(); ++rootPosition)
    {
        const VoxelTree& currentTree = m_cachedRootTrees[rootPosition].second;
        const VoxelTree& largestTree = m_cachedRootTrees[largestPosition].second;

        if (currentTree.chunkCount() > largestTree.chunkCount())
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
    ++m_fallbackRootReuseCount;

    MYVOXEL_ASSERT_MESSAGE(tree.isValid(), "Acquired cached root tree must be valid.");
    MYVOXEL_ASSERT_MESSAGE(tree.state() == VoxelState::Empty, "Acquired cached root tree must be empty.");
    return tree;
}

void VoxelizationWorkspace::recycleRootTree(const VoxelCellIndex& rootIndex, VoxelTree&& tree)
{
    if (tree.chunkCount() == 0)
    {
        return;
    }

    tree.reset();

    MYVOXEL_ASSERT_MESSAGE(tree.isValid(), "Recycled root tree must remain valid.");
    MYVOXEL_ASSERT_MESSAGE(tree.state() == VoxelState::Empty, "Recycled root tree must be empty.");
    MYVOXEL_ASSERT_MESSAGE(tree.allocatedGroupCount() == 0, "Recycled root tree must not retain allocated groups.");

    m_cachedRootTrees.push_back(VoxelForest::TreeEntry(rootIndex, std::move(tree)));
}

void VoxelizationWorkspace::recycleResultRoots()
{
    if (m_result.isEmpty())
    {
        return;
    }

    MYVOXEL_ASSERT_MESSAGE(!m_result.isDataShared(), "Shared voxelization result cannot be recycled in place.");

    std::vector<VoxelForest::TreeEntry> extractedRoots;

    {
        VoxelShapeSession session =
            m_result.session(BaseVoxelLevel);

        session.moveTreesTo(extractedRoots);
    }

    for (std::size_t rootPosition = 0; rootPosition < extractedRoots.size(); ++rootPosition)
    {
        recycleRootTree(extractedRoots[rootPosition].first, std::move(extractedRoots[rootPosition].second));
    }

    MYVOXEL_ASSERT_MESSAGE(m_result.isEmpty(), "Voxelization result forest must be empty after root extraction.");
}

}
}