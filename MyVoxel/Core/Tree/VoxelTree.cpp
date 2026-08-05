#include "VoxelTree.h"

#include <cassert>
#include <cstdint>
#include <unordered_set>
#include <utility>

#include "VoxelTreeCursor.h"
#include "VoxelTreeEditor.h"

namespace MyVoxel
{

namespace
{

// 将终止逻辑状态转换为普通节点存储状态。
VoxelNodeState terminalNodeState(VoxelState state)
{
    assert(state == VoxelState::Empty || state == VoxelState::Material);
    return state == VoxelState::Material ? VoxelNodeState::Material : VoxelNodeState::Empty;
}

// 将一个普通节点块及其全部物理后代复制到目标节点池。
void cloneSubtree(const VoxelNodeBlock& sourceBlock,
                  const VoxelBlockPool& sourcePool,
                  VoxelNodeBlock& targetBlock,
                  VoxelBlockPool& targetPool)
{
    assert(sourceBlock.reserved == 0);

    targetBlock.storageMask = sourceBlock.storageMask;
    targetBlock.leafMask = sourceBlock.leafMask;
    targetBlock.reserved = 0;
    targetBlock.firstChildIndex = InvalidVoxelIndex;

    if (sourceBlock.storageMask == 0)
    {
        assert(sourceBlock.firstChildIndex == InvalidVoxelIndex);
        return;
    }

    assert(sourceBlock.firstChildIndex != InvalidVoxelIndex);
    assert(sourcePool.containsGroup(sourceBlock.firstChildIndex));

    targetBlock.firstChildIndex = targetPool.allocateGroup();

    for (unsigned int cornerIndex = 0; cornerIndex < static_cast<unsigned int>(VoxelCornerCount); ++cornerIndex)
    {
        const VoxelCorner corner = static_cast<VoxelCorner>(cornerIndex);

        if (!hasChildStorage(sourceBlock, corner))
        {
            continue;
        }

        const VoxelNodeState state = nodeState(sourceBlock, corner);
        const VoxelIndex sourceIndex = childStorageIndex(sourceBlock, corner);
        const VoxelIndex targetIndex =
            targetBlock.firstChildIndex + static_cast<VoxelIndex>(corner);

        if (state == VoxelNodeState::MaskLeaf)
        {
            VoxelLeafBlock& targetLeaf = targetPool.initializeLeaf(targetIndex);
            targetLeaf.materialMask = sourcePool.leaf(sourceIndex).materialMask;
            continue;
        }

        assert(state == VoxelNodeState::Branch);

        VoxelNodeBlock& targetChild = targetPool.initializeNode(targetIndex);
        cloneSubtree(sourcePool.node(sourceIndex), sourcePool, targetChild, targetPool);
    }
}

// 检查一个普通节点块及其全部物理后代是否满足基本存储约束。
bool isNodeBlockValid(const VoxelNodeBlock& block,
                      const VoxelBlockPool& pool,
                      std::unordered_set<VoxelIndex>& visitedGroups)
{
    if (block.reserved != 0)
    {
        return false;
    }

    if (block.storageMask == 0)
    {
        return block.firstChildIndex == InvalidVoxelIndex;
    }

    if (block.firstChildIndex == InvalidVoxelIndex || !pool.containsGroup(block.firstChildIndex))
    {
        return false;
    }

    if (!visitedGroups.insert(block.firstChildIndex).second)
    {
        return false;
    }

    for (unsigned int cornerIndex = 0; cornerIndex < static_cast<unsigned int>(VoxelCornerCount); ++cornerIndex)
    {
        const VoxelCorner corner = static_cast<VoxelCorner>(cornerIndex);

        if (!hasChildStorage(block, corner))
        {
            continue;
        }

        const VoxelIndex storageIndex = childStorageIndex(block, corner);

        if (!pool.containsSlot(storageIndex))
        {
            return false;
        }

        const VoxelNodeState state = nodeState(block, corner);

        if (state == VoxelNodeState::Branch)
        {
            if (!isNodeBlockValid(pool.node(storageIndex), pool, visitedGroups))
            {
                return false;
            }

            continue;
        }

        if (state != VoxelNodeState::MaskLeaf)
        {
            return false;
        }
    }

    return true;
}

// 检查普通节点块及其后代是否已经消除全部可折叠状态。
bool isNodeBlockNormalized(const VoxelNodeBlock& block, const VoxelBlockPool& pool)
{
    if (block.storageMask == 0)
    {
        return block.leafMask != static_cast<std::uint8_t>(0) &&
               block.leafMask != static_cast<std::uint8_t>(0xFFU);
    }

    for (unsigned int cornerIndex = 0; cornerIndex < static_cast<unsigned int>(VoxelCornerCount); ++cornerIndex)
    {
        const VoxelCorner corner = static_cast<VoxelCorner>(cornerIndex);
        const VoxelNodeState state = nodeState(block, corner);

        if (state == VoxelNodeState::Branch)
        {
            const VoxelIndex storageIndex = childStorageIndex(block, corner);

            if (!isNodeBlockNormalized(pool.node(storageIndex), pool))
            {
                return false;
            }

            continue;
        }

        if (state == VoxelNodeState::MaskLeaf)
        {
            const VoxelIndex storageIndex = childStorageIndex(block, corner);
            const std::uint64_t materialMask = pool.leaf(storageIndex).materialMask;

            if (materialMask == static_cast<std::uint64_t>(0) ||
                materialMask == ~static_cast<std::uint64_t>(0))
            {
                return false;
            }
        }
    }

    return true;
}

}

VoxelTree::VoxelTree(VoxelState state)
    : m_rootState(state)
    , m_blockPool(Foundation::makeRef<VoxelBlockPool>())
{
    assert(state == VoxelState::Empty || state == VoxelState::Material);
    MyVoxel::reset(m_rootBlock, terminalNodeState(state));
}

VoxelTree::VoxelTree(VoxelTree&& other)
    : m_rootState(other.m_rootState)
    , m_rootBlock(other.m_rootBlock)
    , m_blockPool(std::move(other.m_blockPool))
{
    other.m_rootState = VoxelState::Empty;
    MyVoxel::reset(other.m_rootBlock, VoxelNodeState::Empty);
}

VoxelTree& VoxelTree::operator=(VoxelTree&& other)
{
    if (this == &other)
    {
        return *this;
    }

    m_rootState = other.m_rootState;
    m_rootBlock = other.m_rootBlock;
    m_blockPool = std::move(other.m_blockPool);

    other.m_rootState = VoxelState::Empty;
    MyVoxel::reset(other.m_rootBlock, VoxelNodeState::Empty);

    return *this;
}

/// 根节点状态

VoxelState VoxelTree::state() const
{
    return m_rootState;
}

/// 树访问入口

VoxelTreeCursor VoxelTree::cursor() const
{
    return VoxelTreeCursor(*this);
}

VoxelTreeEditor VoxelTree::editor()
{
    return VoxelTreeEditor(*this);
}

/// 资源管理

void VoxelTree::reset(VoxelState state)
{
    assert(state == VoxelState::Empty || state == VoxelState::Material);

    if (!m_blockPool || m_blockPool->referenceCount() > 1)
    {
        m_blockPool = Foundation::makeRef<VoxelBlockPool>();
    }
    else
    {
        m_blockPool->rewind();
    }

    m_rootState = state;
    MyVoxel::reset(m_rootBlock, terminalNodeState(state));

    assert(m_blockPool->allocatedGroupCount() == 0);
    assert(isValid());
}

void VoxelTree::release(VoxelState state)
{
    assert(state == VoxelState::Empty || state == VoxelState::Material);

    // 直接切换到新的空节点池。旧节点池为独占时立即释放Chunk，为共享时只释放当前引用。
    m_rootState = state;
    MyVoxel::reset(m_rootBlock, terminalNodeState(state));
    m_blockPool = Foundation::makeRef<VoxelBlockPool>();

    assert(isValid());
}

/// 存储统计

std::size_t VoxelTree::allocatedGroupCount() const
{
    return m_blockPool ? m_blockPool->allocatedGroupCount() : 0;
}

std::size_t VoxelTree::highWaterGroupCount() const
{
    return m_blockPool ? m_blockPool->highWaterGroupCount() : 0;
}

std::size_t VoxelTree::chunkCount() const
{
    return m_blockPool ? m_blockPool->chunkCount() : 0;
}

std::size_t VoxelTree::storageCapacityBytes() const
{
    return chunkCount() *
           static_cast<std::size_t>(VoxelBlockPool::SlotsPerChunk) *
           sizeof(VoxelBlock);
}

/// 结构检查

bool VoxelTree::isValid() const
{
    if (!m_blockPool)
    {
        return false;
    }

    if (m_rootBlock.reserved != 0)
    {
        return false;
    }

    if (m_rootState == VoxelState::Empty)
    {
        return m_rootBlock.storageMask == 0 &&
               m_rootBlock.leafMask == static_cast<std::uint8_t>(0) &&
               m_rootBlock.firstChildIndex == InvalidVoxelIndex &&
               m_blockPool->allocatedGroupCount() == 0;
    }

    if (m_rootState == VoxelState::Material)
    {
        return m_rootBlock.storageMask == 0 &&
               m_rootBlock.leafMask == static_cast<std::uint8_t>(0xFFU) &&
               m_rootBlock.firstChildIndex == InvalidVoxelIndex &&
               m_blockPool->allocatedGroupCount() == 0;
    }

    if (m_rootState != VoxelState::Subdivided)
    {
        return false;
    }

    std::unordered_set<VoxelIndex> visitedGroups;

    if (!isNodeBlockValid(m_rootBlock, *m_blockPool, visitedGroups))
    {
        return false;
    }

    return visitedGroups.size() == m_blockPool->allocatedGroupCount();
}

bool VoxelTree::isNormalized() const
{
    if (!isValid())
    {
        return false;
    }

    if (m_rootState == VoxelState::Empty || m_rootState == VoxelState::Material)
    {
        return true;
    }

    return isNodeBlockNormalized(m_rootBlock, *m_blockPool);
}

/// 内部辅助

void VoxelTree::ensureUniqueStorage()
{
    assert(m_blockPool);

    if (m_blockPool->referenceCount() <= 1)
    {
        return;
    }

    Foundation::RefPtr<VoxelBlockPool> detachedPool =Foundation::makeRef<VoxelBlockPool>();

    VoxelNodeBlock detachedRootBlock;

    if (m_rootState == VoxelState::Subdivided)
    {
        cloneSubtree(m_rootBlock, *m_blockPool, detachedRootBlock, *detachedPool);
    }
    else
    {
        MyVoxel::reset(detachedRootBlock, terminalNodeState(m_rootState));
    }

    m_rootBlock = detachedRootBlock;
    m_blockPool = detachedPool;

    assert(isValid());
}

}