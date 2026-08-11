#include "VoxelTree.h"

#include <cassert>
#include <cstdint>
#include <limits>
#include <unordered_set>
#include <utility>

#include "VoxelTreeCursor.h"
#include "VoxelTreeEditor.h"
#include "MyVoxel/Core/Mask/VoxelNodeMask.h"

namespace MyVoxel
{

namespace
{

// 将终止逻辑状态转换为NodeBlock中的物理状态。
VoxelNodeState terminalNodeState(VoxelState state)
{
    assert(state == VoxelState::Empty || state == VoxelState::Material);
    return state == VoxelState::Material ? VoxelNodeState::Material : VoxelNodeState::Empty;
}

// 判断MaskLeaf是否可以无损折叠为Empty或Material。
// 只有全部距离为对应正负无穷时才允许折叠，有限距离即使符号一致仍保留几何信息。
bool leafTerminalState(const ConstLeafData& leaf, VoxelState& state)
{
    assert(leaf.mask);
    assert(leaf.block);

    const float infinity = (std::numeric_limits<float>::infinity)();
    bool allEmpty = true;
    bool allMaterial = true;

    for (unsigned int sampleIndex = 0; sampleIndex < LeafBlockSampleCount; ++sampleIndex)
    {
        const float distance = leaf.block->distances[sampleIndex];

        if (distance != infinity)
        {
            allEmpty = false;
        }

        if (distance != -infinity)
        {
            allMaterial = false;
        }

        if (!allEmpty && !allMaterial)
        {
            return false;
        }
    }

    state = allMaterial ? VoxelState::Material : VoxelState::Empty;
    return true;
}

// 将一个NodeBlock及其全部物理后代复制到目标BlockPool。
void cloneSubtree(const NodeBlock& sourceBlock, const BlockPool& sourcePool, NodeBlock& targetBlock, BlockPool& targetPool)
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
        const VoxelIndex targetIndex = targetBlock.firstChildIndex + static_cast<VoxelIndex>(corner);

        if (state == VoxelNodeState::MaskLeaf)
        {
            targetPool.copyLeaf(targetIndex, sourcePool, sourceIndex);
            continue;
        }

        assert(state == VoxelNodeState::Node);

        NodeBlock& targetChild = targetPool.initializeNode(targetIndex);
        cloneSubtree(sourcePool.node(sourceIndex), sourcePool, targetChild, targetPool);
    }
}

// 检查一个NodeBlock及全部物理后代是否满足基本存储约束。
bool isNodeBlockValid(const NodeBlock& block, const BlockPool& pool, std::unordered_set<VoxelIndex>& visitedGroups, std::size_t& reachableLeafCount)
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

        if (!pool.containsNodeSlot(storageIndex))
        {
            return false;
        }

        const VoxelNodeState state = nodeState(block, corner);

        if (state == VoxelNodeState::Node)
        {
            if (!isNodeBlockValid(pool.node(storageIndex), pool, visitedGroups, reachableLeafCount))
            {
                return false;
            }

            continue;
        }

        if (state != VoxelNodeState::MaskLeaf || !pool.containsLeaf(storageIndex))
        {
            return false;
        }

        const ConstLeafData leaf = pool.leaf(storageIndex);

        if (!isMaskConsistent(*leaf.mask, *leaf.block))
        {
            return false;
        }

        ++reachableLeafCount;
    }

    return true;
}

// 检查NodeBlock及其后代是否已经消除全部可无损折叠状态。
bool isNodeBlockNormalized(const NodeBlock& block, const BlockPool& pool)
{
    if (block.storageMask == 0)
    {
        return block.leafMask != EmptyVoxelNodeMask && block.leafMask != FullVoxelNodeMask;
    }

    for (unsigned int cornerIndex = 0; cornerIndex < static_cast<unsigned int>(VoxelCornerCount); ++cornerIndex)
    {
        const VoxelCorner corner = static_cast<VoxelCorner>(cornerIndex);
        const VoxelNodeState state = nodeState(block, corner);

        if (state == VoxelNodeState::Node)
        {
            if (!isNodeBlockNormalized(pool.node(childStorageIndex(block, corner)), pool))
            {
                return false;
            }

            continue;
        }

        if (state == VoxelNodeState::MaskLeaf)
        {
            VoxelState terminalState = VoxelState::Empty;

            if (leafTerminalState(pool.leaf(childStorageIndex(block, corner)), terminalState))
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
    , m_blockPool(Foundation::makeRef<BlockPool>())
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
        m_blockPool = Foundation::makeRef<BlockPool>();
    }
    else
    {
        m_blockPool->rewind();
    }

    m_rootState = state;
    MyVoxel::reset(m_rootBlock, terminalNodeState(state));

    assert(m_blockPool->allocatedGroupCount() == 0);
    assert(m_blockPool->allocatedLeafCount() == 0);
    assert(isValid());
}

void VoxelTree::release(VoxelState state)
{
    assert(state == VoxelState::Empty || state == VoxelState::Material);

    m_rootState = state;
    MyVoxel::reset(m_rootBlock, terminalNodeState(state));
    m_blockPool = Foundation::makeRef<BlockPool>();

    assert(isValid());
}

/// 节点存储统计

std::size_t VoxelTree::allocatedGroupCount() const
{
    return m_blockPool ? m_blockPool->allocatedGroupCount() : 0;
}

std::size_t VoxelTree::highWaterGroupCount() const
{
    return m_blockPool ? m_blockPool->highWaterGroupCount() : 0;
}

std::size_t VoxelTree::nodeChunkCount() const
{
    return m_blockPool ? m_blockPool->nodeChunkCount() : 0;
}

std::size_t VoxelTree::nodeStorageCapacityBytes() const
{
    return m_blockPool ? m_blockPool->nodeStorageCapacityBytes() : 0;
}

/// 叶存储统计

std::size_t VoxelTree::allocatedLeafCount() const
{
    return m_blockPool ? m_blockPool->allocatedLeafCount() : 0;
}

std::size_t VoxelTree::highWaterLeafCount() const
{
    return m_blockPool ? m_blockPool->highWaterLeafCount() : 0;
}

std::size_t VoxelTree::leafChunkCount() const
{
    return m_blockPool ? m_blockPool->leafChunkCount() : 0;
}

std::size_t VoxelTree::leafStorageCapacityBytes() const
{
    return m_blockPool ? m_blockPool->leafStorageCapacityBytes() : 0;
}

/// 总存储统计

std::size_t VoxelTree::storageCapacityBytes() const
{
    return m_blockPool ? m_blockPool->storageCapacityBytes() : 0;
}

/// 结构检查

bool VoxelTree::isValid() const
{
    if (!m_blockPool || m_rootBlock.reserved != 0)
    {
        return false;
    }

    if (m_rootState == VoxelState::Empty)
    {
        return m_rootBlock.storageMask == 0 &&
               m_rootBlock.leafMask == EmptyVoxelNodeMask &&
               m_rootBlock.firstChildIndex == InvalidVoxelIndex &&
               m_blockPool->allocatedGroupCount() == 0 &&
               m_blockPool->allocatedLeafCount() == 0;
    }

    if (m_rootState == VoxelState::Material)
    {
        return m_rootBlock.storageMask == 0 &&
               m_rootBlock.leafMask == FullVoxelNodeMask &&
               m_rootBlock.firstChildIndex == InvalidVoxelIndex &&
               m_blockPool->allocatedGroupCount() == 0 &&
               m_blockPool->allocatedLeafCount() == 0;
    }

    if (m_rootState != VoxelState::Subdivided)
    {
        return false;
    }

    std::unordered_set<VoxelIndex> visitedGroups;
    std::size_t reachableLeafCount = 0;

    if (!isNodeBlockValid(m_rootBlock, *m_blockPool, visitedGroups, reachableLeafCount))
    {
        return false;
    }

    return visitedGroups.size() == m_blockPool->allocatedGroupCount() &&
           reachableLeafCount == m_blockPool->allocatedLeafCount();
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

    Foundation::RefPtr<BlockPool> detachedPool = Foundation::makeRef<BlockPool>();
    NodeBlock detachedRootBlock;

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