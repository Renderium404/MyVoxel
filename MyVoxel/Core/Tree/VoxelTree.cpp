#include "VoxelTree.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <unordered_set>
#include <utility>

#include "VoxelTreeCursor.h"
#include "VoxelTreeEditor.h"
#include "MyVoxel/Core/Mask/VoxelLeafMask.h"
#include "MyVoxel/Core/Mask/VoxelNodeMask.h"
#include "MyVoxel/Foundation/Diagnostic.h"

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

// 根据终止Value返回对应二值状态，零值按照材料语义处理。
VoxelState terminalState(float value)
{
    return value <= 0.0f ? VoxelState::Material : VoxelState::Empty;
}

// 返回兼容旧独立VoxelTree构造的默认终止Value。
float defaultTerminalValue(VoxelState state)
{
    assert(state == VoxelState::Empty || state == VoxelState::Material);
    return state == VoxelState::Material ? -1.0f : 1.0f;
}

// 检查终止状态与Value符号是否一致。
bool isTerminalValueValid(VoxelState state, float value)
{
    if (!std::isfinite(static_cast<double>(value)))
    {
        return false;
    }
    return state == VoxelState::Material ? value <= 0.0f : value > 0.0f;
}

// 将NodeBlock设置为根节点使用的终止Tile Value表示。
void resetTerminalBlock(NodeBlock& block, float value)
{
    block.mChildMask = 0;
    block.mValueMask = 0;
    block.reserved = 0;
    block.nodeData.value = value;
}

// 检查backgroundDistance是否满足截断距离场基本约束。
bool isValidBackgroundDistance(float backgroundDistance)
{
    return std::isfinite(static_cast<double>(backgroundDistance)) && backgroundDistance > 0.0f;
}

// 检查Value是否位于指定TSDF截断范围。
bool isValidTsdfValue(float value, float backgroundDistance)
{
    return std::isfinite(static_cast<double>(value)) && value >= -backgroundDistance && value <= backgroundDistance;
}

// 判断LeafBlock是否能够无损折叠为单一Tile Value。
bool leafUniformValue(const LeafBlock& block, float& value)
{
    value = block.distances[0];
    if (!std::isfinite(static_cast<double>(value)))
    {
        return false;
    }

    for (unsigned int sampleIndex = 1; sampleIndex < LeafBlockSampleCount; ++sampleIndex)
    {
        if (block.distances[sampleIndex] != value)
        {
            return false;
        }
    }
    return true;
}

// 将一个NodeBlock及其完整八槽子表和全部物理后代复制到目标BlockPool。
void cloneSubtree(const NodeBlock& sourceBlock, const BlockPool& sourcePool, NodeBlock& targetBlock, BlockPool& targetPool)
{
    assert(sourceBlock.reserved == 0);
    assert(hasChildStorage(sourceBlock));
    assert(sourcePool.containsGroup(sourceBlock.nodeData.firstChildIndex));

    targetBlock.mChildMask = sourceBlock.mChildMask;
    targetBlock.mValueMask = sourceBlock.mValueMask;
    targetBlock.reserved = 0;
    targetBlock.nodeData.firstChildIndex = targetPool.allocateGroup(0.0f);

    for (unsigned int cornerIndex = 0; cornerIndex < static_cast<unsigned int>(VoxelCornerCount); ++cornerIndex)
    {
        const VoxelCorner corner = static_cast<VoxelCorner>(cornerIndex);
        const VoxelNodeState state = nodeState(sourceBlock, corner);
        const VoxelIndex sourceIndex = childStorageIndex(sourceBlock, corner);
        const VoxelIndex targetIndex = childStorageIndex(targetBlock, corner);

        if (state == VoxelNodeState::Empty || state == VoxelNodeState::Material)
        {
            targetPool.initializeValue(targetIndex, sourcePool.value(sourceIndex));
            continue;
        }

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

// 检查一个NodeBlock及全部物理后代是否满足基本存储和Value状态约束。
bool isNodeBlockValid(const NodeBlock& block, const BlockPool& pool, std::unordered_set<VoxelIndex>& visitedGroups, std::size_t& reachableLeafCount)
{
    if (block.reserved != 0 || !hasChildStorage(block) || !pool.containsGroup(block.nodeData.firstChildIndex))
    {
        return false;
    }

    if (!visitedGroups.insert(block.nodeData.firstChildIndex).second)
    {
        return false;
    }

    for (unsigned int cornerIndex = 0; cornerIndex < static_cast<unsigned int>(VoxelCornerCount); ++cornerIndex)
    {
        const VoxelCorner corner = static_cast<VoxelCorner>(cornerIndex);
        const VoxelIndex storageIndex = childStorageIndex(block, corner);
        if (!pool.containsNodeSlot(storageIndex))
        {
            return false;
        }

        const VoxelNodeState state = nodeState(block, corner);
        if (state == VoxelNodeState::Empty || state == VoxelNodeState::Material)
        {
            if (!isTerminalValueValid(voxelState(state), pool.value(storageIndex)))
            {
                return false;
            }
            continue;
        }

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
        if (!isValid(*leaf.block) || !isMaskConsistent(*leaf.mask, *leaf.block))
        {
            return false;
        }
        ++reachableLeafCount;
    }

    return true;
}

// 检查一个NodeBlock全部Tile Value和显式叶距离是否位于指定TSDF截断范围。
bool isNodeBlockTsdfValid(const NodeBlock& block, const BlockPool& pool, float backgroundDistance)
{
    for (unsigned int cornerIndex = 0; cornerIndex < static_cast<unsigned int>(VoxelCornerCount); ++cornerIndex)
    {
        const VoxelCorner corner = static_cast<VoxelCorner>(cornerIndex);
        const VoxelNodeState state = nodeState(block, corner);
        const VoxelIndex storageIndex = childStorageIndex(block, corner);

        if (state == VoxelNodeState::Empty || state == VoxelNodeState::Material)
        {
            if (!isValidTsdfValue(pool.value(storageIndex), backgroundDistance))
            {
                return false;
            }
            continue;
        }

        if (state == VoxelNodeState::Node)
        {
            if (!isNodeBlockTsdfValid(pool.node(storageIndex), pool, backgroundDistance))
            {
                return false;
            }
            continue;
        }

        const ConstLeafData leaf = pool.leaf(storageIndex);
        for (unsigned int sampleIndex = 0; sampleIndex < LeafBlockSampleCount; ++sampleIndex)
        {
            if (!isValidTsdfValue(leaf.block->distances[sampleIndex], backgroundDistance))
            {
                return false;
            }
        }
    }
    return true;
}

// 判断NodeBlock的八个直接子项是否已经能够无损归约为一个终止Tile Value。
bool nodeUniformValue(const NodeBlock& block, const BlockPool& pool, VoxelState& state, float& value)
{
    const VoxelCorner firstCorner = static_cast<VoxelCorner>(0);
    const VoxelNodeState firstState = nodeState(block, firstCorner);
    if (firstState != VoxelNodeState::Empty && firstState != VoxelNodeState::Material)
    {
        return false;
    }

    state = voxelState(firstState);
    value = pool.value(childStorageIndex(block, firstCorner));

    for (unsigned int cornerIndex = 1; cornerIndex < static_cast<unsigned int>(VoxelCornerCount); ++cornerIndex)
    {
        const VoxelCorner corner = static_cast<VoxelCorner>(cornerIndex);
        const VoxelNodeState childState = nodeState(block, corner);
        if (childState != firstState || pool.value(childStorageIndex(block, corner)) != value)
        {
            return false;
        }
    }
    return true;
}

// bottom-up裁剪一个NodeBlock，返回当前Node是否能够继续由父节点无损归约为一个终止Tile Value。
bool pruneNodeBlock(NodeBlock& block, BlockPool& pool, VoxelState& collapsedState, float& collapsedValue, bool& changed)
{
    std::uint8_t subdividedMask = block.mChildMask;
    while (subdividedMask != EmptyVoxelNodeMask)
    {
        const VoxelCorner corner = takeFirstNodeCorner(subdividedMask);
        const VoxelNodeState state = nodeState(block, corner);
        const VoxelIndex storageIndex = childStorageIndex(block, corner);

        if (state == VoxelNodeState::Node)
        {
            NodeBlock& childBlock = pool.node(storageIndex);
            VoxelState childState = VoxelState::Empty;
            float childValue = 0.0f;
            if (pruneNodeBlock(childBlock, pool, childState, childValue, changed))
            {
                const VoxelIndex childGroupIndex = childBlock.nodeData.firstChildIndex;
                pool.releaseGroup(childGroupIndex);
                pool.initializeValue(storageIndex, childValue);
                setNodeState(block, corner, terminalNodeState(childState));
                changed = true;
            }
            continue;
        }

        assert(state == VoxelNodeState::MaskLeaf);
        const LeafData leaf = pool.leaf(storageIndex);
        float leafValue = 0.0f;
        if (leafUniformValue(*leaf.block, leafValue))
        {
            const VoxelState leafState = terminalState(leafValue);
            pool.releaseLeaf(storageIndex);
            pool.initializeValue(storageIndex, leafValue);
            setNodeState(block, corner, terminalNodeState(leafState));
            changed = true;
        }
    }

    return nodeUniformValue(block, pool, collapsedState, collapsedValue);
}

// 检查NodeBlock及其后代是否已经消除全部可无损折叠状态。
bool isNodeBlockNormalized(const NodeBlock& block, const BlockPool& pool)
{
    for (unsigned int cornerIndex = 0; cornerIndex < static_cast<unsigned int>(VoxelCornerCount); ++cornerIndex)
    {
        const VoxelCorner corner = static_cast<VoxelCorner>(cornerIndex);
        const VoxelNodeState state = nodeState(block, corner);
        const VoxelIndex storageIndex = childStorageIndex(block, corner);

        if (state == VoxelNodeState::Node)
        {
            if (!isNodeBlockNormalized(pool.node(storageIndex), pool))
            {
                return false;
            }
            continue;
        }

        if (state == VoxelNodeState::MaskLeaf)
        {
            const ConstLeafData leaf = pool.leaf(storageIndex);
            float value = 0.0f;
            if (leafUniformValue(*leaf.block, value))
            {
                return false;
            }
        }
    }

    VoxelState state = VoxelState::Empty;
    float value = 0.0f;
    return !nodeUniformValue(block, pool, state, value);
}

}

VoxelTree::VoxelTree(VoxelState state)
    : m_rootState(state)
    , m_blockPool(Foundation::makeRef<BlockPool>())
{
    assert(state == VoxelState::Empty || state == VoxelState::Material);
    resetTerminalBlock(m_rootBlock, defaultTerminalValue(state));
}

VoxelTree::VoxelTree(VoxelState state, float value)
    : m_rootState(state)
    , m_blockPool(Foundation::makeRef<BlockPool>())
{
    assert(state == VoxelState::Empty || state == VoxelState::Material);
    assert(isTerminalValueValid(state, value));
    resetTerminalBlock(m_rootBlock, value);
}

VoxelTree::VoxelTree(VoxelTree&& other)
    : m_rootState(other.m_rootState)
    , m_rootBlock(other.m_rootBlock)
    , m_blockPool(std::move(other.m_blockPool))
{
    other.m_rootState = VoxelState::Empty;
    resetTerminalBlock(other.m_rootBlock, 1.0f);
    other.m_blockPool = Foundation::makeRef<BlockPool>();
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
    resetTerminalBlock(other.m_rootBlock, 1.0f);
    other.m_blockPool = Foundation::makeRef<BlockPool>();
    return *this;
}

/// 根节点状态

VoxelState VoxelTree::state() const
{
    return m_rootState;
}

float VoxelTree::value() const
{
    assert(m_rootState == VoxelState::Empty || m_rootState == VoxelState::Material);
    return nodeValue(m_rootBlock);
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

VoxelTreeEditor VoxelTree::editor(float backgroundDistance)
{
    return VoxelTreeEditor(*this, backgroundDistance);
}

/// 资源管理

void VoxelTree::reset(VoxelState state)
{
    reset(state, defaultTerminalValue(state));
}

void VoxelTree::reset(VoxelState state, float value)
{
    assert(state == VoxelState::Empty || state == VoxelState::Material);
    assert(isTerminalValueValid(state, value));

    if (!m_blockPool || m_blockPool->referenceCount() > 1)
    {
        m_blockPool = Foundation::makeRef<BlockPool>();
    }
    else
    {
        m_blockPool->rewind();
    }

    m_rootState = state;
    resetTerminalBlock(m_rootBlock, value);
    assert(m_blockPool->allocatedGroupCount() == 0);
    assert(m_blockPool->allocatedLeafCount() == 0);
    assert(isValid());
}

void VoxelTree::release(VoxelState state)
{
    release(state, defaultTerminalValue(state));
}

void VoxelTree::release(VoxelState state, float value)
{
    assert(state == VoxelState::Empty || state == VoxelState::Material);
    assert(isTerminalValueValid(state, value));

    m_rootState = state;
    resetTerminalBlock(m_rootBlock, value);
    m_blockPool = Foundation::makeRef<BlockPool>();
    assert(isValid());
}

/// 节点存储统计

std::size_t VoxelTree::allocatedGroupCount() const{return m_blockPool ? m_blockPool->allocatedGroupCount() : 0;}
std::size_t VoxelTree::highWaterGroupCount() const{return m_blockPool ? m_blockPool->highWaterGroupCount() : 0;}
std::size_t VoxelTree::nodeChunkCount() const{return m_blockPool ? m_blockPool->nodeChunkCount() : 0;}
std::size_t VoxelTree::nodeStorageCapacityBytes() const{return m_blockPool ? m_blockPool->nodeStorageCapacityBytes() : 0;}

/// 叶存储统计

std::size_t VoxelTree::allocatedLeafCount() const{return m_blockPool ? m_blockPool->allocatedLeafCount() : 0;}
std::size_t VoxelTree::highWaterLeafCount() const{return m_blockPool ? m_blockPool->highWaterLeafCount() : 0;}
std::size_t VoxelTree::leafChunkCount() const{return m_blockPool ? m_blockPool->leafChunkCount() : 0;}
std::size_t VoxelTree::leafStorageCapacityBytes() const{return m_blockPool ? m_blockPool->leafStorageCapacityBytes() : 0;}

/// 总存储统计

std::size_t VoxelTree::storageCapacityBytes() const{return m_blockPool ? m_blockPool->storageCapacityBytes() : 0;}

/// 结构检查

bool VoxelTree::isValid() const
{
    if (!m_blockPool || m_rootBlock.reserved != 0)
    {
        return false;
    }

    if (m_rootState == VoxelState::Empty || m_rootState == VoxelState::Material)
    {
        return m_rootBlock.mChildMask == 0 &&
               m_rootBlock.mValueMask == 0 &&
               isTerminalValueValid(m_rootState, nodeValue(m_rootBlock)) &&
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

bool VoxelTree::isTsdfValid(float backgroundDistance) const
{
    if (!isValid() || !isValidBackgroundDistance(backgroundDistance))
    {
        return false;
    }

    if (m_rootState == VoxelState::Empty || m_rootState == VoxelState::Material)
    {
        return isValidTsdfValue(nodeValue(m_rootBlock), backgroundDistance);
    }

    return isNodeBlockTsdfValid(m_rootBlock, *m_blockPool, backgroundDistance);
}

bool VoxelTree::isNormalized(float backgroundDistance) const
{
    if (!isTsdfValid(backgroundDistance))
    {
        return false;
    }

    if (m_rootState == VoxelState::Empty || m_rootState == VoxelState::Material)
    {
        return true;
    }

    return isNodeBlockNormalized(m_rootBlock, *m_blockPool);
}

/// TSDF裁剪

bool VoxelTree::prune(float backgroundDistance)
{
    MYVOXEL_ASSERT_MESSAGE(isTsdfValid(backgroundDistance), "Cannot prune an invalid TSDF VoxelTree.");
    if (m_rootState != VoxelState::Subdivided)
    {
        return false;
    }

    ensureUniqueStorage();

    bool changed = false;
    VoxelState collapsedState = VoxelState::Empty;
    float collapsedValue = 0.0f;
    if (pruneNodeBlock(m_rootBlock, *m_blockPool, collapsedState, collapsedValue, changed))
    {
        const VoxelIndex rootGroupIndex = m_rootBlock.nodeData.firstChildIndex;
        m_blockPool->releaseGroup(rootGroupIndex);
        m_rootState = collapsedState;
        resetTerminalBlock(m_rootBlock, collapsedValue);
        changed = true;
    }

    MYVOXEL_ASSERT_MESSAGE(isTsdfValid(backgroundDistance), "TSDF pruning produced an invalid VoxelTree.");
    MYVOXEL_ASSERT_MESSAGE(isNormalized(backgroundDistance), "TSDF pruning failed to normalize the VoxelTree.");
    return changed;
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
        resetTerminalBlock(detachedRootBlock, nodeValue(m_rootBlock));
    }

    m_rootBlock = detachedRootBlock;
    m_blockPool = detachedPool;
    assert(isValid());
}

}