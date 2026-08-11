#include "BlockPool.h"

#include <cassert>

namespace MyVoxel
{

BlockPool::BlockPool()
{
}

BlockPool::~BlockPool()
{
}

/// 节点组分配与回收

VoxelIndex BlockPool::allocateGroup()
{
    return m_nodePool.allocateGroup();
}

void BlockPool::releaseGroup(VoxelIndex firstSlotIndex)
{
    m_nodePool.releaseGroup(firstSlotIndex);
}

/// 节点初始化

NodeBlock& BlockPool::initializeNode(VoxelIndex index, VoxelNodeState childState)
{
    return m_nodePool.initializeNode(index, childState);
}

/// MaskLeaf初始化与释放

LeafData BlockPool::initializeLeaf(VoxelIndex index, float distance)
{
    assert(m_nodePool.containsSlot(index));

    const VoxelIndex leafIndex = m_leafPool.allocateBlock();

    try
    {
        m_leafPool.initializeBlock(leafIndex, distance);
        m_nodePool.initializeIndex(index, leafIndex);
    }
    catch (...)
    {
        m_leafPool.releaseBlock(leafIndex);
        throw;
    }

    return leaf(index);
}

LeafData BlockPool::copyLeaf(VoxelIndex destinationIndex, const BlockPool& sourcePool, VoxelIndex sourceIndex)
{
    assert(m_nodePool.containsSlot(destinationIndex));
    assert(sourcePool.containsLeaf(sourceIndex));

    const IndexBlock& sourceIndexBlock = sourcePool.leafIndexBlock(sourceIndex);
    const VoxelIndex destinationLeafIndex = m_leafPool.allocateBlock();

    try
    {
        m_leafPool.copyBlock(destinationLeafIndex, sourcePool.m_leafPool, sourceIndexBlock.leafIndex);
        m_nodePool.initializeIndex(destinationIndex, destinationLeafIndex);
    }
    catch (...)
    {
        m_leafPool.releaseBlock(destinationLeafIndex);
        throw;
    }

    return leaf(destinationIndex);
}

void BlockPool::releaseLeaf(VoxelIndex index)
{
    assert(containsLeaf(index));

    IndexBlock& indexBlock = leafIndexBlock(index);
    const VoxelIndex leafIndex = indexBlock.leafIndex;

    m_leafPool.releaseBlock(leafIndex);
    reset(indexBlock);
}

/// 存储检查

bool BlockPool::containsGroup(VoxelIndex firstSlotIndex) const
{
    return m_nodePool.containsGroup(firstSlotIndex);
}

bool BlockPool::containsNodeSlot(VoxelIndex index) const
{
    return m_nodePool.containsSlot(index);
}

bool BlockPool::containsLeaf(VoxelIndex index) const
{
    if (!m_nodePool.containsSlot(index))
    {
        return false;
    }

    const IndexBlock& indexBlock = leafIndexBlock(index);

    if (indexBlock.reserved != 0 || indexBlock.leafIndex == InvalidVoxelIndex)
    {
        return false;
    }

    return m_leafPool.containsBlock(indexBlock.leafIndex);
}

/// 资源管理

void BlockPool::rewind()
{
    m_nodePool.rewind();
    m_leafPool.rewind();
}

void BlockPool::clear()
{
    m_nodePool.clear();
    m_leafPool.clear();
}

/// 节点存储统计

std::size_t BlockPool::allocatedGroupCount() const
{
    return m_nodePool.allocatedGroupCount();
}

std::size_t BlockPool::highWaterGroupCount() const
{
    return m_nodePool.highWaterGroupCount();
}

std::size_t BlockPool::nodeChunkCount() const
{
    return m_nodePool.chunkCount();
}

std::size_t BlockPool::nodeStorageCapacityBytes() const
{
    return m_nodePool.storageCapacityBytes();
}

/// 叶存储统计

std::size_t BlockPool::allocatedLeafCount() const
{
    return m_leafPool.allocatedBlockCount();
}

std::size_t BlockPool::highWaterLeafCount() const
{
    return m_leafPool.highWaterBlockCount();
}

std::size_t BlockPool::leafChunkCount() const
{
    return m_leafPool.chunkCount();
}

std::size_t BlockPool::leafStorageCapacityBytes() const
{
    return m_leafPool.storageCapacityBytes();
}

/// 总存储统计

std::size_t BlockPool::storageCapacityBytes() const
{
    return nodeStorageCapacityBytes() + leafStorageCapacityBytes();
}

}