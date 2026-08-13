#include "BlockPool.h"

#include <cassert>
#include <cmath>
#include <cstring>

namespace
{

// 将非分化节点数值编码为一个完整VoxelBlock的原始八字节数据。
std::uint64_t makeValueRaw(float value)
{
    assert(std::isfinite(static_cast<double>(value)));

    MyVoxel::NodeBlock block;
    block.mChildMask = 0;
    block.mValueMask = 0;
    block.reserved = 0;
    MyVoxel::setNodeValue(block, value);

    std::uint64_t rawValue = 0;
    static_assert(sizeof(rawValue) == sizeof(block), "VoxelBlock raw value size must match NodeBlock.");
    std::memcpy(&rawValue, &block, sizeof(rawValue));
    return rawValue;
}

}

namespace MyVoxel
{

BlockPool::BlockPool()
{
}

BlockPool::~BlockPool()
{
}

/// 节点组分配与回收

VoxelIndex BlockPool::allocateGroup(float value)
{
    assert(std::isfinite(static_cast<double>(value)));
    return m_nodePool.allocateGroup(makeValueRaw(value));
}

void BlockPool::initGroup(VoxelIndex firstSlotIndex, float value)
{
    assert(std::isfinite(static_cast<double>(value)));
    m_nodePool.initGroup(firstSlotIndex, makeValueRaw(value));
}

void BlockPool::releaseGroup(VoxelIndex firstSlotIndex)
{
    m_nodePool.releaseGroup(firstSlotIndex);
}

/// 非分化节点

void BlockPool::initializeValue(VoxelIndex index, float value)
{
    assert(m_nodePool.containsSlot(index));
    assert(std::isfinite(static_cast<double>(value)));
    m_nodePool.block(index).rawValue = makeValueRaw(value);
}

/// 普通节点初始化与访问

NodeBlock& BlockPool::initializeNode(VoxelIndex index, VoxelNodeState childState)
{
    return m_nodePool.initializeNode(index, childState);
}

/// MaskLeaf初始化与释放

LeafData BlockPool::initializeLeaf(VoxelIndex index, float distance)
{
    assert(m_nodePool.containsSlot(index));
    assert(std::isfinite(static_cast<double>(distance)));

    const VoxelIndex leafIndex = m_leafPool.allocateBlock(distance);

    try
    {
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