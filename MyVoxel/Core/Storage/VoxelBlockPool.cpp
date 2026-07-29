#include "VoxelBlockPool.h"

#include <cassert>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <new>
#include <utility>

#ifdef _MSC_VER
#include <malloc.h>
#endif

namespace
{

// 分配指定字节数的对齐内存，VS2013使用_aligned_malloc。
void* allocateAlignedMemory(std::size_t byteCount, std::size_t alignment)
{
#ifdef _MSC_VER
    return _aligned_malloc(byteCount, alignment);
#else
    void* memory = nullptr;
    return posix_memalign(&memory, alignment, byteCount) == 0 ? memory : nullptr;
#endif
}

// 释放由allocateAlignedMemory创建的对齐内存。
void releaseAlignedMemory(void* memory)
{
#ifdef _MSC_VER
    _aligned_free(memory);
#else
    std::free(memory);
#endif
}

}

namespace MyVoxel
{

VoxelBlockPool::VoxelBlockPool()
    : m_nextNodeIndex(0)
    , m_allocatedGroupCount(0)
{
}

VoxelBlockPool::VoxelBlockPool(const VoxelBlockPool& other)
    : m_nextNodeIndex(other.m_nextNodeIndex)
    , m_allocatedGroupCount(other.m_allocatedGroupCount)
{
    try
    {
        for (std::size_t chunkIndex = 0; chunkIndex < other.m_chunks.size(); ++chunkIndex)
        {
            allocateChunk();

            const std::size_t byteCount = sizeof(VoxelStorageSlot) * static_cast<std::size_t>(NodesPerChunk);
            std::memcpy(m_chunks.back(), other.m_chunks[chunkIndex], byteCount);
        }

        m_freeFirstChildIndexes = other.m_freeFirstChildIndexes;
        m_groupAllocated = other.m_groupAllocated;
    }
    catch (...)
    {
        clear();
        throw;
    }
}

VoxelBlockPool& VoxelBlockPool::operator=(const VoxelBlockPool& other)
{
    if (this == &other)
    {
        return *this;
    }

    VoxelBlockPool copy(other);
    swap(copy);
    return *this;
}

VoxelBlockPool::~VoxelBlockPool()
{
    clear();
}

VoxelNodeIndex VoxelBlockPool::allocateChildren()
{
    VoxelNodeIndex firstChildIndex = InvalidVoxelNodeIndex;

    if (!m_freeFirstChildIndexes.empty())
    {
        firstChildIndex = m_freeFirstChildIndexes.back();
        m_freeFirstChildIndexes.pop_back();
    }
    else
    {
        const std::size_t capacity = capacityNodeCount();

        if (static_cast<std::size_t>(m_nextNodeIndex) == capacity)
        {
            allocateChunk();
        }

        assert(m_nextNodeIndex <= InvalidVoxelNodeIndex - ChildrenPerGroup);

        firstChildIndex = m_nextNodeIndex;
        m_nextNodeIndex += ChildrenPerGroup;
    }

    assert(firstChildIndex != InvalidVoxelNodeIndex);
    assert((firstChildIndex % ChildrenPerGroup) == 0);

    const std::size_t groupIndex = static_cast<std::size_t>(firstChildIndex / ChildrenPerGroup);

    assert(groupIndex < m_groupAllocated.size());
    assert(m_groupAllocated[groupIndex] == 0);

    m_groupAllocated[groupIndex] = 1;
    ++m_allocatedGroupCount;

    resetChildren(firstChildIndex);

    assert(isChildrenAddressAligned(firstChildIndex));
    return firstChildIndex;
}

void VoxelBlockPool::releaseChildren(const VoxelNodeBlock& parentBlock)
{
    assert(parentBlock.childMask != 0);
    assert(parentBlock.firstChildIndex != InvalidVoxelNodeIndex);
    assert(containsChildren(parentBlock.firstChildIndex));

    releaseChildrenRecursive(parentBlock.firstChildIndex, parentBlock.childMask, parentBlock.leafMask);
}

void VoxelBlockPool::releaseUnusedChildren(VoxelNodeIndex firstChildIndex)
{
    assert(containsChildren(firstChildIndex));
    releaseAllocatedGroup(firstChildIndex);
}

void VoxelBlockPool::clear()
{
    for (std::size_t chunkIndex = 0; chunkIndex < m_chunks.size(); ++chunkIndex)
    {
        releaseAlignedMemory(m_chunks[chunkIndex]);
    }

    m_chunks.clear();
    m_freeFirstChildIndexes.clear();
    m_groupAllocated.clear();
    m_nextNodeIndex = 0;
    m_allocatedGroupCount = 0;
}

VoxelNodeBlock& VoxelBlockPool::initializeNode(VoxelNodeIndex index, VoxelState childState)
{
    assert(containsNode(index));
    assert(childState == VoxelState::Empty || childState == VoxelState::Material);

    VoxelStorageSlot& storageSlot = slotUnchecked(index);
    VoxelNodeBlock* block = new (&storageSlot.nodeBlock) VoxelNodeBlock;

    block->reset(childState);
    return *block;
}

VoxelNodeBlock& VoxelBlockPool::node(VoxelNodeIndex index)
{
    assert(containsNode(index));
    return nodeUnchecked(index);
}

const VoxelNodeBlock& VoxelBlockPool::node(VoxelNodeIndex index) const
{
    assert(containsNode(index));
    return nodeUnchecked(index);
}

VoxelLeafBlock& VoxelBlockPool::initializeLeaf(VoxelNodeIndex index, VoxelState state)
{
    assert(containsNode(index));
    assert(state == VoxelState::Empty || state == VoxelState::Material);

    VoxelStorageSlot& storageSlot = slotUnchecked(index);
    VoxelLeafBlock* block = new (&storageSlot.leafBlock) VoxelLeafBlock;

    block->reset(state);
    return *block;
}

VoxelLeafBlock& VoxelBlockPool::leaf(VoxelNodeIndex index)
{
    assert(containsNode(index));
    return leafUnchecked(index);
}

const VoxelLeafBlock& VoxelBlockPool::leaf(VoxelNodeIndex index) const
{
    assert(containsNode(index));
    return leafUnchecked(index);
}

bool VoxelBlockPool::containsChildren(VoxelNodeIndex firstChildIndex) const
{
    if (firstChildIndex == InvalidVoxelNodeIndex || (firstChildIndex % ChildrenPerGroup) != 0)
    {
        return false;
    }

    if (firstChildIndex >= m_nextNodeIndex)
    {
        return false;
    }

    const std::size_t groupIndex = static_cast<std::size_t>(firstChildIndex / ChildrenPerGroup);
    return groupIndex < m_groupAllocated.size() && m_groupAllocated[groupIndex] != 0;
}

bool VoxelBlockPool::containsNode(VoxelNodeIndex index) const
{
    if (index == InvalidVoxelNodeIndex || index >= m_nextNodeIndex)
    {
        return false;
    }

    const VoxelNodeIndex firstChildIndex = index - index % ChildrenPerGroup;
    return containsChildren(firstChildIndex);
}

bool VoxelBlockPool::isChildrenAddressAligned(VoxelNodeIndex firstChildIndex) const
{
    if (!containsChildren(firstChildIndex))
    {
        return false;
    }

    const std::uintptr_t address = reinterpret_cast<std::uintptr_t>(&slotUnchecked(firstChildIndex));
    return address % CacheLineAlignment == 0;
}

std::size_t VoxelBlockPool::allocatedGroupCount() const
{
    return m_allocatedGroupCount;
}

std::size_t VoxelBlockPool::storageGroupCount() const
{
    return static_cast<std::size_t>(m_nextNodeIndex / ChildrenPerGroup);
}

std::size_t VoxelBlockPool::chunkCount() const
{
    return m_chunks.size();
}

std::size_t VoxelBlockPool::capacityNodeCount() const
{
    return m_chunks.size() * static_cast<std::size_t>(NodesPerChunk);
}

void VoxelBlockPool::swap(VoxelBlockPool& other)
{
    m_chunks.swap(other.m_chunks);
    m_freeFirstChildIndexes.swap(other.m_freeFirstChildIndexes);
    m_groupAllocated.swap(other.m_groupAllocated);
    std::swap(m_nextNodeIndex, other.m_nextNodeIndex);
    std::swap(m_allocatedGroupCount, other.m_allocatedGroupCount);
}

void VoxelBlockPool::allocateChunk()
{
    const std::size_t currentCapacity = capacityNodeCount();
    const std::size_t maximumNodeCount = static_cast<std::size_t>(InvalidVoxelNodeIndex);

    if (currentCapacity > maximumNodeCount - static_cast<std::size_t>(NodesPerChunk))
    {
        throw std::bad_alloc();
    }

    const std::size_t byteCount = sizeof(VoxelStorageSlot) * static_cast<std::size_t>(NodesPerChunk);
    VoxelStorageSlot* chunk = static_cast<VoxelStorageSlot*>(allocateAlignedMemory(byteCount, CacheLineAlignment));

    if (!chunk)
    {
        throw std::bad_alloc();
    }

    assert(reinterpret_cast<std::uintptr_t>(chunk) % CacheLineAlignment == 0);

    m_chunks.push_back(chunk);
    m_groupAllocated.resize(m_groupAllocated.size() + static_cast<std::size_t>(GroupsPerChunk), 0);

    for (VoxelNodeIndex nodeIndex = 0; nodeIndex < NodesPerChunk; ++nodeIndex)
    {
        VoxelStorageSlot& storageSlot = chunk[nodeIndex];
        VoxelNodeBlock* block = new (&storageSlot.nodeBlock) VoxelNodeBlock;
        block->reset();
    }
}

void VoxelBlockPool::resetChildren(VoxelNodeIndex firstChildIndex)
{
    assert(firstChildIndex != InvalidVoxelNodeIndex);
    assert((firstChildIndex % ChildrenPerGroup) == 0);

    const VoxelNodeIndex chunkOffset = firstChildIndex % NodesPerChunk;

    assert(chunkOffset + ChildrenPerGroup <= NodesPerChunk);

    for (VoxelNodeIndex childOffset = 0; childOffset < ChildrenPerGroup; ++childOffset)
    {
        initializeNode(firstChildIndex + childOffset);
    }
}

void VoxelBlockPool::releaseChildrenRecursive(VoxelNodeIndex firstChildIndex, std::uint8_t childMask, std::uint8_t leafMask)
{
    assert(containsChildren(firstChildIndex));
    assert(childMask != 0);

    for (VoxelNodeIndex childOffset = 0; childOffset < ChildrenPerGroup; ++childOffset)
    {
        const std::uint8_t bit = static_cast<std::uint8_t>(1U << childOffset);

        if ((childMask & bit) == 0)
        {
            continue;
        }

        // childMask和leafMask同时为1时，该物理槽是掩码叶块，没有下级存储。
        if ((leafMask & bit) != 0)
        {
            continue;
        }

        const VoxelNodeBlock& childBlock = nodeUnchecked(firstChildIndex + childOffset);

        assert(childBlock.isValid());

        if (childBlock.childMask != 0)
        {
            releaseChildrenRecursive(childBlock.firstChildIndex, childBlock.childMask, childBlock.leafMask);
        }
    }

    releaseAllocatedGroup(firstChildIndex);
}

void VoxelBlockPool::releaseAllocatedGroup(VoxelNodeIndex firstChildIndex)
{
    assert(containsChildren(firstChildIndex));

    resetChildren(firstChildIndex);

    const std::size_t groupIndex = static_cast<std::size_t>(firstChildIndex / ChildrenPerGroup);

    assert(groupIndex < m_groupAllocated.size());
    assert(m_groupAllocated[groupIndex] != 0);
    assert(m_allocatedGroupCount > 0);

    m_groupAllocated[groupIndex] = 0;
    --m_allocatedGroupCount;
    m_freeFirstChildIndexes.push_back(firstChildIndex);
}

VoxelStorageSlot& VoxelBlockPool::slotUnchecked(VoxelNodeIndex index)
{
    const VoxelNodeIndex chunkIndex = index / NodesPerChunk;
    const VoxelNodeIndex chunkOffset = index % NodesPerChunk;

    assert(static_cast<std::size_t>(chunkIndex) < m_chunks.size());
    return m_chunks[static_cast<std::size_t>(chunkIndex)][chunkOffset];
}

const VoxelStorageSlot& VoxelBlockPool::slotUnchecked(VoxelNodeIndex index) const
{
    const VoxelNodeIndex chunkIndex = index / NodesPerChunk;
    const VoxelNodeIndex chunkOffset = index % NodesPerChunk;

    assert(static_cast<std::size_t>(chunkIndex) < m_chunks.size());
    return m_chunks[static_cast<std::size_t>(chunkIndex)][chunkOffset];
}

VoxelNodeBlock& VoxelBlockPool::nodeUnchecked(VoxelNodeIndex index)
{
    return slotUnchecked(index).nodeBlock;
}

const VoxelNodeBlock& VoxelBlockPool::nodeUnchecked(VoxelNodeIndex index) const
{
    return slotUnchecked(index).nodeBlock;
}

VoxelLeafBlock& VoxelBlockPool::leafUnchecked(VoxelNodeIndex index)
{
    return slotUnchecked(index).leafBlock;
}

const VoxelLeafBlock& VoxelBlockPool::leafUnchecked(VoxelNodeIndex index) const
{
    return slotUnchecked(index).leafBlock;
}

}