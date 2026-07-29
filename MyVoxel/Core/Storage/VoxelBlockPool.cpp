#include "VoxelBlockPool.h"

#include <cassert>
#include <cstdlib>
#include <new>
#include <utility>

#ifdef _MSC_VER
#include <malloc.h>
#endif

namespace
{

// 分配指定字节数的对齐内存，MSVC使用_aligned_malloc。
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

const VoxelIndex VoxelBlockPool::ChildrenPerGroup;
const std::size_t VoxelBlockPool::CacheLineAlignment;
const VoxelIndex VoxelBlockPool::GroupsPerChunk;
const VoxelIndex VoxelBlockPool::SlotsPerChunk;

VoxelBlockPool::VoxelBlockPool()
    : m_nextSlotIndex(0)
    , m_allocatedGroupCount(0)
{
}
VoxelBlockPool::~VoxelBlockPool()
{
    clear();
}

/// 八槽组分配与回收

VoxelIndex VoxelBlockPool::allocateChildren()
{
    VoxelIndex firstChildIndex = InvalidVoxelIndex;

    if (!m_freeFirstChildIndexes.empty())
    {
        firstChildIndex = m_freeFirstChildIndexes.back();
        m_freeFirstChildIndexes.pop_back();
    }
    else
    {
        const std::size_t capacity = capacitySlotCount();

        assert(static_cast<std::size_t>(m_nextSlotIndex) <= capacity);

        if (static_cast<std::size_t>(m_nextSlotIndex) == capacity)
        {
            allocateChunk();
        }

        if (m_nextSlotIndex > InvalidVoxelIndex - ChildrenPerGroup)
        {
            throw std::bad_alloc();
        }

        firstChildIndex = m_nextSlotIndex;
        m_nextSlotIndex += ChildrenPerGroup;
    }

    assert(firstChildIndex >= 0);
    assert(firstChildIndex != InvalidVoxelIndex);
    assert((firstChildIndex % ChildrenPerGroup) == 0);

    const std::size_t groupIndex = static_cast<std::size_t>(firstChildIndex / ChildrenPerGroup);

    assert(groupIndex < m_groupAllocated.size());
    assert(m_groupAllocated[groupIndex] == 0);

    m_groupAllocated[groupIndex] = 1;
    ++m_allocatedGroupCount;

    assert(isChildrenAddressAligned(firstChildIndex));
    return firstChildIndex;
}

void VoxelBlockPool::releaseChildren(VoxelIndex firstChildIndex)
{
    assert(containsChildren(firstChildIndex));
    assert(m_allocatedGroupCount > 0);
    assert(m_freeFirstChildIndexes.size() < m_freeFirstChildIndexes.capacity());

    const std::size_t groupIndex = static_cast<std::size_t>(firstChildIndex / ChildrenPerGroup);

    m_freeFirstChildIndexes.push_back(firstChildIndex);
    m_groupAllocated[groupIndex] = 0;
    --m_allocatedGroupCount;
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
    m_nextSlotIndex = 0;
    m_allocatedGroupCount = 0;
}

/// 物理槽初始化

VoxelNodeBlock& VoxelBlockPool::initializeNode(VoxelIndex index, VoxelState childState)
{
    assert(containsSlot(index));
    assert(childState == VoxelState::Empty || childState == VoxelState::Material);

    VoxelBlock& storageSlot = slotUnchecked(index);
    VoxelNodeBlock* block = new (&storageSlot.nodeBlock) VoxelNodeBlock;

    reset(*block, childState);
    return *block;
}

VoxelLeafBlock& VoxelBlockPool::initializeLeaf(VoxelIndex index, VoxelState state)
{
    assert(containsSlot(index));
    assert(state == VoxelState::Empty || state == VoxelState::Material);

    VoxelBlock& storageSlot = slotUnchecked(index);
    VoxelLeafBlock* block = new (&storageSlot.leafBlock) VoxelLeafBlock;

    reset(*block, state);
    return *block;
}

/// 存储检查

bool VoxelBlockPool::containsChildren(VoxelIndex firstChildIndex) const
{
    if (firstChildIndex < 0 || firstChildIndex == InvalidVoxelIndex)
    {
        return false;
    }

    if ((firstChildIndex % ChildrenPerGroup) != 0 || firstChildIndex >= m_nextSlotIndex)
    {
        return false;
    }

    const std::size_t groupIndex = static_cast<std::size_t>(firstChildIndex / ChildrenPerGroup);
    return groupIndex < m_groupAllocated.size() && m_groupAllocated[groupIndex] != 0;
}

bool VoxelBlockPool::containsSlot(VoxelIndex index) const
{
    if (index < 0 || index == InvalidVoxelIndex || index >= m_nextSlotIndex)
    {
        return false;
    }

    const VoxelIndex firstChildIndex = index - index % ChildrenPerGroup;
    return containsChildren(firstChildIndex);
}

bool VoxelBlockPool::isChildrenAddressAligned(VoxelIndex firstChildIndex) const
{
    if (!containsChildren(firstChildIndex))
    {
        return false;
    }

    const std::uintptr_t address = reinterpret_cast<std::uintptr_t>(&slotUnchecked(firstChildIndex));
    return address % static_cast<std::uintptr_t>(CacheLineAlignment) == 0;
}

/// 存储统计

std::size_t VoxelBlockPool::allocatedGroupCount() const
{
    return m_allocatedGroupCount;
}

std::size_t VoxelBlockPool::storageGroupCount() const
{
    return static_cast<std::size_t>(m_nextSlotIndex / ChildrenPerGroup);
}

std::size_t VoxelBlockPool::chunkCount() const
{
    return m_chunks.size();
}

std::size_t VoxelBlockPool::capacitySlotCount() const
{
    return m_chunks.size() * static_cast<std::size_t>(SlotsPerChunk);
}

/// 内部辅助


void VoxelBlockPool::allocateChunk()
{
    const std::size_t currentCapacity = capacitySlotCount();
    const std::size_t maximumSlotCount = static_cast<std::size_t>(InvalidVoxelIndex);

    if (currentCapacity > maximumSlotCount - static_cast<std::size_t>(SlotsPerChunk))
    {
        throw std::bad_alloc();
    }

    const std::size_t nextChunkCount = m_chunks.size() + 1;
    const std::size_t nextGroupCount = m_groupAllocated.size() + static_cast<std::size_t>(GroupsPerChunk);

    m_chunks.reserve(nextChunkCount);
    m_freeFirstChildIndexes.reserve(nextGroupCount);
    m_groupAllocated.reserve(nextGroupCount);

    const std::size_t byteCount = sizeof(VoxelBlock) * static_cast<std::size_t>(SlotsPerChunk);
    VoxelBlock* chunk = static_cast<VoxelBlock*>(allocateAlignedMemory(byteCount, CacheLineAlignment));

    if (!chunk)
    {
        throw std::bad_alloc();
    }

    try
    {
        m_groupAllocated.resize(nextGroupCount, 0);
    }
    catch (...)
    {
        releaseAlignedMemory(chunk);
        throw;
    }

    assert(reinterpret_cast<std::uintptr_t>(chunk) % static_cast<std::uintptr_t>(CacheLineAlignment) == 0);

    for (VoxelIndex slotIndex = 0; slotIndex < SlotsPerChunk; ++slotIndex)
    {
        new (&chunk[slotIndex]) VoxelBlock;
    }

    m_chunks.push_back(chunk);
}
}