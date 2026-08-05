#include "VoxelBlockPool.h"

#include <cassert>
#include <cstdlib>
#include <new>

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

const VoxelIndex VoxelBlockPool::SlotsPerGroup;
const std::size_t VoxelBlockPool::GroupAlignment;
const VoxelIndex VoxelBlockPool::GroupsPerChunk;
const VoxelIndex VoxelBlockPool::SlotsPerChunk;

VoxelBlockPool::VoxelBlockPool()
    : m_nextSlotIndex(0)
    , m_highWaterSlotCount(0)
    , m_allocatedGroupCount(0)
{
}

VoxelBlockPool::~VoxelBlockPool()
{
    clear();
}

/// 八槽组分配与回收

VoxelIndex VoxelBlockPool::allocateGroup()
{
    VoxelIndex firstSlotIndex = InvalidVoxelIndex;
    bool reusedReleasedGroup = false;

    if (!m_freeGroupFirstSlotIndexes.empty())
    {
        firstSlotIndex = m_freeGroupFirstSlotIndexes.back();
        m_freeGroupFirstSlotIndexes.pop_back();
        reusedReleasedGroup = true;
    }
    else
    {
        const std::size_t capacity = capacitySlotCount();

        assert(static_cast<std::size_t>(m_nextSlotIndex) <= capacity);

        if (static_cast<std::size_t>(m_nextSlotIndex) == capacity)
        {
            allocateChunk();
        }

        if (m_nextSlotIndex > InvalidVoxelIndex - SlotsPerGroup)
        {
            throw std::bad_alloc();
        }

        firstSlotIndex = m_nextSlotIndex;
        m_nextSlotIndex += SlotsPerGroup;

        if (m_nextSlotIndex > m_highWaterSlotCount)
        {
            m_highWaterSlotCount = m_nextSlotIndex;
        }
    }

    assert(firstSlotIndex >= 0);
    assert(firstSlotIndex != InvalidVoxelIndex);
    assert((firstSlotIndex % SlotsPerGroup) == 0);

    const std::size_t groupIndex = static_cast<std::size_t>(firstSlotIndex / SlotsPerGroup);

    assert(groupIndex < m_groupAllocated.size());

    // 空闲栈只保存当前分配轮次内明确释放的节点组，其分配标记必须已经清零。
    if (reusedReleasedGroup)
    {
        assert(m_groupAllocated[groupIndex] == 0);
    }

    // 顺序分配可能覆盖上一轮遗留标记，因此直接写入当前轮次状态。
    m_groupAllocated[groupIndex] = 1;
    ++m_allocatedGroupCount;

    assert(isGroupAligned(firstSlotIndex));
    return firstSlotIndex;
}

void VoxelBlockPool::releaseGroup(VoxelIndex firstSlotIndex)
{
    assert(containsGroup(firstSlotIndex));
    assert(m_allocatedGroupCount > 0);
    assert(m_freeGroupFirstSlotIndexes.size() < m_freeGroupFirstSlotIndexes.capacity());

    const std::size_t groupIndex = static_cast<std::size_t>(firstSlotIndex / SlotsPerGroup);

    m_freeGroupFirstSlotIndexes.push_back(firstSlotIndex);
    m_groupAllocated[groupIndex] = 0;
    --m_allocatedGroupCount;
}

void VoxelBlockPool::rewind()
{

    m_freeGroupFirstSlotIndexes.clear();
    m_nextSlotIndex = 0;
    m_allocatedGroupCount = 0;
}

void VoxelBlockPool::clear()
{
    for (std::size_t chunkIndex = 0; chunkIndex < m_chunks.size(); ++chunkIndex)
    {
        releaseAlignedMemory(m_chunks[chunkIndex]);
    }

    m_chunks.clear();
    m_freeGroupFirstSlotIndexes.clear();
    m_groupAllocated.clear();
    m_nextSlotIndex = 0;
    m_highWaterSlotCount = 0;
    m_allocatedGroupCount = 0;
}

/// 物理槽初始化

VoxelNodeBlock& VoxelBlockPool::initializeNode(VoxelIndex index, VoxelNodeState childState)
{
    assert(containsSlot(index));
    assert(childState == VoxelNodeState::Empty || childState == VoxelNodeState::Material);

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

bool VoxelBlockPool::containsGroup(VoxelIndex firstSlotIndex) const
{
    if (firstSlotIndex < 0 || firstSlotIndex == InvalidVoxelIndex)
    {
        return false;
    }

    if ((firstSlotIndex % SlotsPerGroup) != 0 || firstSlotIndex >= m_nextSlotIndex)
    {
        return false;
    }

    const std::size_t groupIndex = static_cast<std::size_t>(firstSlotIndex / SlotsPerGroup);
    return groupIndex < m_groupAllocated.size() && m_groupAllocated[groupIndex] != 0;
}

bool VoxelBlockPool::containsSlot(VoxelIndex index) const
{
    if (index < 0 || index == InvalidVoxelIndex || index >= m_nextSlotIndex)
    {
        return false;
    }

    const VoxelIndex firstSlotIndex = index - index % SlotsPerGroup;
    return containsGroup(firstSlotIndex);
}

bool VoxelBlockPool::isGroupAligned(VoxelIndex firstSlotIndex) const
{
    if (!containsGroup(firstSlotIndex))
    {
        return false;
    }

    const std::uintptr_t address = reinterpret_cast<std::uintptr_t>(&slotUnchecked(firstSlotIndex));
    return address % static_cast<std::uintptr_t>(GroupAlignment) == 0;
}

/// 存储统计

std::size_t VoxelBlockPool::allocatedGroupCount() const
{
    return m_allocatedGroupCount;
}

std::size_t VoxelBlockPool::highWaterGroupCount() const
{
    return static_cast<std::size_t>(m_highWaterSlotCount / SlotsPerGroup);
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
    m_freeGroupFirstSlotIndexes.reserve(nextGroupCount);
    m_groupAllocated.reserve(nextGroupCount);

    const std::size_t byteCount = sizeof(VoxelBlock) * static_cast<std::size_t>(SlotsPerChunk);
    VoxelBlock* chunk = static_cast<VoxelBlock*>(allocateAlignedMemory(byteCount, GroupAlignment));

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

    assert(reinterpret_cast<std::uintptr_t>(chunk) % static_cast<std::uintptr_t>(GroupAlignment) == 0);

    for (VoxelIndex slotIndex = 0; slotIndex < SlotsPerChunk; ++slotIndex)
    {
        new (&chunk[slotIndex]) VoxelBlock;
    }

    m_chunks.push_back(chunk);
}

}