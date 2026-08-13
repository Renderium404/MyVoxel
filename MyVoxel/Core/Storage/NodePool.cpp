#include "NodePool.h"

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

const VoxelIndex NodePool::SlotsPerGroup;
const std::size_t NodePool::GroupAlignment;
const VoxelIndex NodePool::GroupsPerChunk;
const VoxelIndex NodePool::SlotsPerChunk;

NodePool::NodePool()
    : m_nextSlotIndex(0)
    , m_highWaterSlotCount(0)
    , m_allocatedGroupCount(0)
{
}

NodePool::~NodePool()
{
    clear();
}

/// 节点组分配与回收

VoxelIndex NodePool::allocateGroup(std::uint64_t rawValue)
{
    VoxelIndex firstSlotIndex = InvalidVoxelIndex;

    if (!m_freeGroupFirstSlotIndexes.empty())
    {
        firstSlotIndex = m_freeGroupFirstSlotIndexes.back();
        m_freeGroupFirstSlotIndexes.pop_back();

        const std::size_t groupIndex = static_cast<std::size_t>(firstSlotIndex / SlotsPerGroup);
        assert(groupIndex < m_groupAllocated.size());
        assert(m_groupAllocated[groupIndex] == 0);
        m_groupAllocated[groupIndex] = 1;
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

        const std::size_t groupIndex = static_cast<std::size_t>(firstSlotIndex / SlotsPerGroup);
        assert(groupIndex < m_groupAllocated.size());
        assert(m_groupAllocated[groupIndex] == 0);
        m_groupAllocated[groupIndex] = 1;
    }

    ++m_allocatedGroupCount;

    assert(firstSlotIndex >= 0);
    assert((firstSlotIndex % SlotsPerGroup) == 0);
    assert(isGroupAligned(firstSlotIndex));

    initGroup(firstSlotIndex, rawValue);
    return firstSlotIndex;
}

void NodePool::initGroup(VoxelIndex firstSlotIndex, std::uint64_t rawValue)
{
    assert(containsGroup(firstSlotIndex));

    VoxelBlock* blocks = group(firstSlotIndex);
    for (VoxelIndex slotOffset = 0; slotOffset < SlotsPerGroup; ++slotOffset)
    {
        blocks[slotOffset].rawValue = rawValue;
    }
}

void NodePool::releaseGroup(VoxelIndex firstSlotIndex)
{
    assert(containsGroup(firstSlotIndex));
    assert(m_allocatedGroupCount > 0);

    const std::size_t groupIndex = static_cast<std::size_t>(firstSlotIndex / SlotsPerGroup);
    m_freeGroupFirstSlotIndexes.push_back(firstSlotIndex);
    m_groupAllocated[groupIndex] = 0;
    --m_allocatedGroupCount;
}

void NodePool::rewind()
{
    m_freeGroupFirstSlotIndexes.clear();
    m_nextSlotIndex = 0;
    m_allocatedGroupCount = 0;
}

void NodePool::clear()
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

void NodePool::swap(NodePool& other)
{
    if (this == &other)
    {
        return;
    }

    m_chunks.swap(other.m_chunks);
    m_freeGroupFirstSlotIndexes.swap(other.m_freeGroupFirstSlotIndexes);
    m_groupAllocated.swap(other.m_groupAllocated);
    std::swap(m_nextSlotIndex, other.m_nextSlotIndex);
    std::swap(m_highWaterSlotCount, other.m_highWaterSlotCount);
    std::swap(m_allocatedGroupCount, other.m_allocatedGroupCount);
}

/// 物理槽初始化

NodeBlock& NodePool::initializeNode(VoxelIndex index, VoxelNodeState childState)
{
    assert(containsSlot(index));
    assert(childState == VoxelNodeState::Empty || childState == VoxelNodeState::Material);

    VoxelBlock& storage = blockUnchecked(index);
    NodeBlock* block = new (&storage.node) NodeBlock;
    reset(*block, childState);
    return *block;
}

IndexBlock& NodePool::initializeIndex(VoxelIndex index, VoxelIndex leafIndex)
{
    assert(containsSlot(index));
    assert(leafIndex >= 0);
    assert(leafIndex != InvalidVoxelIndex);

    VoxelBlock& storage = blockUnchecked(index);
    IndexBlock* block = new (&storage.index) IndexBlock;
    reset(*block);
    block->leafIndex = leafIndex;
    return *block;
}



/// 存储检查

bool NodePool::containsGroup(VoxelIndex firstSlotIndex) const
{
    if (firstSlotIndex < 0 || firstSlotIndex == InvalidVoxelIndex || firstSlotIndex >= m_nextSlotIndex)
    {
        return false;
    }

    if ((firstSlotIndex % SlotsPerGroup) != 0)
    {
        return false;
    }

    const std::size_t groupIndex = static_cast<std::size_t>(firstSlotIndex / SlotsPerGroup);
    return groupIndex < m_groupAllocated.size() && m_groupAllocated[groupIndex] != 0;
}

bool NodePool::containsSlot(VoxelIndex index) const
{
    if (index < 0 || index == InvalidVoxelIndex || index >= m_nextSlotIndex)
    {
        return false;
    }

    return containsGroup(index - index % SlotsPerGroup);
}

bool NodePool::isGroupAligned(VoxelIndex firstSlotIndex) const
{
    if (!containsGroup(firstSlotIndex))
    {
        return false;
    }

    const std::uintptr_t address = reinterpret_cast<std::uintptr_t>(&blockUnchecked(firstSlotIndex));
    return address % static_cast<std::uintptr_t>(GroupAlignment) == 0;
}

/// 存储统计

std::size_t NodePool::allocatedGroupCount() const
{
    return m_allocatedGroupCount;
}

std::size_t NodePool::highWaterGroupCount() const
{
    return static_cast<std::size_t>(m_highWaterSlotCount / SlotsPerGroup);
}

std::size_t NodePool::chunkCount() const
{
    return m_chunks.size();
}

std::size_t NodePool::capacitySlotCount() const
{
    return m_chunks.size() * static_cast<std::size_t>(SlotsPerChunk);
}

std::size_t NodePool::storageCapacityBytes() const
{
    return capacitySlotCount() * sizeof(VoxelBlock);
}

/// 内部辅助

void NodePool::allocateChunk()
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

    const std::size_t byteCount = sizeof(VoxelBlock) * static_cast<std::size_t>(SlotsPerChunk); // 8192个8字节VoxelBlock固定组成64 KiB节点Chunk。
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

    assert(byteCount == static_cast<std::size_t>(64 * 1024));
    assert(reinterpret_cast<std::uintptr_t>(chunk) % static_cast<std::uintptr_t>(GroupAlignment) == 0);

    for (VoxelIndex slotIndex = 0; slotIndex < SlotsPerChunk; ++slotIndex)
    {
        new (&chunk[slotIndex]) VoxelBlock;
    }

    m_chunks.push_back(chunk);
}

}