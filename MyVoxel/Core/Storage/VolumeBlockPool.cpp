#include "VolumeBlockPool.h"

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

const std::size_t VolumeBlockPool::BlockAlignment;
const VoxelIndex VolumeBlockPool::BlocksPerChunk;

VolumeBlockPool::VolumeBlockPool()
    : m_nextBlockIndex(0)
    , m_highWaterBlockCount(0)
    , m_allocatedBlockCount(0)
{
}

VolumeBlockPool::~VolumeBlockPool()
{
    clear();
}

/// 距离块分配与回收

VoxelIndex VolumeBlockPool::allocateBlock()
{
    VoxelIndex blockIndex = InvalidVoxelIndex;
    bool reusedReleasedBlock = false;

    if (!m_freeBlockIndexes.empty())
    {
        blockIndex = m_freeBlockIndexes.back();
        m_freeBlockIndexes.pop_back();
        reusedReleasedBlock = true;
    }
    else
    {
        const std::size_t capacity = capacityBlockCount();

        assert(static_cast<std::size_t>(m_nextBlockIndex) <= capacity);

        if (static_cast<std::size_t>(m_nextBlockIndex) == capacity)
        {
            allocateChunk();
        }

        if (m_nextBlockIndex == InvalidVoxelIndex)
        {
            throw std::bad_alloc();
        }

        blockIndex = m_nextBlockIndex;
        ++m_nextBlockIndex;

        if (m_nextBlockIndex > m_highWaterBlockCount)
        {
            m_highWaterBlockCount = m_nextBlockIndex;
        }
    }

    assert(blockIndex >= 0);
    assert(blockIndex != InvalidVoxelIndex);
    assert(static_cast<std::size_t>(blockIndex) < m_blockAllocated.size());

    if (reusedReleasedBlock)
    {
        assert(m_blockAllocated[static_cast<std::size_t>(blockIndex)] == 0);
    }

    m_blockAllocated[static_cast<std::size_t>(blockIndex)] = 1;
    ++m_allocatedBlockCount;

    assert(isBlockAligned(blockIndex));
    return blockIndex;
}

void VolumeBlockPool::releaseBlock(VoxelIndex index)
{
    assert(containsBlock(index));
    assert(m_allocatedBlockCount > 0);
    assert(m_freeBlockIndexes.size() < m_freeBlockIndexes.capacity());

    m_freeBlockIndexes.push_back(index);
    m_blockAllocated[static_cast<std::size_t>(index)] = 0;
    --m_allocatedBlockCount;
}

void VolumeBlockPool::rewind()
{
    m_freeBlockIndexes.clear();
    m_nextBlockIndex = 0;
    m_allocatedBlockCount = 0;
}

void VolumeBlockPool::clear()
{
    for (std::size_t chunkIndex = 0; chunkIndex < m_chunks.size(); ++chunkIndex)
    {
        releaseAlignedMemory(m_chunks[chunkIndex]);
    }

    m_chunks.clear();
    m_freeBlockIndexes.clear();
    m_blockAllocated.clear();
    m_nextBlockIndex = 0;
    m_highWaterBlockCount = 0;
    m_allocatedBlockCount = 0;
}

/// 距离块初始化与访问

VolumeBlock& VolumeBlockPool::initializeBlock(VoxelIndex index, float distance)
{
    assert(containsBlock(index));

    VolumeBlock& result = blockUnchecked(index);
    MyVoxel::reset(result, distance);
    return result;
}

/// 存储检查

bool VolumeBlockPool::containsBlock(VoxelIndex index) const
{
    if (index < 0 || index == InvalidVoxelIndex || index >= m_nextBlockIndex)
    {
        return false;
    }

    return static_cast<std::size_t>(index) < m_blockAllocated.size() && m_blockAllocated[static_cast<std::size_t>(index)] != 0;
}

bool VolumeBlockPool::isBlockAligned(VoxelIndex index) const
{
    if (!containsBlock(index))
    {
        return false;
    }

    const std::uintptr_t address = reinterpret_cast<std::uintptr_t>(&blockUnchecked(index));
    return address % static_cast<std::uintptr_t>(BlockAlignment) == 0;
}

/// 存储统计

std::size_t VolumeBlockPool::allocatedBlockCount() const
{
    return m_allocatedBlockCount;
}

std::size_t VolumeBlockPool::highWaterBlockCount() const
{
    return static_cast<std::size_t>(m_highWaterBlockCount);
}

std::size_t VolumeBlockPool::chunkCount() const
{
    return m_chunks.size();
}

std::size_t VolumeBlockPool::capacityBlockCount() const
{
    return m_chunks.size() * static_cast<std::size_t>(BlocksPerChunk);
}

std::size_t VolumeBlockPool::storageCapacityBytes() const
{
    return capacityBlockCount() * sizeof(VolumeBlock);
}

/// 内部辅助

void VolumeBlockPool::allocateChunk()
{
    const std::size_t currentCapacity = capacityBlockCount();
    const std::size_t maximumBlockCount = static_cast<std::size_t>(InvalidVoxelIndex);

    if (currentCapacity > maximumBlockCount - static_cast<std::size_t>(BlocksPerChunk))
    {
        throw std::bad_alloc();
    }

    const std::size_t nextChunkCount = m_chunks.size() + 1;
    const std::size_t nextCapacity = currentCapacity + static_cast<std::size_t>(BlocksPerChunk);

    m_chunks.reserve(nextChunkCount);
    m_freeBlockIndexes.reserve(nextCapacity);
    m_blockAllocated.reserve(nextCapacity);

    const std::size_t byteCount = sizeof(VolumeBlock) * static_cast<std::size_t>(BlocksPerChunk);
    VolumeBlock* chunk = static_cast<VolumeBlock*>(allocateAlignedMemory(byteCount, BlockAlignment));

    if (!chunk)
    {
        throw std::bad_alloc();
    }

    try
    {
        m_blockAllocated.resize(nextCapacity, 0);
    }
    catch (...)
    {
        releaseAlignedMemory(chunk);
        throw;
    }

    assert(reinterpret_cast<std::uintptr_t>(chunk) % static_cast<std::uintptr_t>(BlockAlignment) == 0);
    assert(byteCount == static_cast<std::size_t>(64 * 1024));

    for (VoxelIndex blockIndex = 0; blockIndex < BlocksPerChunk; ++blockIndex)
    {
        new (&chunk[blockIndex]) VolumeBlock;
    }

    m_chunks.push_back(chunk);
}

}