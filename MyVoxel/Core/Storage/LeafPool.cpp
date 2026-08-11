#include "LeafPool.h"

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

const std::size_t LeafPool::ChunkAlignment;
const VoxelIndex LeafPool::BlocksPerChunk;

static_assert(sizeof(MaskBlock) == 8, "MaskBlock must be exactly 8 bytes.");
static_assert(sizeof(LeafBlock) == 256, "LeafBlock must be exactly 256 bytes.");
static_assert((sizeof(MaskBlock) * 256) % 64 == 0, "Mask region must end on a cache-line boundary.");
static_assert(sizeof(LeafBlock) % 64 == 0, "LeafBlock size must be a multiple of cache-line size.");

LeafPool::LeafPool()
    : m_nextBlockIndex(0)
    , m_highWaterBlockCount(0)
    , m_allocatedBlockCount(0)
{
}

LeafPool::~LeafPool()
{
    clear();
}

/// 叶分配与回收

VoxelIndex LeafPool::allocateBlock()
{
    VoxelIndex blockIndex = InvalidVoxelIndex;

    if (!m_freeBlockIndexes.empty())
    {
        blockIndex = m_freeBlockIndexes.back();
        m_freeBlockIndexes.pop_back();

        assert(blockIndex >= 0);
        assert(blockIndex < m_nextBlockIndex);
        assert(m_blockAllocated[static_cast<std::size_t>(blockIndex)] == 0);

        m_blockAllocated[static_cast<std::size_t>(blockIndex)] = 1;
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

        assert(static_cast<std::size_t>(blockIndex) < m_blockAllocated.size());
        m_blockAllocated[static_cast<std::size_t>(blockIndex)] = 1;
    }

    ++m_allocatedBlockCount;

    assert(containsBlock(blockIndex));
    assert(isLeafAligned(blockIndex));
    return blockIndex;
}

void LeafPool::releaseBlock(VoxelIndex index)
{
    assert(containsBlock(index));
    assert(m_allocatedBlockCount > 0);

    m_freeBlockIndexes.push_back(index);
    m_blockAllocated[static_cast<std::size_t>(index)] = 0;
    --m_allocatedBlockCount;
}

void LeafPool::rewind()
{
    m_freeBlockIndexes.clear();
    m_nextBlockIndex = 0;
    m_allocatedBlockCount = 0;
}

void LeafPool::clear()
{
    for (std::size_t chunkIndex = 0; chunkIndex < m_chunks.size(); ++chunkIndex)
    {
        releaseAlignedMemory(m_chunks[chunkIndex].memory);
    }

    m_chunks.clear();
    m_freeBlockIndexes.clear();
    m_blockAllocated.clear();
    m_nextBlockIndex = 0;
    m_highWaterBlockCount = 0;
    m_allocatedBlockCount = 0;
}

void LeafPool::swap(LeafPool& other)
{
    if (this == &other)
    {
        return;
    }

    m_chunks.swap(other.m_chunks);
    m_freeBlockIndexes.swap(other.m_freeBlockIndexes);
    m_blockAllocated.swap(other.m_blockAllocated);
    std::swap(m_nextBlockIndex, other.m_nextBlockIndex);
    std::swap(m_highWaterBlockCount, other.m_highWaterBlockCount);
    std::swap(m_allocatedBlockCount, other.m_allocatedBlockCount);
}

/// 叶初始化与复制

void LeafPool::initializeBlock(VoxelIndex index, float distance)
{
    assert(containsBlock(index));
    MyVoxel::reset(maskUnchecked(index), leafUnchecked(index), distance);
}

void LeafPool::copyBlock(VoxelIndex destinationIndex, const LeafPool& sourcePool, VoxelIndex sourceIndex)
{
    assert(containsBlock(destinationIndex));
    assert(sourcePool.containsBlock(sourceIndex));

    maskUnchecked(destinationIndex) = sourcePool.maskUnchecked(sourceIndex);
    leafUnchecked(destinationIndex) = sourcePool.leafUnchecked(sourceIndex);
}

/// 存储检查

bool LeafPool::containsBlock(VoxelIndex index) const
{
    if (index < 0 || index == InvalidVoxelIndex || index >= m_nextBlockIndex)
    {
        return false;
    }

    return static_cast<std::size_t>(index) < m_blockAllocated.size() && m_blockAllocated[static_cast<std::size_t>(index)] != 0;
}

bool LeafPool::isChunkAligned(std::size_t chunkIndex) const
{
    if (chunkIndex >= m_chunks.size() || !m_chunks[chunkIndex].memory)
    {
        return false;
    }

    const std::uintptr_t address = reinterpret_cast<std::uintptr_t>(m_chunks[chunkIndex].memory);
    return address % static_cast<std::uintptr_t>(ChunkAlignment) == 0;
}

bool LeafPool::isLeafAligned(VoxelIndex index) const
{
    if (!containsBlock(index))
    {
        return false;
    }

    const std::uintptr_t address = reinterpret_cast<std::uintptr_t>(&leafUnchecked(index));
    return address % static_cast<std::uintptr_t>(ChunkAlignment) == 0;
}

/// 存储统计

std::size_t LeafPool::allocatedBlockCount() const
{
    return m_allocatedBlockCount;
}

std::size_t LeafPool::highWaterBlockCount() const
{
    return static_cast<std::size_t>(m_highWaterBlockCount);
}

std::size_t LeafPool::chunkCount() const
{
    return m_chunks.size();
}

std::size_t LeafPool::capacityBlockCount() const
{
    return m_chunks.size() * static_cast<std::size_t>(BlocksPerChunk);
}

std::size_t LeafPool::maskStorageCapacityBytes() const
{
    return m_chunks.size() * maskChunkByteCount();
}

std::size_t LeafPool::leafStorageCapacityBytes() const
{
    return m_chunks.size() * leafChunkByteCount();
}

std::size_t LeafPool::storageCapacityBytes() const
{
    return m_chunks.size() * chunkByteCount();
}

/// 内部辅助

std::size_t LeafPool::maskChunkByteCount()
{
    return sizeof(MaskBlock) * static_cast<std::size_t>(BlocksPerChunk);
}

std::size_t LeafPool::leafChunkByteCount()
{
    return sizeof(LeafBlock) * static_cast<std::size_t>(BlocksPerChunk);
}

std::size_t LeafPool::chunkByteCount()
{
    return maskChunkByteCount() + leafChunkByteCount();
}

void LeafPool::allocateChunk()
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

    const std::size_t maskByteCount = maskChunkByteCount();
    const std::size_t leafByteCount = leafChunkByteCount();
    const std::size_t byteCount = chunkByteCount(); // 2 KiB Mask区域与64 KiB Leaf区域连续组成一个66 KiB Chunk。

    assert(maskByteCount == static_cast<std::size_t>(2 * 1024));
    assert(leafByteCount == static_cast<std::size_t>(64 * 1024));
    assert(byteCount == static_cast<std::size_t>(66 * 1024));
    assert(maskByteCount % ChunkAlignment == 0);

    unsigned char* memory = static_cast<unsigned char*>(allocateAlignedMemory(byteCount, ChunkAlignment));

    if (!memory)
    {
        throw std::bad_alloc();
    }

    Chunk chunk;
    chunk.memory = memory;

    MaskBlock* masks = maskBlocks(chunk);
    LeafBlock* leaves = leafBlocks(chunk);

    try
    {
        m_blockAllocated.resize(nextCapacity, 0);
    }
    catch (...)
    {
        releaseAlignedMemory(memory);
        throw;
    }

    assert(reinterpret_cast<std::uintptr_t>(memory) % static_cast<std::uintptr_t>(ChunkAlignment) == 0);
    assert(reinterpret_cast<std::uintptr_t>(masks) % static_cast<std::uintptr_t>(ChunkAlignment) == 0);
    assert(reinterpret_cast<std::uintptr_t>(leaves) % static_cast<std::uintptr_t>(ChunkAlignment) == 0);

    for (VoxelIndex blockIndex = 0; blockIndex < BlocksPerChunk; ++blockIndex)
    {
        new (&masks[blockIndex]) MaskBlock;
        new (&leaves[blockIndex]) LeafBlock;
    }

    m_chunks.push_back(chunk);
}

}