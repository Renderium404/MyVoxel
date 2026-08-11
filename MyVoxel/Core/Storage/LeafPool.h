#ifndef MYVOXEL_CORE_STORAGE_LEAFPOOL_H
#define MYVOXEL_CORE_STORAGE_LEAFPOOL_H

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "LeafBlock.h"
#include "MaskBlock.h"

namespace MyVoxel
{

// 使用统一leafIndex管理一一对应的MaskBlock和LeafBlock。
//
// 每个Chunk通过一次64字节对齐分配取得连续66 KiB内存：
// 前2 KiB保存256个MaskBlock，后64 KiB保存256个LeafBlock。
// 同一个leafIndex在两个区域中始终表示同一个逻辑MaskLeaf。
class LeafPool
{
public:
    static const std::size_t ChunkAlignment = 64; // Chunk和全部LeafBlock首地址按64字节缓存行对齐。
    static const VoxelIndex BlocksPerChunk = 256; // 每个Chunk固定保存256个逻辑MaskLeaf。

public:
    // 构造空叶数据池。
    LeafPool();
    // 禁止复制叶数据池。
    LeafPool(const LeafPool& other) = delete;
    // 禁止复制赋值叶数据池。
    LeafPool& operator=(const LeafPool& other) = delete;
    // 释放全部叶数据Chunk。
    ~LeafPool();

    /// 叶分配与回收

    // 分配一个统一leafIndex，不初始化对应MaskBlock和LeafBlock数据。
    VoxelIndex allocateBlock();
    // 回收指定统一leafIndex，不修改原有MaskBlock和LeafBlock内容。
    void releaseBlock(VoxelIndex index);
    // 回卷全部分配状态并保留Chunk，调用后之前返回的全部leafIndex均失效。
    void rewind();
    // 清空全部Chunk和分配状态。
    void clear();
    // 交换两个叶数据池的完整存储和分配状态。
    void swap(LeafPool& other);

    /// 叶初始化与复制

    // 将指定叶的全部距离初始化为同一值，并同步初始化材料掩码。
    void initializeBlock(VoxelIndex index, float distance = 0.0f);
    // 从另一个叶池完整复制MaskBlock和LeafBlock到当前已分配目标叶。
    void copyBlock(VoxelIndex destinationIndex, const LeafPool& sourcePool, VoxelIndex sourceIndex);

    /// 叶数据访问

    // 返回指定leafIndex对应的可写MaskBlock。
    inline MaskBlock& mask(VoxelIndex index);
    // 返回指定leafIndex对应的只读MaskBlock。
    inline const MaskBlock& mask(VoxelIndex index) const;
    // 返回指定leafIndex对应的可写LeafBlock。
    inline LeafBlock& leaf(VoxelIndex index);
    // 返回指定leafIndex对应的只读LeafBlock。
    inline const LeafBlock& leaf(VoxelIndex index) const;

    /// 存储检查

    // 检查指定leafIndex当前是否有效。
    bool containsBlock(VoxelIndex index) const;
    // 检查指定Chunk首地址是否满足64字节对齐。
    bool isChunkAligned(std::size_t chunkIndex) const;
    // 检查指定LeafBlock首地址是否满足64字节对齐。
    bool isLeafAligned(VoxelIndex index) const;

    /// 存储统计

    // 返回当前有效逻辑叶数量。
    std::size_t allocatedBlockCount() const;
    // 返回历史实际使用过的逻辑叶数量高水位。
    std::size_t highWaterBlockCount() const;
    // 返回当前Chunk数量。
    std::size_t chunkCount() const;
    // 返回全部Chunk能够容纳的逻辑叶数量。
    std::size_t capacityBlockCount() const;
    // 返回Mask区域总存储容量。
    std::size_t maskStorageCapacityBytes() const;
    // 返回Leaf区域总存储容量。
    std::size_t leafStorageCapacityBytes() const;
    // 返回全部Chunk实际存储容量。
    std::size_t storageCapacityBytes() const;

private:
    // 保存一个完整连续66 KiB叶数据Chunk的首地址。
    struct Chunk
    {
        Chunk()
            : memory(nullptr)
        {
        }

        unsigned char* memory; // 一次64字节对齐分配取得的完整连续Chunk。
    };

private:
    // 返回一个Chunk中Mask区域固定字节数。
    static std::size_t maskChunkByteCount();
    // 返回一个Chunk中Leaf区域固定字节数。
    static std::size_t leafChunkByteCount();
    // 返回一个完整Chunk固定字节数。
    static std::size_t chunkByteCount();

    // 分配并构造一个新的66 KiB连续Chunk。
    void allocateChunk();

    // 返回指定Chunk的MaskBlock数组首地址。
    inline MaskBlock* maskBlocks(Chunk& chunk);
    inline const MaskBlock* maskBlocks(const Chunk& chunk) const;
    // 返回指定Chunk的LeafBlock数组首地址。
    inline LeafBlock* leafBlocks(Chunk& chunk);
    inline const LeafBlock* leafBlocks(const Chunk& chunk) const;
    // 返回指定leafIndex对应的MaskBlock，不检查当前分配状态。
    inline MaskBlock& maskUnchecked(VoxelIndex index);
    inline const MaskBlock& maskUnchecked(VoxelIndex index) const;
    // 返回指定leafIndex对应的LeafBlock，不检查当前分配状态。
    inline LeafBlock& leafUnchecked(VoxelIndex index);
    inline const LeafBlock& leafUnchecked(VoxelIndex index) const;

private:
    std::vector<Chunk> m_chunks; // 各个独立但内部连续的66 KiB叶数据Chunk。
    std::vector<VoxelIndex> m_freeBlockIndexes; // 被单独回收的统一leafIndex栈。
    std::vector<std::uint8_t> m_blockAllocated; // 当前分配前沿内各leafIndex是否有效。
    VoxelIndex m_nextBlockIndex; // 当前分配轮次尚未使用的下一个顺序leafIndex。
    VoxelIndex m_highWaterBlockCount; // 历史实际使用过的逻辑叶数量高水位。
    std::size_t m_allocatedBlockCount; // 当前有效逻辑叶数量。
};

inline MaskBlock& LeafPool::mask(VoxelIndex index)
{
    assert(containsBlock(index));
    return maskUnchecked(index);
}

inline const MaskBlock& LeafPool::mask(VoxelIndex index) const
{
    assert(containsBlock(index));
    return maskUnchecked(index);
}

inline LeafBlock& LeafPool::leaf(VoxelIndex index)
{
    assert(containsBlock(index));
    return leafUnchecked(index);
}

inline const LeafBlock& LeafPool::leaf(VoxelIndex index) const
{
    assert(containsBlock(index));
    return leafUnchecked(index);
}

inline MaskBlock* LeafPool::maskBlocks(Chunk& chunk)
{
    assert(chunk.memory);
    return reinterpret_cast<MaskBlock*>(chunk.memory);
}

inline const MaskBlock* LeafPool::maskBlocks(const Chunk& chunk) const
{
    assert(chunk.memory);
    return reinterpret_cast<const MaskBlock*>(chunk.memory);
}

inline LeafBlock* LeafPool::leafBlocks(Chunk& chunk)
{
    assert(chunk.memory);
    return reinterpret_cast<LeafBlock*>(chunk.memory + maskChunkByteCount());
}

inline const LeafBlock* LeafPool::leafBlocks(const Chunk& chunk) const
{
    assert(chunk.memory);
    return reinterpret_cast<const LeafBlock*>(chunk.memory + maskChunkByteCount());
}

inline MaskBlock& LeafPool::maskUnchecked(VoxelIndex index)
{
    assert(index >= 0);
    assert(index < m_nextBlockIndex);
    const VoxelIndex chunkIndex = index / BlocksPerChunk;
    const VoxelIndex chunkOffset = index % BlocksPerChunk;
    return maskBlocks(m_chunks[static_cast<std::size_t>(chunkIndex)])[chunkOffset];
}

inline const MaskBlock& LeafPool::maskUnchecked(VoxelIndex index) const
{
    assert(index >= 0);
    assert(index < m_nextBlockIndex);
    const VoxelIndex chunkIndex = index / BlocksPerChunk;
    const VoxelIndex chunkOffset = index % BlocksPerChunk;
    return maskBlocks(m_chunks[static_cast<std::size_t>(chunkIndex)])[chunkOffset];
}

inline LeafBlock& LeafPool::leafUnchecked(VoxelIndex index)
{
    assert(index >= 0);
    assert(index < m_nextBlockIndex);
    const VoxelIndex chunkIndex = index / BlocksPerChunk;
    const VoxelIndex chunkOffset = index % BlocksPerChunk;
    return leafBlocks(m_chunks[static_cast<std::size_t>(chunkIndex)])[chunkOffset];
}

inline const LeafBlock& LeafPool::leafUnchecked(VoxelIndex index) const
{
    assert(index >= 0);
    assert(index < m_nextBlockIndex);
    const VoxelIndex chunkIndex = index / BlocksPerChunk;
    const VoxelIndex chunkOffset = index % BlocksPerChunk;
    return leafBlocks(m_chunks[static_cast<std::size_t>(chunkIndex)])[chunkOffset];
}

}

#endif // MYVOXEL_CORE_STORAGE_LEAFPOOL_H