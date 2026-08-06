#ifndef MYVOXEL_CORE_STORAGE_VOLUMEBLOCKPOOL_H
#define MYVOXEL_CORE_STORAGE_VOLUMEBLOCKPOOL_H

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "VolumeBlock.h"
#include "MyVoxel/Foundation/ReferenceCounted.h"

namespace MyVoxel
{

// 使用64字节对齐的64 KiB分段内存管理VolumeBlock的分配、回收和存储复用。
class VolumeBlockPool : public Foundation::ReferenceCounted
{
public:
    static const std::size_t BlockAlignment = 64; // 每个256字节VolumeBlock起始地址按缓存行对齐。
    static const VoxelIndex BlocksPerChunk = 256; // 每个64 KiB Chunk保存256个VolumeBlock。

public:
    // 构造空距离块池。
    VolumeBlockPool();
    // 禁止复制距离块池。
    VolumeBlockPool(const VolumeBlockPool& other) = delete;
    // 禁止复制赋值距离块池。
    VolumeBlockPool& operator=(const VolumeBlockPool& other) = delete;
    // 释放全部Chunk。
    ~VolumeBlockPool() override;

    /// 距离块分配与回收
    // 分配一个64字节对齐的距离块，不初始化块内距离值。
    VoxelIndex allocateBlock();
    // 回收一个距离块，不修改块内原有距离值。
    void releaseBlock(VoxelIndex index);
    // 回卷全部分配状态并保留已分配Chunk，调用后之前返回的全部索引均视为失效。
    void rewind();
    // 清空全部Chunk、空闲块和分配状态。
    void clear();
    // 交换两个距离块池持有的全部存储和分配状态。
    void swap(VolumeBlockPool& other);
    /// 距离块初始化与访问
    // 将指定已分配距离块的全部样本初始化为同一距离值。
    VolumeBlock& initializeBlock(VoxelIndex index, float distance = 0.0f);
    // 返回指定索引对应的可写距离块。
    inline VolumeBlock& block(VoxelIndex index);
    // 返回指定索引对应的只读距离块。
    inline const VolumeBlock& block(VoxelIndex index) const;

    /// 存储检查
    // 检查指定索引是否对应当前分配轮次中已分配的距离块。
    bool containsBlock(VoxelIndex index) const;
    // 检查指定距离块首地址是否满足64字节对齐。
    bool isBlockAligned(VoxelIndex index) const;

    /// 存储统计
    // 返回当前分配轮次中已分配的距离块数量。
    std::size_t allocatedBlockCount() const;
    // 返回距离块池历史实际使用过的距离块数量高水位。
    std::size_t highWaterBlockCount() const;
    // 返回当前Chunk数量。
    std::size_t chunkCount() const;
    // 返回当前全部Chunk能够容纳的距离块数量。
    std::size_t capacityBlockCount() const;
    // 返回当前全部Chunk占用的存储容量，单位为字节。
    std::size_t storageCapacityBytes() const;

private:
    // 分配并初始化一个新的64 KiB Chunk。
    void allocateChunk();
    // 返回指定距离块，不执行分配状态检查。
    inline VolumeBlock& blockUnchecked(VoxelIndex index);
    inline const VolumeBlock& blockUnchecked(VoxelIndex index) const;

private:
    std::vector<VolumeBlock*> m_chunks; // 各个64字节对齐且地址稳定的64 KiB存储Chunk。
    std::vector<VoxelIndex> m_freeBlockIndexes; // 当前分配轮次中被单独回收的距离块索引栈。
    std::vector<std::uint8_t> m_blockAllocated; // 当前分配轮次前沿内各距离块是否仍处于已分配状态。
    VoxelIndex m_nextBlockIndex; // 当前分配轮次尚未使用的下一个顺序距离块索引。
    VoxelIndex m_highWaterBlockCount; // 距离块池历史实际使用过的距离块数量高水位。
    std::size_t m_allocatedBlockCount; // 当前分配轮次中已分配的距离块数量。
};

inline VolumeBlock& VolumeBlockPool::block(VoxelIndex index)
{
    assert(containsBlock(index));
    return blockUnchecked(index);
}

inline const VolumeBlock& VolumeBlockPool::block(VoxelIndex index) const
{
    assert(containsBlock(index));
    return blockUnchecked(index);
}

inline VolumeBlock& VolumeBlockPool::blockUnchecked(VoxelIndex index)
{
    assert(index >= 0);
    assert(index < m_nextBlockIndex);

    const VoxelIndex chunkIndex = index / BlocksPerChunk;
    const VoxelIndex chunkOffset = index % BlocksPerChunk;
    return m_chunks[static_cast<std::size_t>(chunkIndex)][chunkOffset];
}

inline const VolumeBlock& VolumeBlockPool::blockUnchecked(VoxelIndex index) const
{
    assert(index >= 0);
    assert(index < m_nextBlockIndex);

    const VoxelIndex chunkIndex = index / BlocksPerChunk;
    const VoxelIndex chunkOffset = index % BlocksPerChunk;
    return m_chunks[static_cast<std::size_t>(chunkIndex)][chunkOffset];
}

}

#endif // MYVOXEL_CORE_STORAGE_VOLUMEBLOCKPOOL_H