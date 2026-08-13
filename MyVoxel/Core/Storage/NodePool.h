#ifndef MYVOXEL_CORE_STORAGE_NODEPOOL_H
#define MYVOXEL_CORE_STORAGE_NODEPOOL_H

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "NodeBlock.h"

namespace MyVoxel
{

// 使用64字节对齐的分段内存管理VoxelBlock八槽节点组。
//
// 一个VoxelBlock固定8字节，一个NodeBlock固定描述八个直接子节点，
// 因此一个节点组固定包含八个连续VoxelBlock并占用一个64字节缓存行。
class NodePool
{
public:
    static const VoxelIndex SlotsPerGroup = 8; // 一个NodeBlock固定对应八个连续子节点物理槽。
    static const std::size_t GroupAlignment = 64; // 一个八槽节点组固定占用一个64字节缓存行。
    static const VoxelIndex GroupsPerChunk = 1024; // 每个64 KiB Chunk固定保存1024个节点组。
    static const VoxelIndex SlotsPerChunk = SlotsPerGroup * GroupsPerChunk; // 每个Chunk固定保存8192个VoxelBlock。

public:
    // 构造空节点池。
    NodePool();
    // 禁止复制节点池。
    NodePool(const NodePool& other) = delete;
    // 禁止复制赋值节点池。
    NodePool& operator=(const NodePool& other) = delete;
    // 释放全部节点Chunk。
    ~NodePool();

    /// 节点组分配与回收

    // 分配八个连续且64字节对齐的VoxelBlock槽，并使用指定原始值初始化全部八个物理槽。
    VoxelIndex allocateGroup(std::uint64_t rawValue = 0);
    // 使用指定原始值初始化已经分配的完整八槽节点组。
    void initGroup(VoxelIndex firstSlotIndex, std::uint64_t rawValue);
    // 回收指定八槽节点组，不递归释放槽内记录的节点或叶数据。
    void releaseGroup(VoxelIndex firstSlotIndex);
    // 回卷全部分配状态并保留已经申请的Chunk，调用后之前返回的全部槽索引均失效。
    void rewind();
    // 清空全部Chunk和分配状态。
    void clear();
    // 交换两个节点池的完整存储和分配状态。
    void swap(NodePool& other);

    /// 物理槽初始化

    // 将指定物理槽初始化为NodeBlock。
    NodeBlock& initializeNode(VoxelIndex index, VoxelNodeState childState = VoxelNodeState::Empty);
    // 将指定物理槽初始化为IndexBlock并记录统一叶索引。
    IndexBlock& initializeIndex(VoxelIndex index, VoxelIndex leafIndex);

    /// 节点访问与修改

    // 返回指定物理槽中的可写NodeBlock，调用者负责保证当前槽实际按照NodeBlock解释。
    inline NodeBlock& node(VoxelIndex index);
    // 返回指定物理槽中的只读NodeBlock，调用者负责保证当前槽实际按照NodeBlock解释。
    inline const NodeBlock& node(VoxelIndex index) const;
    // 返回指定物理槽中的可写IndexBlock，调用者负责保证当前槽实际按照IndexBlock解释。
    inline IndexBlock& index(VoxelIndex index);
    // 返回指定物理槽中的只读IndexBlock，调用者负责保证当前槽实际按照IndexBlock解释。
    inline const IndexBlock& index(VoxelIndex index) const;
    // 返回指定物理槽中的完整可写VoxelBlock。
    inline VoxelBlock& block(VoxelIndex index);
    // 返回指定物理槽中的完整只读VoxelBlock。
    inline const VoxelBlock& block(VoxelIndex index) const;

    /// 节点组访问与修改
    inline VoxelBlock* group(VoxelIndex firstSlotIndex);
    inline const VoxelBlock* group(VoxelIndex firstSlotIndex) const;

    /// 存储检查

    // 检查指定首索引是否对应当前有效的八槽节点组。
    bool containsGroup(VoxelIndex firstSlotIndex) const;
    // 检查指定物理槽是否属于当前有效的八槽节点组。
    bool containsSlot(VoxelIndex index) const;
    // 检查指定节点组首地址是否满足64字节对齐。
    bool isGroupAligned(VoxelIndex firstSlotIndex) const;

    /// 存储统计

    // 返回当前有效节点组数量。
    std::size_t allocatedGroupCount() const;
    // 返回历史实际使用过的节点组数量高水位。
    std::size_t highWaterGroupCount() const;
    // 返回当前Chunk数量。
    std::size_t chunkCount() const;
    // 返回当前全部Chunk能够容纳的VoxelBlock槽数量。
    std::size_t capacitySlotCount() const;
    // 返回当前全部Chunk占用的节点存储容量，单位为字节。
    std::size_t storageCapacityBytes() const;

private:
    // 分配一个新的64字节对齐64 KiB节点Chunk。
    void allocateChunk();
    // 返回指定物理槽，不检查当前分配状态。
    inline VoxelBlock& blockUnchecked(VoxelIndex index);
    inline const VoxelBlock& blockUnchecked(VoxelIndex index) const;

private:
    std::vector<VoxelBlock*> m_chunks; // 各个64字节对齐且地址稳定的64 KiB节点Chunk。
    std::vector<VoxelIndex> m_freeGroupFirstSlotIndexes; // 被单独回收的节点组首索引栈。
    std::vector<std::uint8_t> m_groupAllocated; // 当前分配前沿内各节点组是否有效。
    VoxelIndex m_nextSlotIndex; // 当前分配轮次尚未使用的下一个顺序槽索引。
    VoxelIndex m_highWaterSlotCount; // 历史实际使用过的槽数量高水位。
    std::size_t m_allocatedGroupCount; // 当前有效节点组数量。
};

inline NodeBlock& NodePool::node(VoxelIndex index)
{
    assert(containsSlot(index));
    return blockUnchecked(index).node;
}

inline const NodeBlock& NodePool::node(VoxelIndex index) const
{
    assert(containsSlot(index));
    return blockUnchecked(index).node;
}

inline IndexBlock& NodePool::index(VoxelIndex index)
{
    assert(containsSlot(index));
    return blockUnchecked(index).index;
}

inline const IndexBlock& NodePool::index(VoxelIndex index) const
{
    assert(containsSlot(index));
    return blockUnchecked(index).index;
}

inline VoxelBlock& NodePool::block(VoxelIndex index)
{
    assert(containsSlot(index));
    return blockUnchecked(index);
}

inline const VoxelBlock& NodePool::block(VoxelIndex index) const
{
    assert(containsSlot(index));
    return blockUnchecked(index);
}

inline VoxelBlock* NodePool::group(VoxelIndex firstSlotIndex)
{
    assert(containsGroup(firstSlotIndex));
    return &blockUnchecked(firstSlotIndex);
}
inline const VoxelBlock* NodePool::group(VoxelIndex firstSlotIndex) const
{
    assert(containsGroup(firstSlotIndex));
    return &blockUnchecked(firstSlotIndex);
}
inline VoxelBlock& NodePool::blockUnchecked(VoxelIndex index)
{
    assert(index >= 0);
    assert(index < m_nextSlotIndex);
    const VoxelIndex chunkIndex = index / SlotsPerChunk;
    const VoxelIndex chunkOffset = index % SlotsPerChunk;
    return m_chunks[static_cast<std::size_t>(chunkIndex)][chunkOffset];
}

inline const VoxelBlock& NodePool::blockUnchecked(VoxelIndex index) const
{
    assert(index >= 0);
    assert(index < m_nextSlotIndex);
    const VoxelIndex chunkIndex = index / SlotsPerChunk;
    const VoxelIndex chunkOffset = index % SlotsPerChunk;
    return m_chunks[static_cast<std::size_t>(chunkIndex)][chunkOffset];
}

}

#endif // MYVOXEL_CORE_STORAGE_NODEPOOL_H