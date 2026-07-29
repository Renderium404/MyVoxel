#ifndef MYVOXEL_CORE_STORAGE_VOXELBLOCKPOOL_H
#define MYVOXEL_CORE_STORAGE_VOXELBLOCKPOOL_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "VoxelBlock.h"
#include "MyVoxel/Foundation/ReferenceCounted.h"
namespace MyVoxel
{



// 使用64字节对齐的分段内存管理八槽连续分配、回收和深复制。
class VoxelBlockPool : public Foundation::ReferenceCounted
{
public:
    static const VoxelIndex ChildrenPerGroup = 8; // 一个细分节点固定对应八个连续物理槽。
    static const std::size_t CacheLineAlignment = 64; // 一个八槽组固定占用一个64字节缓存行。
    static const VoxelIndex GroupsPerChunk = 1024; // 每个Chunk保存1024个八槽组。
    static const VoxelIndex SlotsPerChunk = ChildrenPerGroup * GroupsPerChunk; // 每个Chunk保存8192个物理槽。

public:
    // 构造空节点池。
    VoxelBlockPool();
    // 深复制节点池及其全部物理存储。
    VoxelBlockPool(const VoxelBlockPool& other)=delete;
    // 深复制赋值节点池。
    VoxelBlockPool& operator=(const VoxelBlockPool& other)=delete;
    // 释放全部Chunk。
    ~VoxelBlockPool() override;

    /// 八槽组分配与回收
    // 分配八个连续且64字节对齐的物理槽，不初始化槽内节点类型。
    VoxelIndex allocateChildren();
    // 回收一个八槽组，不检查或递归释放其中记录的后代。
    void releaseChildren(VoxelIndex firstChildIndex);
    // 清空全部Chunk、空闲组和分配状态。
    void clear();

    /// 物理槽初始化
    // 将指定物理槽初始化为普通节点块。
    VoxelNodeBlock& initializeNode(VoxelIndex index, VoxelState childState = VoxelState::Empty);
    // 将指定物理槽初始化为掩码叶块。
    VoxelLeafBlock& initializeLeaf(VoxelIndex index, VoxelState state = VoxelState::Empty);

    /// 普通节点访问
    // 返回指定物理槽中的可写普通节点块，调用者必须保证该槽当前为Branch。
    inline VoxelNodeBlock& node(VoxelIndex index);
    // 返回指定物理槽中的只读普通节点块，调用者必须保证该槽当前为Branch。
    inline const VoxelNodeBlock& node(VoxelIndex index) const;

    /// 掩码叶块访问
    // 返回指定物理槽中的可写掩码叶块，调用者必须保证该槽当前为MaskLeaf。
    inline VoxelLeafBlock& leaf(VoxelIndex index);
    // 返回指定物理槽中的只读掩码叶块，调用者必须保证该槽当前为MaskLeaf。
    inline const VoxelLeafBlock& leaf(VoxelIndex index) const;

    /// 存储检查
    // 检查指定首索引是否对应当前已分配的八槽组。
    bool containsChildren(VoxelIndex firstChildIndex) const;
    // 检查指定物理槽是否属于当前已分配的八槽组。
    bool containsSlot(VoxelIndex index) const;
    // 检查指定八槽组首地址是否满足64字节对齐。
    bool isChildrenAddressAligned(VoxelIndex firstChildIndex) const;

    /// 存储统计
    // 返回当前已分配的八槽组数量。
    std::size_t allocatedGroupCount() const;
    // 返回池中曾经创建的八槽组数量，包括当前空闲组。
    std::size_t storageGroupCount() const;
    // 返回当前Chunk数量。
    std::size_t chunkCount() const;
    // 返回当前全部Chunk能够容纳的物理槽数量。
    std::size_t capacitySlotCount() const;

private:

    // 分配并初始化一个新的64 KiB Chunk。
    void allocateChunk();

    // 返回指定物理槽，不执行分配状态检查。
    inline VoxelBlock& slotUnchecked(VoxelIndex index);
    inline const VoxelBlock& slotUnchecked(VoxelIndex index) const;

    // 返回指定普通节点块，不检查槽类型。
    inline VoxelNodeBlock& nodeUnchecked(VoxelIndex index);
    inline const VoxelNodeBlock& nodeUnchecked(VoxelIndex index) const;

    // 返回指定掩码叶块，不检查槽类型。
    inline VoxelLeafBlock& leafUnchecked(VoxelIndex index);
    inline const VoxelLeafBlock& leafUnchecked(VoxelIndex index) const;

private:
    std::vector<VoxelBlock*> m_chunks; // 各个64字节对齐且地址稳定的存储Chunk。
    std::vector<VoxelIndex> m_freeFirstChildIndexes; // 当前空闲八槽组的首索引栈。
    std::vector<std::uint8_t> m_groupAllocated; // 各八槽组当前是否处于已分配状态。
    VoxelIndex m_nextSlotIndex; // 尚未使用的下一个顺序物理槽索引。
    std::size_t m_allocatedGroupCount; // 当前已分配的八槽组数量。
};

inline VoxelNodeBlock& VoxelBlockPool::node(VoxelIndex index)
{
    assert(containsSlot(index));
    return nodeUnchecked(index);
}

inline const VoxelNodeBlock& VoxelBlockPool::node(VoxelIndex index) const
{
    assert(containsSlot(index));
    return nodeUnchecked(index);
}

inline VoxelLeafBlock& VoxelBlockPool::leaf(VoxelIndex index)
{
    assert(containsSlot(index));
    return leafUnchecked(index);
}

inline const VoxelLeafBlock& VoxelBlockPool::leaf(VoxelIndex index) const
{
    assert(containsSlot(index));
    return leafUnchecked(index);
}

inline VoxelBlock& VoxelBlockPool::slotUnchecked(VoxelIndex index)
{
    assert(index >= 0);
    assert(index < m_nextSlotIndex);

    const VoxelIndex chunkIndex = index / SlotsPerChunk;
    const VoxelIndex chunkOffset = index % SlotsPerChunk;

    return m_chunks[static_cast<std::size_t>(chunkIndex)][chunkOffset];
}

inline const VoxelBlock& VoxelBlockPool::slotUnchecked(VoxelIndex index) const
{
    assert(index >= 0);
    assert(index < m_nextSlotIndex);

    const VoxelIndex chunkIndex = index / SlotsPerChunk;
    const VoxelIndex chunkOffset = index % SlotsPerChunk;

    return m_chunks[static_cast<std::size_t>(chunkIndex)][chunkOffset];
}

inline VoxelNodeBlock& VoxelBlockPool::nodeUnchecked(VoxelIndex index)
{
    return slotUnchecked(index).nodeBlock;
}

inline const VoxelNodeBlock& VoxelBlockPool::nodeUnchecked(VoxelIndex index) const
{
    return slotUnchecked(index).nodeBlock;
}

inline VoxelLeafBlock& VoxelBlockPool::leafUnchecked(VoxelIndex index)
{
    return slotUnchecked(index).leafBlock;
}

inline const VoxelLeafBlock& VoxelBlockPool::leafUnchecked(VoxelIndex index) const
{
    return slotUnchecked(index).leafBlock;
}

}

#endif // MYVOXEL_CORE_STORAGE_VOXELBLOCKPOOL_H