#ifndef MYVOXEL_VOXELBLOCKPOOL_H
#define MYVOXEL_VOXELBLOCKPOOL_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "VoxelLeafBlock.h"
#include "VoxelNodeBlock.h"

namespace MyVoxel
{

// 保存一个8字节节点块或掩码叶块，具体类型由父节点掩码决定。
union VoxelStorageSlot
{
    std::uint64_t rawValue;
    VoxelNodeBlock nodeBlock;
    VoxelLeafBlock leafBlock;
};

static_assert(sizeof(VoxelStorageSlot) == 8, "VoxelStorageSlot must be exactly 8 bytes.");

// 使用64字节对齐分段内存管理八槽连续分配、回收和深复制。
class VoxelBlockPool
{
public:
    static const VoxelNodeIndex ChildrenPerGroup = 8; // 一个已细分单元固定对应八个连续存储槽。
    static const VoxelNodeIndex CacheLineAlignment = 64; // 当前目标平台使用64字节缓存行对齐。
    static const VoxelNodeIndex GroupsPerChunk = 1024; // 每个Chunk保存1024组，即固定64 KiB槽数据。
    static const VoxelNodeIndex NodesPerChunk = ChildrenPerGroup * GroupsPerChunk; // 每个Chunk保存8192个物理槽。

    VoxelBlockPool();
    VoxelBlockPool(const VoxelBlockPool& other);
    VoxelBlockPool& operator=(const VoxelBlockPool& other);
    ~VoxelBlockPool();

    /// 分配与回收

    // 分配八个连续且64字节对齐的物理存储槽，返回第一个槽索引。
    VoxelNodeIndex allocateChildren();

    // 根据父节点掩码递归释放八槽组及全部普通分支后代。
    void releaseChildren(const VoxelNodeBlock& parentBlock);

    // 释放已确认不再包含活动物理子单元的八槽组。
    void releaseUnusedChildren(VoxelNodeIndex firstChildIndex);

    // 清空全部存储槽、空闲索引和分段内存。
    void clear();

    /// 普通节点访问

    // 将指定物理槽初始化为普通节点块。
    VoxelNodeBlock& initializeNode(VoxelNodeIndex index, VoxelState childState = VoxelState::Empty);

    // 返回指定物理槽中的可写普通节点块。
    VoxelNodeBlock& node(VoxelNodeIndex index);

    // 返回指定物理槽中的只读普通节点块。
    const VoxelNodeBlock& node(VoxelNodeIndex index) const;

    /// 掩码叶块访问

    // 将指定物理槽初始化为掩码叶块。
    VoxelLeafBlock& initializeLeaf(VoxelNodeIndex index, VoxelState state = VoxelState::Empty);

    // 返回指定物理槽中的可写掩码叶块。
    VoxelLeafBlock& leaf(VoxelNodeIndex index);

    // 返回指定物理槽中的只读掩码叶块。
    const VoxelLeafBlock& leaf(VoxelNodeIndex index) const;

    /// 存储检查

    // 检查指定首索引是否对应一个当前已分配的八槽组。
    bool containsChildren(VoxelNodeIndex firstChildIndex) const;

    // 检查指定槽索引是否属于一个当前已分配的八槽组。
    bool containsNode(VoxelNodeIndex index) const;

    // 检查指定八槽组首地址是否满足64字节对齐。
    bool isChildrenAddressAligned(VoxelNodeIndex firstChildIndex) const;

    /// 存储统计

    // 返回当前实际分配的八槽组数量。
    std::size_t allocatedGroupCount() const;

    // 返回池中曾经创建的八槽组数量，包括当前空闲组。
    std::size_t storageGroupCount() const;

    // 返回当前分配的Chunk数量。
    std::size_t chunkCount() const;

    // 返回当前Chunk能够容纳的物理槽总数。
    std::size_t capacityNodeCount() const;

private:
    void swap(VoxelBlockPool& other);
    void allocateChunk();
    void resetChildren(VoxelNodeIndex firstChildIndex);
    void releaseChildrenRecursive(VoxelNodeIndex firstChildIndex, std::uint8_t childMask, std::uint8_t leafMask);
    void releaseAllocatedGroup(VoxelNodeIndex firstChildIndex);

    VoxelStorageSlot& slotUnchecked(VoxelNodeIndex index);
    const VoxelStorageSlot& slotUnchecked(VoxelNodeIndex index) const;

    VoxelNodeBlock& nodeUnchecked(VoxelNodeIndex index);
    const VoxelNodeBlock& nodeUnchecked(VoxelNodeIndex index) const;

    VoxelLeafBlock& leafUnchecked(VoxelNodeIndex index);
    const VoxelLeafBlock& leafUnchecked(VoxelNodeIndex index) const;

    std::vector<VoxelStorageSlot*> m_chunks; // 各个64字节对齐且地址稳定的存储Chunk。
    std::vector<VoxelNodeIndex> m_freeFirstChildIndexes; // 当前空闲八槽组的首索引栈。
    std::vector<std::uint8_t> m_groupAllocated; // 每个八槽组是否处于已分配状态。
    VoxelNodeIndex m_nextNodeIndex; // 尚未使用的下一个顺序物理槽索引。
    std::size_t m_allocatedGroupCount; // 当前实际分配的八槽组数量。
};

}

#endif // MYVOXEL_VOXELBLOCKPOOL_H