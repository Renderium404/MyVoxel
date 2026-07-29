#ifndef MYVOXEL_CORE_STORAGE_VOXELNODEPOOL_H
#define MYVOXEL_CORE_STORAGE_VOXELNODEPOOL_H

#include <cstddef>
#include <deque>
#include <vector>

#include "VoxelNodeGroup.h"

namespace MyVoxel
{

// 统一管理体素节点组的分配、访问和递归回收。
class VoxelNodePool
{
public:
    VoxelNodePool();

    /// 节点组管理
    // 分配一个节点组，并将八个子节点初始化为指定叶节点状态。
    VoxelNodeGroupIndex allocateNodeGroup(VoxelState state);
    // 递归释放指定节点组及其连接的全部下级节点组。
    void releaseNodeGroup(VoxelNodeGroupIndex index);
    // 清空节点池中的所有节点组。
    void clear();

    /// 节点组访问
    // 返回指定索引对应的节点组。
    VoxelNodeGroup& nodeGroup(VoxelNodeGroupIndex index);
    // 返回指定索引对应的只读节点组。
    const VoxelNodeGroup& nodeGroup(VoxelNodeGroupIndex index) const;

    /// 状态与统计
    // 检查指定索引是否对应当前已分配的节点组。
    bool contains(VoxelNodeGroupIndex index) const;
    // 判断节点池当前是否没有已分配节点组。
    bool isEmpty() const;
    // 返回当前已分配的节点组数量。
    std::size_t allocatedGroupCount() const;
    // 返回节点池当前占用的节点组存储槽数量。
    std::size_t storageGroupCount() const;
    // 返回当前可复用的空闲节点组槽数量。
    std::size_t freeGroupCount() const;

private:
    // 表示节点池中的一个节点组存储槽。
    struct NodeGroupSlot
    {
        NodeGroupSlot();

        VoxelNodeGroup group; // 当前存储槽中的节点组。
        bool allocated; // 当前存储槽是否已经分配。
    };

    // 递归释放节点组内部使用的统一入口。
    void releaseNodeGroupRecursive(VoxelNodeGroupIndex index);

private:
    std::deque<NodeGroupSlot> m_slots; // 节点组存储槽，追加元素时保持已有节点组地址稳定。
    std::vector<VoxelNodeGroupIndex> m_freeIndexes; // 已释放并且可以重新使用的节点组索引。
    std::size_t m_allocatedGroupCount; // 当前已分配的节点组数量。
};

}

#endif // MYVOXEL_CORE_STORAGE_VOXELNODEPOOL_H