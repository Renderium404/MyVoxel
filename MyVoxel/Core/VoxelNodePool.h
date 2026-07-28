#ifndef MYVOXEL_VOXELNODEPOOL_H
#define MYVOXEL_VOXELNODEPOOL_H

#include <cstddef>
#include <deque>
#include <vector>

#include "VoxelNodeGroup.h"

namespace MyVoxel
{

// 节点池， 统一管理体素节点组的分配、访问和回收。
class VoxelNodePool
{
public:
    VoxelNodePool() = default;

    // 分配一个节点组，并将八个节点初始化为指定状态。
    VoxelNodeGroupIndex allocateNodeGroup(VoxelState state);

    // 递归释放指定节点组及其连接的所有下级节点组。
    void releaseNodeGroup(VoxelNodeGroupIndex index);

    // 返回指定索引对应的节点组。
    VoxelNodeGroup& nodeGroup(VoxelNodeGroupIndex index);

    // 返回指定索引对应的只读节点组。
    const VoxelNodeGroup& nodeGroup(VoxelNodeGroupIndex index) const;

    // 检查指定索引是否对应当前已分配的节点组。
    bool contains(VoxelNodeGroupIndex index) const;

    // 返回当前已分配的节点组数量。
    std::size_t allocatedGroupCount() const;

    // 返回节点池当前占用的节点组存储槽数量。
    std::size_t storageGroupCount() const;

    // 清空节点池中的所有节点组。
    void clear();

private:
    // 表示节点池中的一个节点组存储槽。
    struct NodeGroupSlot
    {
        VoxelNodeGroup group; // 当前存储槽中的节点组。
        bool allocated = false; // 当前存储槽是否已经分配。
    };

    // 递归释放节点组内部使用的统一入口。
    void releaseNodeGroupRecursive(VoxelNodeGroupIndex index);

    std::deque<NodeGroupSlot> m_slots; // 节点组存储槽，追加元素时保持已有对象地址稳定。
    std::vector<VoxelNodeGroupIndex> m_freeIndexes; // 已释放并可重新使用的节点组索引。
    std::size_t m_allocatedGroupCount = 0; // 当前已分配的节点组数量。
};

}

#endif // MYVOXEL_VOXELNODEPOOL_H