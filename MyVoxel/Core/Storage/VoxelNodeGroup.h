#ifndef MYVOXEL_CORE_STORAGE_VOXELNODEGROUP_H
#define MYVOXEL_CORE_STORAGE_VOXELNODEGROUP_H

#include <array>

#include "VoxelNode.h"

namespace MyVoxel
{

// 表示一个节点细分后生成的八个子节点。
class VoxelNodeGroup
{
public:
    // 构造包含八个空叶节点的节点组。
    VoxelNodeGroup();
    // 使用指定叶节点状态初始化八个子节点。
    explicit VoxelNodeGroup(VoxelState state);

    /// 子节点访问
    // 返回指定角点方向对应的子节点。
    VoxelNode& child(VoxelCorner corner);
    // 返回指定角点方向对应的只读子节点。
    const VoxelNode& child(VoxelCorner corner) const;

    /// 节点组状态
    // 检查节点组中的全部子节点是否有效。
    bool isValid() const;
    // 检查八个子节点是否可以合并为一个叶节点。
    bool canMerge() const;
    // 返回八个子节点合并后的叶节点状态。
    VoxelState mergedState() const;

    /// 状态修改
    // 使用指定叶节点状态重置八个子节点。
    void reset(VoxelState state);

private:
    std::array<VoxelNode, VoxelCornerCount> m_children; // 按体素角点顺序保存的八个子节点。
};

}

#endif // MYVOXEL_CORE_STORAGE_VOXELNODEGROUP_H