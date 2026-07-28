#ifndef MYVOXEL_VOXELNODEGROUP_H
#define MYVOXEL_VOXELNODEGROUP_H

#include <array>

#include "VoxelNode.h"

namespace MyVoxel
{

// 节点组，表示一个节点分割后生成的八个子节点。
class VoxelNodeGroup
{
public:
    VoxelNodeGroup() = default;

    // 使用指定非分割状态初始化八个子节点。
    explicit VoxelNodeGroup(VoxelState state);

    // 返回指定角点对应的子节点。
    VoxelNode& child(VoxelCorner corner);

    // 返回指定角点对应的只读子节点。
    const VoxelNode& child(VoxelCorner corner) const;

    // 使用指定非分割状态重置八个子节点。
    void reset(VoxelState state);

    // 检查八个子节点是否可以合并为一个节点。
    bool canMerge() const;

    // 返回八个子节点合并后的节点状态。
    VoxelState mergedState() const;

private:
    std::array<VoxelNode, VoxelCornerCount> m_children; // 按体素角点顺序存储的八个子节点。
};

}

#endif // MYVOXEL_VOXELNODEGROUP_H