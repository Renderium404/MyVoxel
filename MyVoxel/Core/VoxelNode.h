#ifndef MYVOXEL_VOXELNODE_H
#define MYVOXEL_VOXELNODE_H

#include <cstdint>
#include <limits>

#include "VoxelTypes.h"

namespace MyVoxel
{

using VoxelNodeGroupIndex = std::uint32_t; // 节点池中的节点组索引。

const VoxelNodeGroupIndex InvalidVoxelNodeGroupIndex = (std::numeric_limits<VoxelNodeGroupIndex>::max)(); // 表示节点未连接节点组。

// 标准节点，表示一个多层体素节点，不包含地址、层级和空间尺寸。
class VoxelNode
{
public:
    VoxelNode() = default;

    // 使用指定节点状态创建节点，state不能为Subdivided。
    explicit VoxelNode(VoxelState state);

    // 返回当前节点状态。
    VoxelState state() const;

    // 检查当前节点状态与节点组索引是否一致。
    // 如果没有索引但是状态为Subdivided，说明异常
    bool isValid() const;

    // 返回当前节点连接的节点组索引。
    VoxelNodeGroupIndex childGroupIndex() const;

    // 将当前节点设置为空状态或未分割材料状态。
    void setState(VoxelState state);

    // 将当前节点设置为已分割状态并连接指定节点组。
    void setChildGroupIndex(VoxelNodeGroupIndex index);

private:
    VoxelNodeGroupIndex m_childGroupIndex = InvalidVoxelNodeGroupIndex; // 子节点组索引。
    VoxelState m_state = VoxelState::Empty; // 当前节点状态。
};

}

#endif // MYVOXEL_VOXELNODE_H