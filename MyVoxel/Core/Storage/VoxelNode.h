#ifndef MYVOXEL_CORE_STORAGE_VOXELNODE_H
#define MYVOXEL_CORE_STORAGE_VOXELNODE_H

#include "MyVoxel/Core/VoxelTypes.h"

namespace MyVoxel
{

// 表示一个不包含地址、层级和空间尺寸的标准多层体素节点。
class VoxelNode
{
public:
    // 构造空叶节点。
    VoxelNode();
    // 使用指定叶节点状态创建节点，state不能为Subdivided。
    explicit VoxelNode(VoxelState state);

    /// 状态判断
    // 返回当前节点状态。
    VoxelState state() const;
    // 检查当前节点状态与子节点组索引是否一致。
    bool isValid() const;
    // 判断当前节点是否为空叶节点。
    bool isEmpty() const;
    // 判断当前节点是否为材料叶节点。
    bool isMaterial() const;
    // 判断当前节点是否已经细分。
    bool isSubdivided() const;
    // 判断当前节点是否为未细分叶节点。
    bool isLeaf() const;
    // 判断当前节点是否连接有效子节点组。
    bool hasChildGroup() const;

    /// 子节点组
    // 返回当前节点连接的子节点组索引。
    VoxelNodeGroupIndex childGroupIndex() const;

    /// 状态修改
    // 将当前节点设置为空状态或未细分材料状态。
    void setState(VoxelState state);
    // 将当前节点设置为已细分状态并连接指定子节点组。
    void setChildGroupIndex(VoxelNodeGroupIndex index);

private:
    VoxelNodeGroupIndex m_childGroupIndex; // 当前节点连接的子节点组索引。
    VoxelState m_state; // 当前节点状态。
};

}

#endif // MYVOXEL_CORE_STORAGE_VOXELNODE_H