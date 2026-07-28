#ifndef MYVOXEL_VOXELNODEFORESTCONSTCURSOR_H
#define MYVOXEL_VOXELNODEFORESTCONSTCURSOR_H

#include "VoxelNodeForest.h"

namespace MyVoxel
{

// 在只读独立根树中直接访问当前节点及其子节点，并支持未实际创建的材料子节点。
class VoxelNodeForestConstCursor
{
public:
    // 使用森林中已经存在的独立节点创建只读游标。
    VoxelNodeForestConstCursor(const VoxelNodeForest& forest, const VoxelCellAddress& address);

    /// 节点状态

    // 返回当前节点状态。
    VoxelState state() const;

    /// 子节点访问

    // 返回指定角点对应的子节点游标，材料叶节点的子游标表示虚拟材料节点。
    VoxelNodeForestConstCursor child(VoxelCorner corner) const;

private:
    // 使用已经定位的根树、节点或虚拟材料节点创建游标。
    VoxelNodeForestConstCursor(const VoxelRootTree& tree, const VoxelNode* node, bool virtualMaterial);

    const VoxelRootTree* m_tree; // 当前游标所属的只读独立根树。
    const VoxelNode* m_node; // 当前实际节点，虚拟材料节点时为空。
    bool m_virtualMaterial; // 当前是否表示未实际创建的材料节点。
};

}

#endif // MYVOXEL_VOXELNODEFORESTCONSTCURSOR_H