#ifndef MYVOXEL_VOXELNODEFORESTEDITOR_H
#define MYVOXEL_VOXELNODEFORESTEDITOR_H

#include "VoxelNodeForest.h"

namespace MyVoxel
{

// 在已经执行根级写时复制的节点上直接进行结构和状态修改。
class VoxelNodeForestEditor
{

public:
    // 使用森林中已经存在的独立节点创建编辑器，并在根树共享时复制该根树。
    VoxelNodeForestEditor(VoxelNodeForest& forest, const VoxelCellAddress& address);

    // 使用已经完成写时复制的独立根树创建根节点编辑器。
    explicit VoxelNodeForestEditor(VoxelRootTree& tree);
    /// 节点状态

    // 返回当前编辑节点的状态。
    VoxelState state() const;

    // 将当前节点设置为空或未分割材料状态，并释放原有下级节点组。
    void setState(VoxelState state);

    /// 节点结构

    // 将当前材料节点分割为八个材料子节点。
    void split();

    // 返回指定角点对应的子节点编辑器。
    VoxelNodeForestEditor child(VoxelCorner corner) const;

    // 在八个子节点状态一致时合并当前节点，并返回处理后的节点状态。
    VoxelState merge();

private:
    // 使用已经定位的根树和节点创建子节点编辑器。
    VoxelNodeForestEditor(VoxelRootTree& tree, VoxelNode& node);

    VoxelRootTree* m_tree; // 当前编辑节点所属的独立根树。
    VoxelNode* m_node; // 当前直接编辑的节点。
};

}

#endif // MYVOXEL_VOXELNODEFORESTEDITOR_H