#ifndef MYVOXEL_CORE_TREE_VOXELTREEEDITOR_H
#define MYVOXEL_CORE_TREE_VOXELTREEEDITOR_H

#include "MyVoxel/Core/VoxelAddress.h"
#include "VoxelForest.h"
#include "VoxelTree.h"

namespace MyVoxel
{

// 在已经完成根级写时复制的体素树节点上直接修改状态和下级结构。
class VoxelTreeEditor
{
public:
    // 使用森林中已经实际存在的节点创建编辑器，共享体素树会先执行写时复制。
    VoxelTreeEditor(VoxelForest& forest, const VoxelCellAddress& address);
    // 使用指定可写体素树的根节点创建编辑器。
    explicit VoxelTreeEditor(VoxelTree& tree);

    /// 节点状态
    // 返回当前编辑节点的状态。
    VoxelState state() const;
    // 判断当前节点是否为空叶节点。
    bool isEmpty() const;
    // 判断当前节点是否为材料叶节点。
    bool isMaterial() const;
    // 判断当前节点是否已经细分。
    bool isSubdivided() const;
    // 判断当前节点是否为叶节点。
    bool isLeaf() const;
    // 将当前节点设置为指定叶节点状态，并释放原有全部下级节点组。
    bool setState(VoxelState state);

    /// 节点结构
    // 将当前叶节点细分为八个相同状态的子节点，已经细分时返回false。
    bool split();
    // 返回指定角点对应的实际子节点编辑器，当前节点必须已经细分。
    VoxelTreeEditor child(VoxelCorner corner) const;
    // 确保当前节点已经细分，并返回指定角点对应的子节点编辑器。
    VoxelTreeEditor ensureChild(VoxelCorner corner);
    // 判断当前节点的八个子节点是否可以合并。
    bool canMerge() const;
    // 在八个子节点状态一致时合并当前节点，并返回处理后的节点状态。
    VoxelState merge();

private:
    // 使用已经定位的体素树和节点创建子节点编辑器。
    VoxelTreeEditor(VoxelTree& tree, VoxelNode& node);

private:
    VoxelTree* m_tree; // 当前编辑节点所属的独立可写体素树。
    VoxelNode* m_node; // 当前直接编辑的实际节点。
};

}

#endif // MYVOXEL_CORE_TREE_VOXELTREEEDITOR_H