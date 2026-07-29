#ifndef MYVOXEL_VOXELPACKEDTREEEDITOR_H
#define MYVOXEL_VOXELPACKEDTREEEDITOR_H

#include "VoxelPackedRootTree.h"

namespace MyVoxel
{

// 在一个独立Packed根树中修改普通树节点或掩码叶块中的逻辑节点。
class VoxelPackedTreeEditor
{
public:
    // 使用已经完成写时复制的独立根树创建根节点编辑器。
    explicit VoxelPackedTreeEditor(VoxelPackedRootTree& tree);

    /// 节点状态

    // 返回当前编辑节点的公共体素状态。
    VoxelState state() const;

    // 将当前节点设置为空或材料状态，并释放当前节点拥有的下级存储。
    void setState(VoxelState state);

    /// 节点结构

    // 将当前普通叶节点细分为八个继承原状态的普通子节点，未发生结构变化时返回false。
    bool split();

    // 将当前非根普通叶节点转换为包含下面两层的掩码叶块。
    bool makeMaskLeaf();

    // 检查当前节点是否直接对应一个掩码叶块。
    bool isMaskLeaf() const;

    // 检查当前节点是否允许继续访问逻辑子节点。
    bool canAccessChildren() const;

    // 返回指定角点对应的直接逻辑子节点编辑器。
    VoxelPackedTreeEditor child(VoxelCorner corner) const;

    // 尝试合并当前节点，并返回处理后的公共体素状态。
    VoxelState merge();

    /// 掩码叶块访问

    // 返回当前节点直接对应的可写掩码叶块。
    VoxelLeafBlock& maskLeaf();

    // 返回当前节点直接对应的只读掩码叶块。
    const VoxelLeafBlock& maskLeaf() const;

private:
    enum class ReferenceType
    {
        Root,
        BranchCell,
        MaskLeaf,
        MaskOctant,
        MaskVoxel
    };

    VoxelPackedTreeEditor(
        VoxelPackedRootTree& tree,
        ReferenceType referenceType,
        VoxelNodeBlock* parentBlock,
        VoxelCorner corner,
        VoxelLeafBlock* leafBlock,
        VoxelCorner coarseCorner,
        VoxelCorner fineCorner);

    // 返回当前普通已细分节点用于描述八个直接子单元的节点块。
    VoxelNodeBlock& currentBlock() const;

    // 将当前掩码叶块在完全为空或完全为材料时折叠回普通叶节点。
    VoxelState mergeMaskLeaf();

    // 检查当前编辑器引用和底层树结构是否有效。
    bool isValid() const;

    VoxelPackedRootTree* m_tree; // 当前编辑节点所属的独立根树。
    VoxelNodeBlock* m_parentBlock; // 非根节点或掩码叶块在普通树中的父节点块。
    VoxelLeafBlock* m_leafBlock; // 当前掩码叶块及其内部逻辑节点使用的物理叶块。
    VoxelCorner m_corner; // 当前普通节点或掩码叶块在父节点块中的角点。
    VoxelCorner m_coarseCorner; // 掩码叶块内部第一级角点。
    VoxelCorner m_fineCorner; // 掩码叶块内部第二级角点。
    ReferenceType m_referenceType; // 当前编辑器引用类型。
};

}

#endif // MYVOXEL_VOXELPACKEDTREEEDITOR_H