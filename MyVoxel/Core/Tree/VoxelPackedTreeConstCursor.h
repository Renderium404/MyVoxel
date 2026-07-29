#ifndef MYVOXEL_VOXELPACKEDTREECONSTCURSOR_H
#define MYVOXEL_VOXELPACKEDTREECONSTCURSOR_H

#include "VoxelPackedRootTree.h"

namespace MyVoxel
{

// 在只读Packed根树中访问普通树节点、掩码叶块及其内部逻辑节点。
class VoxelPackedTreeConstCursor
{
public:
    // 使用指定只读根树创建根节点游标。
    explicit VoxelPackedTreeConstCursor(const VoxelPackedRootTree& tree);

    /// 节点状态

    // 返回当前逻辑节点状态。
    VoxelState state() const;

    // 检查当前节点是否直接对应一个掩码叶块。
    bool isMaskLeaf() const;

    // 检查当前节点是否允许继续访问逻辑子节点。
    bool canAccessChildren() const;

    /// 子节点访问

    // 返回指定角点对应的直接逻辑子节点游标。
    VoxelPackedTreeConstCursor child(VoxelCorner corner) const;

    /// 掩码叶块访问

    // 返回当前节点直接对应的只读掩码叶块。
    const VoxelLeafBlock& maskLeaf() const;

private:
    enum class ReferenceType
    {
        Root,
        BranchCell,
        MaskLeaf,
        MaskOctant,
        MaskVoxel,
        VirtualLeaf
    };

    VoxelPackedTreeConstCursor(
        const VoxelPackedRootTree& tree,
        ReferenceType referenceType,
        const VoxelNodeBlock* parentBlock,
        VoxelCorner corner,
        const VoxelLeafBlock* leafBlock,
        VoxelCorner coarseCorner,
        VoxelCorner fineCorner,
        VoxelState virtualState);

    // 返回当前普通已细分节点用于描述八个直接子单元的节点块。
    const VoxelNodeBlock& currentBlock() const;

    // 检查当前游标引用和底层树结构是否有效。
    bool isValid() const;

    const VoxelPackedRootTree* m_tree; // 当前游标所属的只读根树。
    const VoxelNodeBlock* m_parentBlock; // 非根普通节点或掩码叶块的父节点块。
    const VoxelLeafBlock* m_leafBlock; // 当前掩码叶块及其内部逻辑节点使用的物理叶块。
    VoxelCorner m_corner; // 当前普通节点或掩码叶块在父节点块中的角点。
    VoxelCorner m_coarseCorner; // 掩码叶块内部第一级角点。
    VoxelCorner m_fineCorner; // 掩码叶块内部第二级角点。
    VoxelState m_virtualState; // 虚拟叶节点继承的空或材料状态。
    ReferenceType m_referenceType; // 当前游标引用类型。
};

}

#endif // MYVOXEL_VOXELPACKEDTREECONSTCURSOR_H