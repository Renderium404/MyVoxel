#ifndef MYVOXEL_CORE_TREE_VOXELTREECURSOR_H
#define MYVOXEL_CORE_TREE_VOXELTREECURSOR_H

#include <cassert>
#include <cstdint>

#include "MyVoxel/Core/Tree/VoxelChildStateMasks.h"
#include "VoxelTree.h"

namespace MyVoxel
{

// 在只读体素树中访问逻辑体素、MaskLeaf及其距离样本。
class VoxelTreeCursor
{
public:
    // 使用指定只读体素树创建根体素游标。
    explicit VoxelTreeCursor(const VoxelTree& tree);

    /// 体素状态

    VoxelState state() const{return m_state;}
    bool isEmpty() const{return m_state == VoxelState::Empty;}
    bool isMaterial() const{return m_state == VoxelState::Material;}
    bool isSubdivided() const{return m_state == VoxelState::Subdivided;}
    bool isTerminal() const{return m_state != VoxelState::Subdivided;}

    /// 子体素读取

    // 一次返回当前逻辑体素八个直接子体素的状态掩码。
    VoxelChildStateMasks childStateMasks() const;
    // 返回指定角点对应的直接逻辑子体素游标。
    VoxelTreeCursor child(VoxelCorner corner) const;

    /// MaskLeaf读取

    // 判断当前游标是否直接位于一个完整MaskLeaf入口。
    bool hasLeafData() const{return m_source == Source::MaskLeaf;}
    // 返回当前MaskLeaf的材料符号缓存。
    const MaskBlock& maskBlock() const;
    // 返回当前MaskLeaf的64个距离样本。
    const LeafBlock& leafBlock() const;
    // 返回当前MaskLeaf的64位材料符号缓存。
    std::uint64_t materialMask() const;

    /// 距离读取

    // 判断当前终止逻辑位置是否具有可解释距离。
    // 普通Empty/Material终止节点分别使用正无穷和负无穷，MaskLeaf最细样本返回实际距离。
    bool hasDistance() const{return isTerminal();}
    // 返回当前终止逻辑位置的距离。
    float distance() const;

private:
    enum class Source : std::uint8_t
    {
        Terminal = 0, // Empty或Material隐式终止节点。
        NodeBlock = 1, // 子体素由NodeBlock描述。
        MaskLeaf = 2, // 完整MaskLeaf入口。
        MaskLeafGroup = 3, // MaskLeaf中的一个粗层组。
        MaskLeafVoxel = 4 // MaskLeaf中的一个实际距离样本。
    };

private:
    VoxelTreeCursor(const VoxelTree& tree, VoxelState state);
    VoxelTreeCursor(const VoxelTree& tree, const NodeBlock& block);
    VoxelTreeCursor(const VoxelTree& tree, const MaskBlock& maskBlock, const LeafBlock& leafBlock);
    VoxelTreeCursor(const VoxelTree& tree, const MaskBlock& maskBlock, const LeafBlock& leafBlock, VoxelCorner coarseCorner);
    VoxelTreeCursor(const VoxelTree& tree, const MaskBlock& maskBlock, const LeafBlock& leafBlock, VoxelCorner coarseCorner, VoxelCorner fineCorner);

    VoxelTreeCursor nodeChild(VoxelCorner corner) const;
    VoxelTreeCursor maskLeafChild(VoxelCorner corner) const;
    VoxelTreeCursor maskLeafGroupChild(VoxelCorner corner) const;

private:
    const VoxelTree* m_tree; // 当前游标所属体素树。
    const NodeBlock* m_nodeBlock; // 当前普通节点块。
    const MaskBlock* m_maskBlock; // 当前MaskLeaf材料符号缓存。
    const LeafBlock* m_leafBlock; // 当前MaskLeaf距离块。
    VoxelCorner m_coarseCorner; // MaskLeaf粗层角点。
    VoxelCorner m_fineCorner; // MaskLeaf细层角点。
    VoxelState m_state; // 当前逻辑状态。
    Source m_source; // 当前底层数据来源。
};

}

#endif // MYVOXEL_CORE_TREE_VOXELTREECURSOR_H