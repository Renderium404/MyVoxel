#ifndef MYVOXEL_CORE_TREE_VOXELTREECURSOR_H
#define MYVOXEL_CORE_TREE_VOXELTREECURSOR_H

#include <cassert>
#include <cstdint>

#include "MyVoxel/Core/Tree/VoxelChildStateMasks.h"
#include "VoxelTree.h"

namespace MyVoxel
{

// 在只读体素树中访问逻辑结构、终止Tile Value、MaskLeaf及其显式距离样本。
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

    // 一次返回当前逻辑体素八个直接子体素的结构状态掩码。
    VoxelChildStateMasks childStateMasks() const;
    // 返回指定角点对应的直接逻辑子体素游标。
    VoxelTreeCursor child(VoxelCorner corner) const;

    /// MaskLeaf读取

    // 判断当前游标是否直接位于完整MaskLeaf入口。
    bool hasLeafData() const{return m_source == Source::MaskLeaf;}
    // 返回当前完整MaskLeaf的材料二值掩码。
    const MaskBlock& maskBlock() const;
    // 返回当前完整MaskLeaf的64个显式距离样本。
    const LeafBlock& leafBlock() const;
    // 返回当前完整MaskLeaf的64位材料二值掩码。
    std::uint64_t materialMask() const;

    /// 距离读取

    // 判断当前位置是否具有直接可读取的Tile Value或LeafBlock实际距离样本。
    bool hasDistance() const{return m_source == Source::Terminal || m_source == Source::MaskLeafVoxel;}
    // 判断当前位置是否专门对应LeafBlock中的一个实际距离样本。
    bool hasLeafDistance() const{return m_source == Source::MaskLeafVoxel;}
    // 返回当前位置实际距离；终止区域返回自身Tile Value，最细叶体素返回LeafBlock样本。
    float distance() const;

private:
    enum class Source : std::uint8_t
    {
        Terminal = 0, // Empty或Material终止Tile，保存实际Value。
        NodeBlock = 1, // 当前逻辑体素由NodeBlock描述。
        MaskLeaf = 2, // 当前逻辑体素对应完整MaskLeaf入口。
        MaskLeafGroup = 3, // 当前逻辑体素对应MaskLeaf中的一个粗层组。
        MaskLeafVoxel = 4 // 当前逻辑体素对应MaskLeaf中的一个实际距离样本。
    };

private:
    VoxelTreeCursor(const VoxelTree& tree, VoxelState state, float value);
    VoxelTreeCursor(const VoxelTree& tree, const NodeBlock& block);
    VoxelTreeCursor(const VoxelTree& tree, const MaskBlock& maskBlock, const LeafBlock& leafBlock);
    VoxelTreeCursor(const VoxelTree& tree, const MaskBlock& maskBlock, const LeafBlock& leafBlock, VoxelCorner coarseCorner);
    VoxelTreeCursor(const VoxelTree& tree, const MaskBlock& maskBlock, const LeafBlock& leafBlock, VoxelCorner coarseCorner, VoxelCorner fineCorner);

    VoxelTreeCursor nodeChild(VoxelCorner corner) const;
    VoxelTreeCursor maskLeafChild(VoxelCorner corner) const;
    VoxelTreeCursor maskLeafGroupChild(VoxelCorner corner) const;

private:
    const VoxelTree* m_tree;        // 当前游标所属体素树。
    const NodeBlock* m_nodeBlock;   // 当前普通分化节点块。
    const MaskBlock* m_maskBlock;   // 当前MaskLeaf材料二值掩码。
    const LeafBlock* m_leafBlock;   // 当前MaskLeaf距离块。
    VoxelCorner m_coarseCorner;     // MaskLeaf粗层角点。
    VoxelCorner m_fineCorner;       // MaskLeaf细层角点。
    float m_value;                  // Terminal来源保存的实际Tile Value。
    VoxelState m_state;             // 当前逻辑结构状态。
    Source m_source;                // 当前底层数据来源。
};

}

#endif // MYVOXEL_CORE_TREE_VOXELTREECURSOR_H
