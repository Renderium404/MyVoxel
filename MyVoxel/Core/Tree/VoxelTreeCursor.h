#ifndef MYVOXEL_CORE_TREE_VOXELTREECURSOR_H
#define MYVOXEL_CORE_TREE_VOXELTREECURSOR_H

#include <cassert>
#include <cstdint>

#include "VoxelTree.h"

namespace MyVoxel
{

// 表示游标当前逻辑体素的状态，不描述底层物理存储类型。
enum class VoxelCursorState : std::uint8_t
{
    Empty = 0, // 当前逻辑体素为空。
    Material = 1, // 当前逻辑体素完全包含材料。
    Subdivided = 2 // 当前逻辑体素已经细分，允许继续访问八个子体素。
};

// 在只读体素树中访问普通节点、掩码叶块及掩码叶块内部逻辑节点。
class VoxelTreeCursor
{
public:
    // 使用指定只读体素树创建根节点游标。
    explicit VoxelTreeCursor(const VoxelTree& tree);

    /// 节点状态

    // 返回当前逻辑体素状态。
    VoxelCursorState state() const;

    // 判断当前逻辑体素是否为空。
    bool isEmpty() const;

    // 判断当前逻辑体素是否完全包含材料。
    bool isMaterial() const;

    // 判断当前逻辑体素是否已经细分。
    bool isSubdivided() const;

    // 判断当前逻辑体素是否为终止体素。
    bool isTerminal() const;

    // 判断当前逻辑体素是否允许继续访问八个子体素。
    bool canAccessChildren() const;

    /// 子节点访问

    // 返回指定角点对应的直接逻辑子体素游标。
    VoxelTreeCursor child(VoxelCorner corner) const;

    /// 底层存储

    // 判断当前逻辑体素是否直接使用普通节点块描述子体素。
    bool usesNodeBlock() const;

    // 判断当前逻辑体素是否位于掩码叶块中。
    bool usesMaskLeaf() const;

    // 返回当前逻辑体素使用的普通节点块。
    const VoxelNodeBlock& nodeBlock() const;

    // 返回当前逻辑体素使用的掩码叶块。
    const VoxelLeafBlock& maskLeaf() const;

private:
    // 表示当前游标使用的底层数据来源。
    enum class Source : std::uint8_t
    {
        Terminal = 0, // 当前体素为空或材料，不使用底层描述块。
        NodeBlock = 1, // 当前体素的子体素由VoxelNodeBlock描述。
        MaskLeaf = 2, // 当前体素是一个掩码叶块入口。
        MaskLeafGroup = 3 // 当前体素是掩码叶块中的一个粗层逻辑节点。
    };

private:
    // 创建空或材料终止游标。
    VoxelTreeCursor(const VoxelTree& tree, VoxelCursorState state);

    // 创建使用普通节点块的已细分游标。
    VoxelTreeCursor(const VoxelTree& tree, const VoxelNodeBlock& block);

    // 创建掩码叶块入口游标。
    VoxelTreeCursor(const VoxelTree& tree, const VoxelLeafBlock& leafBlock);

    // 创建掩码叶块内部粗层节点游标。
    VoxelTreeCursor(const VoxelTree& tree, const VoxelLeafBlock& leafBlock, VoxelCorner coarseCorner);

    // 根据普通节点记录的子节点状态创建游标。
    VoxelTreeCursor nodeChild(VoxelCorner corner) const;

    // 返回掩码叶块入口的粗层子节点游标。
    VoxelTreeCursor maskLeafChild(VoxelCorner corner) const;

    // 返回掩码叶块粗层节点中的最高层子体素游标。
    VoxelTreeCursor maskLeafGroupChild(VoxelCorner corner) const;

private:
    const VoxelTree* m_tree; // 当前游标所属的只读体素树。
    const VoxelNodeBlock* m_nodeBlock; // 当前使用的普通节点块。
    const VoxelLeafBlock* m_leafBlock; // 当前使用的掩码叶块。
    VoxelCorner m_coarseCorner; // 当前掩码叶块粗层节点对应的角点。
    VoxelCursorState m_state; // 当前逻辑体素状态。
    Source m_source; // 当前逻辑体素的底层数据来源。
};

}

#endif // MYVOXEL_CORE_TREE_VOXELTREECURSOR_H