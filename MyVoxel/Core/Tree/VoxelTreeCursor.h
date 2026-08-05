#ifndef MYVOXEL_CORE_TREE_VOXELTREECURSOR_H
#define MYVOXEL_CORE_TREE_VOXELTREECURSOR_H

#include <cassert>
#include <cstdint>

#include "MyVoxel/Core/Tree/VoxelChildStateMasks.h"

#include "VoxelTree.h"

namespace MyVoxel
{

// 在只读体素树中访问逻辑体素及其直接子体素。
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
    // Empty或Material体素不会实际细分，而是返回八个继承当前状态的虚拟子体素。
    VoxelChildStateMasks childStateMasks() const;
    // 返回指定角点对应的直接逻辑子体素游标，当前体素必须已经细分。
    VoxelTreeCursor child(VoxelCorner corner) const;

    /// 压缩材料读取
    // 判断当前逻辑体素是否直接具有覆盖后两层的64位材料掩码。
    bool hasMaterialMask() const{ return m_source == Source::MaskLeaf;}
    // 返回当前逻辑体素覆盖后两层的64位材料掩码。
    std::uint64_t materialMask() const;
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
    VoxelTreeCursor(const VoxelTree& tree, VoxelState state);
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
    // 返回掩码叶块粗层节点中的细层子体素游标。
    VoxelTreeCursor maskLeafGroupChild(VoxelCorner corner) const;

private:
    const VoxelTree* m_tree; // 当前游标所属的只读体素树。
    const VoxelNodeBlock* m_nodeBlock; // 当前使用的普通节点块。
    const VoxelLeafBlock* m_leafBlock; // 当前使用的掩码叶块。
    VoxelCorner m_coarseCorner; // 当前掩码叶块粗层节点对应的角点。
    VoxelState m_state; // 当前逻辑体素状态。
    Source m_source; // 当前逻辑体素的底层数据来源。
};

}

#endif // MYVOXEL_CORE_TREE_VOXELTREECURSOR_H