#ifndef MYVOXEL_CORE_TREE_VOXELTREEEDITOR_H
#define MYVOXEL_CORE_TREE_VOXELTREEEDITOR_H

#include <cstdint>

#include "MyVoxel/Core/Tree/VoxelChildStateMasks.h"
#include "VoxelTree.h"

namespace MyVoxel
{

// 在可写体素树中访问并修改逻辑体素和MaskLeaf距离场。
//
// Empty和Material终止状态分别使用正无穷和负无穷作为隐式距离。
// 编辑MaskLeaf时距离为几何真值，MaskBlock始终由距离符号同步维护。
class VoxelTreeEditor
{
public:
    // 使用指定体素树创建根编辑器，并保证当前树独占完整BlockPool。
    explicit VoxelTreeEditor(VoxelTree& tree);

    /// 体素状态

    VoxelState state() const;
    bool isEmpty() const{return state() == VoxelState::Empty;}
    bool isMaterial() const{return state() == VoxelState::Material;}
    bool isSubdivided() const{return state() == VoxelState::Subdivided;}
    bool isTerminal() const{return state() != VoxelState::Subdivided;}

    /// 子体素读取

    // 一次返回当前逻辑体素八个直接子体素状态。
    VoxelChildStateMasks childStateMasks() const;

    /// 结构和终止状态修改

    // 将当前逻辑体素设置为空；显式距离位置写入正无穷。
    void setEmpty();
    // 将当前逻辑体素设置为材料；显式距离位置写入负无穷。
    void setMaterial();
    // 将当前空或材料体素细分为普通Node。
    bool subdivide();
    // 将指定直接子体素批量设置为空或材料。
    bool setChildrenState(std::uint8_t childMask, VoxelState state);
    // 返回指定直接子体素编辑器。
    VoxelTreeEditor child(VoxelCorner corner);

    /// MaskLeaf修改

    // 判断当前位置是否允许直接写入完整LeafBlock。
    bool canSetLeafBlock() const{return m_source == Source::NodeChild;}
    // 使用指定64个距离样本替换当前位置的完整MaskLeaf，MaskBlock自动由距离符号重建。
    bool setLeafBlock(const LeafBlock& block);

    // 使用64位二值状态覆盖当前位置。
    // Material位写为负无穷，Empty位写为正无穷，因此不会产生Mask/Distance不一致。
    bool setMaterialMask(std::uint64_t materialMask);

    /// 单距离修改

    // 判断当前位置是否对应MaskLeaf中的实际距离样本。
    bool canSetDistance() const{return m_source == Source::MaskLeafVoxel;}
    // 返回当前位置实际距离样本。
    float distance() const;
    // 修改当前位置实际距离并同步MaskBlock。
    bool setDistance(float distance);

private:
    enum class Source : std::uint8_t
    {
        Root = 0,
        NodeChild = 1,
        MaskLeafGroup = 2,
        MaskLeafVoxel = 3
    };

private:
    VoxelTreeEditor(VoxelTree& tree, NodeBlock& parentBlock, VoxelCorner childCorner);
    VoxelTreeEditor(VoxelTree& tree, MaskBlock& maskBlock, LeafBlock& leafBlock, VoxelCorner coarseCorner);
    VoxelTreeEditor(VoxelTree& tree, MaskBlock& maskBlock, LeafBlock& leafBlock, VoxelCorner coarseCorner, VoxelCorner fineCorner);

    VoxelNodeState nodeChildState() const;
    NodeBlock& currentNodeBlock();
    const NodeBlock& currentNodeBlock() const;
    LeafData currentLeafData();
    ConstLeafData currentLeafData() const;

    void setTerminal(VoxelState state);
    bool setNodeChildren(NodeBlock& block, std::uint8_t childMask, VoxelState state);
    bool setLeafGroups(std::uint8_t groupMask, VoxelState state);
    bool setLeafGroupChildren(std::uint8_t childMask, VoxelState state);
    void ensureParentGroup();
    void releaseUnusedParentGroup();

private:
    VoxelTree* m_tree; // 当前所属可写体素树。
    NodeBlock* m_parentBlock; // NodeChild对应父节点。
    MaskBlock* m_maskBlock; // 当前MaskLeaf符号缓存。
    LeafBlock* m_leafBlock; // 当前MaskLeaf距离块。
    VoxelCorner m_childCorner; // NodeChild或细层角点。
    VoxelCorner m_coarseCorner; // MaskLeaf粗层角点。
    Source m_source; // 当前逻辑位置来源。
};

}

#endif // MYVOXEL_CORE_TREE_VOXELTREEEDITOR_H