#ifndef MYVOXEL_CORE_TREE_VOXELTREEEDITOR_H
#define MYVOXEL_CORE_TREE_VOXELTREEEDITOR_H

#include <cstdint>

#include "MyVoxel/Core/Tree/VoxelChildStateMasks.h"
#include "VoxelTree.h"

namespace MyVoxel
{

// 在可写体素树中访问并修改逻辑结构、终止Tile Value和显式TSDF距离。
//
// 不带backgroundDistance创建的编辑器可执行保持现有Value的结构操作以及直接setValue。
// setEmpty、setMaterial、setChildrenState和二值Mask入口需要backgroundDistance以生成±B。
class VoxelTreeEditor
{
public:
    // 使用指定体素树创建编辑器，并保证当前树独占BlockPool。
    explicit VoxelTreeEditor(VoxelTree& tree);
    // 使用指定截断背景距离创建场编辑器，并保证当前树独占BlockPool。
    VoxelTreeEditor(VoxelTree& tree, float backgroundDistance);

    /// 体素状态

    VoxelState state() const;
    bool isEmpty() const{return state() == VoxelState::Empty;}
    bool isMaterial() const{return state() == VoxelState::Material;}
    bool isSubdivided() const{return state() == VoxelState::Subdivided;}
    bool isTerminal() const{return state() != VoxelState::Subdivided;}
    // 判断当前编辑器是否具有可解释±B和TSDF范围的backgroundDistance。
    bool hasBackgroundDistance() const{return m_backgroundDistance > 0.0f;}
    // 返回当前编辑器使用的截断背景距离。
    float backgroundDistance() const;

    /// 子体素读取

    // 一次返回当前逻辑体素八个直接子体素的结构状态掩码。
    VoxelChildStateMasks childStateMasks() const;

    /// 结构和终止Value修改

    // 将当前逻辑区域设置为+B空侧终止Tile，需要field-aware editor。
    void setEmpty();
    // 将当前逻辑区域设置为-B材料侧终止Tile，需要field-aware editor。
    void setMaterial();
    // 将当前逻辑区域设置为指定终止Tile Value并释放更细物理后代，零值视为Material。
    bool setValue(float value);
    // 将当前Empty或Material终止Tile细分为普通Node，八个直接子Tile继承原Value。
    bool subdivide();
    // 将指定直接子体素批量设置为+B空侧或-B材料侧终止Tile，需要field-aware editor。
    bool setChildrenState(std::uint8_t childMask, VoxelState state);
    // 返回指定角点对应的直接逻辑子体素编辑器；普通终止Tile会按需无损细分为Node。
    VoxelTreeEditor child(VoxelCorner corner);

    /// MaskLeaf结构

    // 判断当前位置是否能够被显式化为完整MaskLeaf。
    bool canTouchLeaf() const;
    // 将当前Empty或Material NodeChild无损显式化为MaskLeaf，64个距离统一继承当前Tile Value。
    bool touchLeaf();
    // 判断当前位置是否能够直接替换完整MaskLeaf数据。
    bool canSetLeafBlock() const{return m_source == Source::NodeChild;}
    // 使用指定64个截断距离替换当前位置；64个Value完全一致时直接保存为终止Tile。
    bool setLeafBlock(const LeafBlock& block);
    // 使用二值材料掩码生成±B距离并替换当前位置，需要field-aware editor。
    bool setMaterialMask(std::uint64_t materialMask);

    /// 单距离修改

    // 判断当前位置是否对应MaskLeaf中的实际距离样本。
    bool canSetDistance() const{return m_source == Source::MaskLeafVoxel;}
    // 返回当前位置实际距离样本。
    float distance() const;
    // 修改当前位置实际距离并同步MaskBlock；field-aware editor会同时检查[-B,+B]。
    bool setDistance(float distance);

private:
    enum class Source : std::uint8_t
    {
        Root = 0, // 当前编辑器对应体素树根体素。
        NodeChild = 1, // 当前编辑器对应NodeBlock中的一个直接子体素。
        MaskLeafGroup = 2, // 当前编辑器对应MaskLeaf中的一个粗层组。
        MaskLeafVoxel = 3 // 当前编辑器对应MaskLeaf中的一个实际距离样本。
    };

private:
    VoxelTreeEditor(VoxelTree& tree, NodeBlock& parentBlock, VoxelCorner childCorner, float backgroundDistance);
    VoxelTreeEditor(VoxelTree& tree, MaskBlock& maskBlock, LeafBlock& leafBlock, VoxelCorner coarseCorner, float backgroundDistance);
    VoxelTreeEditor(VoxelTree& tree, MaskBlock& maskBlock, LeafBlock& leafBlock, VoxelCorner coarseCorner, VoxelCorner fineCorner, float backgroundDistance);

    VoxelNodeState nodeChildState() const;
    NodeBlock& currentNodeBlock();
    const NodeBlock& currentNodeBlock() const;
    LeafData currentLeafData();
    ConstLeafData currentLeafData() const;
    float terminalDistance(VoxelState state) const;
    void requireBackgroundDistance() const;
    bool setNodeChildren(NodeBlock& block, std::uint8_t childMask, float value);
    bool setLeafGroups(std::uint8_t groupMask, float value);
    bool setLeafGroupChildren(std::uint8_t childMask, float value);

private:
    VoxelTree* m_tree; // 当前所属可写体素树。
    NodeBlock* m_parentBlock; // NodeChild对应父节点。
    MaskBlock* m_maskBlock; // 当前MaskLeaf材料二值掩码。
    LeafBlock* m_leafBlock; // 当前MaskLeaf距离块。
    VoxelCorner m_childCorner; // NodeChild或细层样本角点。
    VoxelCorner m_coarseCorner; // MaskLeaf粗层角点。
    float m_backgroundDistance; // 当前场编辑器使用的截断背景距离，0表示未提供。
    Source m_source; // 当前逻辑位置来源。
};

}

#endif // MYVOXEL_CORE_TREE_VOXELTREEEDITOR_H