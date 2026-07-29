#ifndef MYVOXEL_VOXELTREEEDITOR_H
#define MYVOXEL_VOXELTREEEDITOR_H

#include "VoxelForest.h"
#include "VoxelPackedTreeEditor.h"
#include "VoxelTree.h"

namespace MyVoxel
{

// 使用生产接口修改Packed体素树节点。
class VoxelTreeEditor
{
public:
    // 使用森林中已经存在的节点创建编辑器，并在根树共享时复制目标根树。
    VoxelTreeEditor(VoxelForest& forest, const VoxelCellAddress& address);

    // 使用已经完成写时复制的独立根树创建根节点编辑器。
    explicit VoxelTreeEditor(VoxelTree& tree);

    /// 节点状态

    // 返回当前编辑节点的状态。
    VoxelState state() const;

    // 将当前节点设置为空或材料状态，并释放原有全部后代节点组。
    void setState(VoxelState state);

    /// 节点结构

    // 将当前叶节点细分为八个继承原状态的普通子节点，未发生结构变化时返回false。
    bool split();

    // 将当前非根普通叶节点转换为包含下面两层的掩码叶块。
    bool makeMaskLeaf();

    // 检查当前节点是否直接对应一个掩码叶块。
    bool isMaskLeaf() const;

    // 检查当前节点是否允许继续访问逻辑子节点。
    bool canAccessChildren() const;

    // 返回指定角点对应的直接子节点编辑器。
    VoxelTreeEditor child(VoxelCorner corner) const;

    // 尝试合并当前节点，并返回处理后的状态。
    VoxelState merge();

    /// 掩码叶块访问

    // 返回当前节点直接对应的可写掩码叶块。
    VoxelLeafBlock& maskLeaf();

    // 返回当前节点直接对应的只读掩码叶块。
    const VoxelLeafBlock& maskLeaf() const;

private:
    // 使用已经定位的Packed编辑器创建生产子节点编辑器。
    explicit VoxelTreeEditor(const VoxelPackedTreeEditor& editor);

    // 返回森林中指定地址对应的Packed编辑器。
    static VoxelPackedTreeEditor createEditor(VoxelForest& forest, const VoxelCellAddress& address);

    VoxelPackedTreeEditor m_editor; // 当前实际执行节点修改的Packed编辑器。
};

}

#endif // MYVOXEL_VOXELTREEEDITOR_H