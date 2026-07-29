#ifndef MYVOXEL_CORE_TREE_VOXELTREECONSTCURSOR_H
#define MYVOXEL_CORE_TREE_VOXELTREECONSTCURSOR_H

#include "MyVoxel/Core/VoxelAddress.h"
#include "VoxelForest.h"
#include "VoxelTree.h"

namespace MyVoxel
{

// 在只读体素树中直接访问当前节点及其逻辑子节点。
class VoxelTreeConstCursor
{
public:
    // 使用森林中的指定体素地址创建只读游标，不存在实际节点时表示对应的虚拟叶节点。
    VoxelTreeConstCursor(const VoxelForest& forest, const VoxelCellAddress& address);
    // 使用指定体素树的根节点创建只读游标。
    explicit VoxelTreeConstCursor(const VoxelTree& tree);

    /// 节点状态
    // 返回当前节点状态。
    VoxelState state() const;
    // 判断当前游标是否表示实际创建的节点。
    bool hasActualNode() const;
    // 判断当前游标是否表示叶节点向下延伸形成的虚拟节点。
    bool isVirtual() const;
    // 判断当前节点是否为空叶节点。
    bool isEmpty() const;
    // 判断当前节点是否为材料叶节点。
    bool isMaterial() const;
    // 判断当前节点是否已经细分。
    bool isSubdivided() const;
    // 判断当前节点是否为叶节点。
    bool isLeaf() const;

    /// 子节点访问
    // 返回指定角点对应的子节点游标，叶节点的子游标继承当前叶节点状态。
    VoxelTreeConstCursor child(VoxelCorner corner) const;

private:
    // 使用已经定位的体素树和实际节点创建游标。
    VoxelTreeConstCursor(const VoxelTree* tree, const VoxelNode* node);
    // 使用指定叶节点状态创建虚拟游标。
    VoxelTreeConstCursor(const VoxelTree* tree, VoxelState virtualState);

private:
    const VoxelTree* m_tree; // 当前实际节点所属的只读体素树，森林中不存在根树时为空。
    const VoxelNode* m_node; // 当前实际节点，虚拟节点时为空。
    VoxelState m_virtualState; // 当前虚拟节点继承的叶节点状态。
    bool m_virtual; // 当前游标是否表示虚拟节点。
};

}

#endif // MYVOXEL_CORE_TREE_VOXELTREECONSTCURSOR_H