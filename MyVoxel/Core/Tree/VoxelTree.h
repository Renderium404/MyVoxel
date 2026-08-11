#ifndef MYVOXEL_CORE_TREE_VOXELTREE_H
#define MYVOXEL_CORE_TREE_VOXELTREE_H

#include <cstddef>

#include "../Storage/BlockPool.h"
#include "../Storage/NodeBlock.h"
#include "../../Foundation/RefPtr.h"

namespace MyVoxel
{

class VoxelTreeCursor;
class VoxelTreeEditor;
class VoxelTreeOperation;

// 保存一个根体素、根节点描述块及其共享完整物理块池。
//
// BlockPool统一管理NodePool和LeafPool，复制VoxelTree时共享完整物理存储。
// 首次创建编辑器时由VoxelTree执行写时复制，使节点和MaskLeaf数据始终成组分离。
class VoxelTree
{
public:
    // 创建完全为空或完全包含材料的体素树。
    explicit VoxelTree(VoxelState state = VoxelState::Empty);
    // 复制体素树并共享完整物理块池。
    VoxelTree(const VoxelTree& other) = default;
    // 移动体素树并转移完整物理块池所有权。
    VoxelTree(VoxelTree&& other);

    // 复制体素树并共享完整物理块池。
    VoxelTree& operator=(const VoxelTree& other) = default;
    // 移动体素树并转移完整物理块池所有权。
    VoxelTree& operator=(VoxelTree&& other);

    /// 根节点状态

    // 返回根体素当前逻辑状态。
    VoxelState state() const;

    /// 树访问入口

    // 创建指向根体素的只读游标。
    VoxelTreeCursor cursor() const;
    // 创建指向根体素的可写编辑器，必要时分离共享BlockPool。
    VoxelTreeEditor editor();

    /// 资源管理

    // 将体素树重置为空或材料状态，回卷全部节点和叶数据但保留Chunk。
    void reset(VoxelState state = VoxelState::Empty);
    // 将体素树重置为空或材料状态，并释放当前持有的全部BlockPool存储。
    void release(VoxelState state = VoxelState::Empty);

    /// 节点存储统计

    // 返回当前实际分配的八槽节点组数量。
    std::size_t allocatedGroupCount() const;
    // 返回节点组历史使用数量高水位。
    std::size_t highWaterGroupCount() const;
    // 返回NodePool当前Chunk数量。
    std::size_t nodeChunkCount() const;
    // 返回NodePool当前存储容量。
    std::size_t nodeStorageCapacityBytes() const;

    /// 叶存储统计

    // 返回当前实际分配的MaskLeaf数量。
    std::size_t allocatedLeafCount() const;
    // 返回MaskLeaf历史使用数量高水位。
    std::size_t highWaterLeafCount() const;
    // 返回LeafPool当前Chunk数量。
    std::size_t leafChunkCount() const;
    // 返回LeafPool当前存储容量。
    std::size_t leafStorageCapacityBytes() const;

    /// 总存储统计

    // 返回当前BlockPool节点和叶数据总容量。
    std::size_t storageCapacityBytes() const;

    /// 结构检查

    // 检查根状态、全部可达Node、MaskLeaf、Mask/Distance一致性和池分配数量是否一致。
    bool isValid() const;
    // 检查树是否已经消除全部不损失距离场信息的可折叠节点。
    bool isNormalized() const;

private:
    // 必要时深复制全部可达Node和MaskLeaf，使本树独占完整BlockPool。
    void ensureUniqueStorage();

private:
    friend class VoxelTreeCursor;
    friend class VoxelTreeEditor;
    friend class VoxelTreeOperation;

private:
    VoxelState m_rootState; // 根体素自身的逻辑状态。
    NodeBlock m_rootBlock; // 根体素细分后描述其八个直接子体素。
    Foundation::RefPtr<BlockPool> m_blockPool; // 当前体素树共享的完整物理块池。
};

}

#endif // MYVOXEL_CORE_TREE_VOXELTREE_H