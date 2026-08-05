#ifndef MYVOXEL_CORE_TREE_VOXELTREE_H
#define MYVOXEL_CORE_TREE_VOXELTREE_H

#include <cstddef>

#include "../Storage/VoxelBlock.h"
#include "../Storage/VoxelBlockPool.h"
#include "../../Foundation/RefPtr.h"

namespace MyVoxel
{

class VoxelTreeCursor;
class VoxelTreeEditor;
class VoxelTreeOperation;

// 保存一个根体素、根节点描述块及其共享物理节点池。
//
// 复制VoxelTree时共享物理节点池，首次创建编辑器时由VoxelTree保证存储独占。
// 移动VoxelTree时直接转移节点池所有权，不增加引用计数。
class VoxelTree
{
public:
    // 创建完全为空或完全包含材料的体素树。
    explicit VoxelTree(VoxelState state = VoxelState::Empty);
    // 复制体素树并共享物理节点池。
    VoxelTree(const VoxelTree& other) = default;
    // 移动体素树并转移节点池所有权，other仅保留可析构和可重新赋值状态。
    VoxelTree(VoxelTree&& other);

    // 复制体素树并共享物理节点池。
    VoxelTree& operator=(const VoxelTree& other) = default;
    // 移动体素树并转移节点池所有权，other仅保留可析构和可重新赋值状态。
    VoxelTree& operator=(VoxelTree&& other);

    /// 根节点状态
    // 返回根体素当前逻辑状态。
    VoxelState state() const;

    /// 树访问入口

    // 创建指向根体素的只读游标。
    VoxelTreeCursor cursor() const;
    // 创建指向根体素的可写编辑器，必要时分离共享节点池。
    VoxelTreeEditor editor();

    /// 资源管理

    // 将体素树重置为空或材料状态，回收全部逻辑节点组但保留节点池Chunk。
    void reset(VoxelState state = VoxelState::Empty);
    // 将体素树重置为空或材料状态，并释放当前体素树持有的全部节点池存储。
    void release(VoxelState state = VoxelState::Empty);

    /// 存储统计
    // 返回当前实际分配并被逻辑树引用的八槽组数量。
    std::size_t allocatedGroupCount() const;
    // 返回当前节点池曾经创建的八槽组数量高水位。
    std::size_t highWaterGroupCount() const;
    // 返回当前节点池持有的64 KiB Chunk数量。
    std::size_t chunkCount() const;
    // 返回当前节点池持有的Chunk总容量，单位为字节。
    std::size_t storageCapacityBytes() const;

    /// 结构检查

    // 检查根状态、保留字段、全部可达节点及节点池分配状态是否一致。
    bool isValid() const;
    // 检查树是否已经折叠全部可合并的普通节点和掩码叶节点。
    bool isNormalized() const;

private:
    // 必要时深复制当前可达节点，使本树独占物理节点池。
    void ensureUniqueStorage();

private:
    friend class VoxelTreeCursor;
    friend class VoxelTreeEditor;
    friend class VoxelTreeOperation;

private:
    VoxelState m_rootState; // 根体素自身的逻辑状态。
    VoxelNodeBlock m_rootBlock; // 根体素细分后描述其八个直接子体素。
    Foundation::RefPtr<VoxelBlockPool> m_blockPool; // 当前体素树使用的共享物理节点池。
};

}

#endif // MYVOXEL_CORE_TREE_VOXELTREE_H