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

// 保存一个第0层根体素的稀疏结构、根Tile Value以及共享完整物理块池。
//
// 根为Empty或Material时，m_rootBlock.nodeData.value保存当前根Tile Value。
// 根为Subdivided时，m_rootBlock保存八个直接子项状态和完整八槽子表首索引。
// VoxelTree不持有backgroundDistance，具体TSDF截断范围由VoxelForest或更高层场对象管理。
class VoxelTree
{
public:
    // 创建终止根树，兼容旧独立构造时Empty使用+1、Material使用-1。
    explicit VoxelTree(VoxelState state = VoxelState::Empty);
    // 使用指定终止Value创建根树，Value符号必须与Empty或Material状态一致。
    VoxelTree(VoxelState state, float value);
    // 复制体素树并共享完整物理块池。
    VoxelTree(const VoxelTree& other) = default;
    // 移动体素树并转移完整物理块池所有权。
    VoxelTree(VoxelTree&& other);

    // 复制体素树并共享完整物理块池。
    VoxelTree& operator=(const VoxelTree& other) = default;
    // 移动体素树并转移完整物理块池所有权。
    VoxelTree& operator=(VoxelTree&& other);

    /// 根节点状态

    // 返回根体素当前逻辑结构状态。
    VoxelState state() const;
    // 返回终止根体素保存的Tile Value，当前根必须为Empty或Material。
    float value() const;

    /// 树访问入口

    // 创建指向根体素的只读游标。
    VoxelTreeCursor cursor() const;
    // 创建可写编辑器，必要时分离共享BlockPool。
    VoxelTreeEditor editor();
    // 使用固定截断背景距离创建可写场编辑器，必要时分离共享BlockPool。
    VoxelTreeEditor editor(float backgroundDistance);

    /// 资源管理

    // 将体素树重置为兼容旧语义的±1终止根，并回卷全部节点和叶数据但保留Chunk。
    void reset(VoxelState state = VoxelState::Empty);
    // 将体素树重置为指定终止Value并回卷全部节点和叶数据但保留Chunk。
    void reset(VoxelState state, float value);
    // 将体素树重置为兼容旧语义的±1终止根，并释放当前完整BlockPool。
    void release(VoxelState state = VoxelState::Empty);
    // 将体素树重置为指定终止Value并释放当前完整BlockPool。
    void release(VoxelState state, float value);

    /// 节点存储统计

    std::size_t allocatedGroupCount() const;
    std::size_t highWaterGroupCount() const;
    std::size_t nodeChunkCount() const;
    std::size_t nodeStorageCapacityBytes() const;

    /// 叶存储统计

    std::size_t allocatedLeafCount() const;
    std::size_t highWaterLeafCount() const;
    std::size_t leafChunkCount() const;
    std::size_t leafStorageCapacityBytes() const;

    /// 总存储统计

    // 返回当前BlockPool节点和叶数据总容量。
    std::size_t storageCapacityBytes() const;

    /// 结构检查

    // 检查根状态、全部可达Node、Tile Value、MaskLeaf、Mask/Distance一致性和池分配数量是否一致。
    bool isValid() const;
    // 检查全部Tile Value和显式距离是否同时满足结构有效性和指定[-B,+B]截断范围。
    bool isTsdfValid(float backgroundDistance) const;
    // 检查树是否已经消除全部可无损折叠的Node和MaskLeaf。
    bool isNormalized(float backgroundDistance) const;

    /// TSDF裁剪

    // 在不改变距离场的前提下bottom-up折叠全部统一MaskLeaf和统一Node，返回结构是否发生变化。
    bool prune(float backgroundDistance);

private:
    // 必要时深复制全部可达Node、Tile Value和MaskLeaf，使本树独占完整BlockPool。
    void ensureUniqueStorage();

private:
    friend class VoxelTreeCursor;
    friend class VoxelTreeEditor;
    friend class VoxelTreeOperation;

private:
    VoxelState m_rootState; // 根体素自身的逻辑结构状态。
    NodeBlock m_rootBlock; // 终止根保存Tile Value，分化根保存八个直接子项和子表首索引。
    Foundation::RefPtr<BlockPool> m_blockPool; // 当前体素树共享的完整物理块池。
};

}

#endif // MYVOXEL_CORE_TREE_VOXELTREE_H