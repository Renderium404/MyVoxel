#ifndef MYVOXEL_CORE_TREE_VOXELTREE_H
#define MYVOXEL_CORE_TREE_VOXELTREE_H

#include "MyVoxel/Core/Storage/VoxelNode.h"
#include "MyVoxel/Core/Storage/VoxelNodePool.h"
#include "MyVoxel/Foundation/ReferenceCounted.h"

namespace MyVoxel
{

// 保存一个第0层根节点及其独立拥有的全部下级节点组。
class VoxelTree : public Foundation::ReferenceCounted
{
public:
    // 使用指定叶节点状态创建独立根树。
    explicit VoxelTree(VoxelState state = VoxelState::Empty);

    // 深度复制根节点及其全部下级节点组，不复制引用计数。
    VoxelTree(const VoxelTree& other);

    VoxelTree& operator=(const VoxelTree&) = delete;

    VoxelNode root;         // 当前第0层根节点。
    VoxelNodePool nodePool; // 当前根节点独立拥有的下级节点池。

protected:
    // 通过侵入式引用计数管理根树生命周期。
    ~VoxelTree() override = default;
};

}

#endif // MYVOXEL_CORE_TREE_VOXELTREE_H