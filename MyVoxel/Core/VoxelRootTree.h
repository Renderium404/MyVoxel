#ifndef MYVOXEL_VOXELROOTTREE_H
#define MYVOXEL_VOXELROOTTREE_H

#include "VoxelNode.h"
#include "VoxelNodePool.h"

namespace MyVoxel
{

// 保存一个第0层根节点及其独立拥有的全部下级节点组。
struct VoxelRootTree
{
    explicit VoxelRootTree(VoxelState state = VoxelState::Empty)
        : root(state)
    {
    }

    VoxelNode root; // 当前第0层根节点。
    VoxelNodePool nodePool; // 当前根节点独立拥有的下级节点池。
};

}

#endif // MYVOXEL_VOXELROOTTREE_H