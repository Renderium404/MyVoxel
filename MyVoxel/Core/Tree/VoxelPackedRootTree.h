#ifndef MYVOXEL_VOXELPACKEDROOTTREE_H
#define MYVOXEL_VOXELPACKEDROOTTREE_H

#include <cassert>

#include "../Storage/VoxelBlockPool.h"
#include "../Storage/VoxelNodeBlock.h"

namespace MyVoxel
{

// 保存一个第0层根节点、根节点直接子单元描述块及其独立物理节点池。
struct VoxelPackedRootTree
{
    explicit VoxelPackedRootTree(VoxelState state = VoxelState::Empty)
        : rootState(state)
    {
        assert(state == VoxelState::Empty || state == VoxelState::Material);
        rootBlock.reset(state);
    }

    // 检查根状态、根节点块和物理节点池是否满足基本结构约束。
    bool isValid() const
    {
        if (rootState == VoxelState::Empty || rootState == VoxelState::Material)
        {
            return rootBlock.childMask == 0 &&
                   rootBlock.firstChildIndex == InvalidVoxelNodeIndex &&
                   blockPool.allocatedGroupCount() == 0;
        }

        return rootState == VoxelState::Subdivided && rootBlock.isValid();
    }

    VoxelState rootState; // 第0层根节点自身状态。
    VoxelNodeBlock rootBlock; // 根节点处于Subdivided时描述其八个直接子单元。
    VoxelBlockPool blockPool; // 当前根节点独立拥有的全部物理节点。
};

}

#endif // MYVOXEL_VOXELPACKEDROOTTREE_H