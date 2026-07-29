#ifndef MYVOXEL_CORE_TREE_VOXELTREE_H
#define MYVOXEL_CORE_TREE_VOXELTREE_H

#include <cassert>
#include <cstdint>

#include "../Storage/VoxelBlock.h"
#include "../Storage/VoxelBlockPool.h"
#include "../../Foundation/RefPtr.h"

namespace MyVoxel
{

// 表示体素树中一个逻辑体素的状态。
enum class VoxelTreeState : std::uint8_t
{
    Empty = 0, // 当前逻辑体素为空。
    Material = 1, // 当前逻辑体素完全包含材料。
    Subdivided = 2 // 当前逻辑体素已经细分。
};

using VoxelRootState = VoxelTreeState;

// 保存一个根体素、根节点描述块及其物理节点池。
struct VoxelTree
{
    // 创建完全为空或完全包含材料的体素树。
    explicit VoxelTree(VoxelRootState state = VoxelRootState::Empty)
        : rootState(state)
        , blockPool(Foundation::makeRef<VoxelBlockPool>())
    {
        assert(state == VoxelRootState::Empty || state == VoxelRootState::Material);
        reset(rootBlock, state == VoxelRootState::Material ? VoxelState::Material : VoxelState::Empty);
    }

    // 检查根状态、根节点块和物理节点池是否满足基本结构约束。
    bool isValid() const
    {
        if (!blockPool)
        {
            return false;
        }

        if (rootState == VoxelRootState::Empty)
        {
            return rootBlock.storageMask == 0 &&
                   rootBlock.leafMask == 0 &&
                   rootBlock.firstChildIndex == InvalidVoxelIndex &&
                   blockPool->allocatedGroupCount() == 0;
        }

        if (rootState == VoxelRootState::Material)
        {
            return rootBlock.storageMask == 0 &&
                   rootBlock.leafMask == static_cast<std::uint8_t>(0xFFU) &&
                   rootBlock.firstChildIndex == InvalidVoxelIndex &&
                   blockPool->allocatedGroupCount() == 0;
        }

        if (rootState != VoxelRootState::Subdivided)
        {
            return false;
        }

        if (rootBlock.storageMask == 0)
        {
            if (rootBlock.leafMask == static_cast<std::uint8_t>(0) ||
                rootBlock.leafMask == static_cast<std::uint8_t>(0xFFU))
            {
                return false;
            }

            return rootBlock.firstChildIndex == InvalidVoxelIndex &&
                blockPool->allocatedGroupCount() == 0;
        }


    }
    VoxelRootState rootState; // 根体素自身的状态。
    VoxelNodeBlock rootBlock; // 根体素细分后，其八个直接子体素的描述块。
    Foundation::RefPtr<VoxelBlockPool> blockPool; // 当前体素树使用的物理节点池。
};

}

#endif // MYVOXEL_CORE_TREE_VOXELTREE_H