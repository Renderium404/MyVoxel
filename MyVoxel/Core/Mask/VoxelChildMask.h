#ifndef MYVOXEL_CORE_MASK_VOXELCHILDMASK_H
#define MYVOXEL_CORE_MASK_VOXELCHILDMASK_H

#include <cassert>
#include <cstdint>

#include "MyVoxel/Core/Mask/VoxelLeafMask.h"
#include "MyVoxel/Core/Mask/VoxelNodeMask.h"
#include "MyVoxel/Core/Tree/VoxelChildStateMasks.h"

namespace MyVoxel
{

/// 物理状态转换

// 将普通节点块的四种存储状态归约为三种逻辑子体素状态。
inline VoxelChildStateMasks nodeChildStateMasks(
    const VoxelNodeBlock& block)
{
    const VoxelNodeStateMasks storageStates =
        nodeStateMasks(block);

    VoxelChildStateMasks result;
    result.empty = storageStates.empty;
    result.material = storageStates.material;
    result.subdivided = storageStates.subdivided;

    assert(result.isValid());
    return result;
}

// 将掩码叶块的八个粗层组归约为八个逻辑子体素状态。
inline VoxelChildStateMasks leafChildStateMasks(
    const VoxelLeafBlock& block)
{
    const VoxelLeafGroupStateMasks groupStates =
        leafGroupStateMasks(block);

    VoxelChildStateMasks result;
    result.empty = groupStates.empty;
    result.material = groupStates.material;
    result.subdivided = groupStates.subdivided;

    assert(result.isValid());
    return result;
}

// 返回掩码叶块指定粗层组中的八个细层子体素状态。
inline VoxelChildStateMasks leafGroupChildStateMasks(
    const VoxelLeafBlock& block,
    VoxelCorner coarseCorner)
{
    const std::uint8_t materialMask =
        leafGroupBits(block, coarseCorner);

    VoxelChildStateMasks result;
    result.material = materialMask;
    result.empty =
        static_cast<std::uint8_t>(~materialMask);

    assert(result.isValid());
    return result;
}

}

#endif // MYVOXEL_CORE_MASK_VOXELCHILDMASK_H
