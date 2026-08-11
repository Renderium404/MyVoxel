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

// 将NodeBlock的四种物理状态归约为三种逻辑子体素状态。
inline VoxelChildStateMasks nodeChildStateMasks(const NodeBlock& block)
{
    const VoxelNodeStateMasks storageStates = nodeStateMasks(block);

    VoxelChildStateMasks result;
    result.empty = storageStates.empty;
    result.material = storageStates.material;
    result.subdivided = storageStates.subdivided;

    assert(result.isValid());
    return result;
}

// 将MaskBlock的八个粗层组归约为八个逻辑子体素状态。
inline VoxelChildStateMasks leafChildStateMasks(const MaskBlock& block)
{
    const VoxelLeafGroupStateMasks groupStates = leafGroupStateMasks(block);

    VoxelChildStateMasks result;
    result.empty = groupStates.empty;
    result.material = groupStates.material;
    result.subdivided = groupStates.subdivided;

    assert(result.isValid());
    return result;
}

// 返回MaskBlock指定粗层组中的八个细层子体素状态。
inline VoxelChildStateMasks leafGroupChildStateMasks(const MaskBlock& block, VoxelCorner coarseCorner)
{
    const std::uint8_t materialMask = leafGroupBits(block, coarseCorner);

    VoxelChildStateMasks result;
    result.material = materialMask;
    result.empty = static_cast<std::uint8_t>(~materialMask);

    assert(result.isValid());
    return result;
}

}

#endif // MYVOXEL_CORE_MASK_VOXELCHILDMASK_H