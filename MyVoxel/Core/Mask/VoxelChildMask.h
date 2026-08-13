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

// 将NodeBlock的四种物理状态归约为Empty、Material和Subdivided三种逻辑子体素状态。
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

// 返回MaskLeaf八个粗层组的逻辑结构状态。
// MaskLeaf显式保存完整4×4×4距离样本，因此八个粗层组始终存在更细数据，不能仅凭材料状态一致性提前终止。
inline VoxelChildStateMasks leafChildStateMasks(const MaskBlock&)
{
    VoxelChildStateMasks result;
    result.subdivided = FullVoxelChildMask;

    assert(result.isValid());

    return result;
}

// 返回MaskBlock指定粗层组中的八个最细体素二值材料状态。
// 最细体素已经没有更深结构，因此直接根据materialMask返回Empty或Material。
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