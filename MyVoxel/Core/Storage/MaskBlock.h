#ifndef MYVOXEL_CORE_STORAGE_MASKBLOCK_H
#define MYVOXEL_CORE_STORAGE_MASKBLOCK_H

#include <cassert>
#include <cstdint>

#include "MyVoxel/Core/VoxelTypes.h"

namespace MyVoxel
{

const unsigned int MaskBlockSampleCount = static_cast<unsigned int>(VoxelCornerCount) * static_cast<unsigned int>(VoxelCornerCount); // 一个掩码块保存连续两级4×4×4共64个最高层体素的材料状态。

// 使用一个64位掩码保存连续两层4×4×4共64个最高层体素的材料二值状态。
//
// 64位掩码按照coarseCorner×8+fineCorner排列，位为1表示材料存在，位为0表示材料不存在。
struct MaskBlock
{
    std::uint64_t materialMask; // 64个最高层体素的材料二值状态。
};

static_assert(MaskBlockSampleCount == 64, "MaskBlock requires exactly 64 material samples.");
static_assert(sizeof(MaskBlock) == 8, "MaskBlock must be exactly 8 bytes.");

// 将64个材料状态统一设置为空或材料。
inline void reset(MaskBlock& block, VoxelState state = VoxelState::Empty)
{
    assert(state == VoxelState::Empty || state == VoxelState::Material);
    block.materialMask = state == VoxelState::Material ? ~static_cast<std::uint64_t>(0) : static_cast<std::uint64_t>(0);
}

}

#endif // MYVOXEL_CORE_STORAGE_MASKBLOCK_H