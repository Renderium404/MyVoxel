#ifndef MYVOXEL_CORE_STORAGE_MASKBLOCK_H
#define MYVOXEL_CORE_STORAGE_MASKBLOCK_H

#include <cassert>
#include <cstdint>

#include "MyVoxel/Core/VoxelTypes.h"

namespace MyVoxel
{

const unsigned int MaskBlockSampleCount = static_cast<unsigned int>(VoxelCornerCount) * static_cast<unsigned int>(VoxelCornerCount); // 一个掩码块保存连续两级4×4×4共64个最高层体素的材料状态。

// 使用一个64位掩码保存连续两层4×4×4共64个最高层体素的材料状态。
//
// 64位掩码按照coarseCorner×8+fineCorner排列：
//
// 位范围    粗层角点
// [0, 7]    Minimum
// [8, 15]   MaximumX
// [16, 23]  MaximumY
// [24, 31]  MaximumXY
// [32, 39]  MaximumZ
// [40, 47]  MaximumXZ
// [48, 55]  MaximumYZ
// [56, 63]  MaximumXYZ
//
// 位为1表示材料存在，位为0表示材料不存在。
struct MaskBlock
{
    std::uint64_t materialMask; // 64个最高层体素的材料状态。
};

static_assert(MaskBlockSampleCount == 64, "MaskBlock requires exactly 64 material samples.");
static_assert(sizeof(MaskBlock) == 8, "MaskBlock must be exactly 8 bytes.");

/// 掩码块初始化

// 将64个材料状态统一设置为空或材料。
inline void reset(MaskBlock& block, VoxelState state = VoxelState::Empty)
{
    assert(state == VoxelState::Empty || state == VoxelState::Material);
    block.materialMask = state == VoxelState::Material ? ~static_cast<std::uint64_t>(0) : static_cast<std::uint64_t>(0);
}

/// 样本索引

// 返回粗层角点和细层角点对应的64位样本索引。
inline unsigned int maskSampleIndex(VoxelCorner coarseCorner, VoxelCorner fineCorner)
{
    const unsigned int coarseIndex = static_cast<unsigned int>(coarseCorner);
    const unsigned int fineIndex = static_cast<unsigned int>(fineCorner);
    assert(coarseIndex < static_cast<unsigned int>(VoxelCornerCount));
    assert(fineIndex < static_cast<unsigned int>(VoxelCornerCount));
    return coarseIndex * static_cast<unsigned int>(VoxelCornerCount) + fineIndex;
}

// 返回指定样本索引对应的64位掩码位。
inline std::uint64_t maskBitValue(unsigned int sampleIndex)
{
    assert(sampleIndex < MaskBlockSampleCount);
    return static_cast<std::uint64_t>(1ULL << sampleIndex);
}

// 返回粗层角点和细层角点对应的64位掩码位。
inline std::uint64_t maskBitValue(VoxelCorner coarseCorner, VoxelCorner fineCorner)
{
    return maskBitValue(maskSampleIndex(coarseCorner, fineCorner));
}

/// 材料状态访问

// 返回指定样本索引是否包含材料。
inline bool maskBit(const MaskBlock& block, unsigned int sampleIndex)
{
    return (block.materialMask & maskBitValue(sampleIndex)) != 0;
}

// 返回粗层角点和细层角点对应的样本是否包含材料。
inline bool maskBit(const MaskBlock& block, VoxelCorner coarseCorner, VoxelCorner fineCorner)
{
    return maskBit(block, maskSampleIndex(coarseCorner, fineCorner));
}

// 设置指定样本索引对应的材料状态。
inline void setMaskBit(MaskBlock& block, unsigned int sampleIndex, bool material)
{
    const std::uint64_t bit = maskBitValue(sampleIndex);
    if (material)
    {
        block.materialMask |= bit;
    }
    else
    {
        block.materialMask &= ~bit;
    }
}

// 设置粗层角点和细层角点对应的材料状态。
inline void setMaskBit(MaskBlock& block, VoxelCorner coarseCorner, VoxelCorner fineCorner, bool material)
{
    setMaskBit(block, maskSampleIndex(coarseCorner, fineCorner), material);
}

// 返回指定粗层角点对应的八个细层样本材料掩码。
inline std::uint8_t maskBits(const MaskBlock& block, VoxelCorner coarseCorner)
{
    const unsigned int coarseIndex = static_cast<unsigned int>(coarseCorner);
    assert(coarseIndex < static_cast<unsigned int>(VoxelCornerCount));
    return static_cast<std::uint8_t>((block.materialMask >> (coarseIndex * static_cast<unsigned int>(VoxelCornerCount))) & 0xFFULL);
}

/// 掩码状态判断

// 判断64个样本是否全部为空。
inline bool isEmpty(const MaskBlock& block)
{
    return block.materialMask == static_cast<std::uint64_t>(0);
}

// 判断64个样本是否全部为材料。
inline bool isFull(const MaskBlock& block)
{
    return block.materialMask == ~static_cast<std::uint64_t>(0);
}

// 判断当前掩码是否同时包含材料和空样本。
inline bool isMixed(const MaskBlock& block)
{
    return !isEmpty(block) && !isFull(block);
}

}

#endif // MYVOXEL_CORE_STORAGE_MASKBLOCK_H