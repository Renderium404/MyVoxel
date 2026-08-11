#ifndef MYVOXEL_CORE_STORAGE_LEAFBLOCK_H
#define MYVOXEL_CORE_STORAGE_LEAFBLOCK_H

#include <cassert>
#include <cmath>

#include "MyVoxel/Core/VoxelTypes.h"
#include "MaskBlock.h"

namespace MyVoxel
{

const unsigned int LeafBlockSampleCount = static_cast<unsigned int>(VoxelCornerCount) * static_cast<unsigned int>(VoxelCornerCount); // 一个叶块保存连续两级4×4×4共64个最高层体素中心的有符号距离。

// 保存一个逻辑叶区中64个最高层体素中心的有符号距离。
//
// 距离样本按照coarseCorner×8+fineCorner排列，与MaskBlock中的材料位一一对应。
// 负值表示材料内部，正值表示材料外部，零值表示等值面。
struct LeafBlock
{
    float distances[LeafBlockSampleCount]; // 64个最高层体素中心的有符号距离。
};

static_assert(LeafBlockSampleCount == 64, "LeafBlock requires exactly 64 distance samples.");
static_assert(LeafBlockSampleCount == MaskBlockSampleCount, "LeafBlock and MaskBlock sample counts must match.");
static_assert(sizeof(LeafBlock) == sizeof(float) * LeafBlockSampleCount, "LeafBlock must contain exactly 64 floats.");

/// 叶块初始化

// 将64个距离样本统一设置为指定值。
inline void reset(LeafBlock& block, float distance = 0.0f)
{
    assert(std::isfinite(static_cast<double>(distance)));
    for (unsigned int sampleIndex = 0; sampleIndex < LeafBlockSampleCount; ++sampleIndex)
    {
        block.distances[sampleIndex] = distance;
    }
}

// 同时初始化距离叶块和对应材料掩码，距离小于等于零视为材料。
inline void reset(MaskBlock& maskBlock, LeafBlock& leafBlock, float distance)
{
    assert(std::isfinite(static_cast<double>(distance)));
    reset(leafBlock, distance);
    reset(maskBlock, distance <= 0.0f ? VoxelState::Material : VoxelState::Empty);
}

/// 样本索引

// 返回粗层角点和细层角点对应的距离样本索引。
inline unsigned int leafSampleIndex(VoxelCorner coarseCorner, VoxelCorner fineCorner)
{
    const unsigned int coarseIndex = static_cast<unsigned int>(coarseCorner);
    const unsigned int fineIndex = static_cast<unsigned int>(fineCorner);
    assert(coarseIndex < static_cast<unsigned int>(VoxelCornerCount));
    assert(fineIndex < static_cast<unsigned int>(VoxelCornerCount));
    return coarseIndex * static_cast<unsigned int>(VoxelCornerCount) + fineIndex;
}

/// 距离样本访问

// 返回指定样本索引对应的可写距离值，直接修改不会自动同步MaskBlock。
inline float& leafDistance(LeafBlock& block, unsigned int sampleIndex)
{
    assert(sampleIndex < LeafBlockSampleCount);
    return block.distances[sampleIndex];
}

// 返回指定样本索引对应的只读距离值。
inline float leafDistance(const LeafBlock& block, unsigned int sampleIndex)
{
    assert(sampleIndex < LeafBlockSampleCount);
    return block.distances[sampleIndex];
}

// 返回粗层角点和细层角点对应的可写距离值，直接修改不会自动同步MaskBlock。
inline float& leafDistance(LeafBlock& block, VoxelCorner coarseCorner, VoxelCorner fineCorner)
{
    return leafDistance(block, leafSampleIndex(coarseCorner, fineCorner));
}

// 返回粗层角点和细层角点对应的只读距离值。
inline float leafDistance(const LeafBlock& block, VoxelCorner coarseCorner, VoxelCorner fineCorner)
{
    return leafDistance(block, leafSampleIndex(coarseCorner, fineCorner));
}

/// 距离与材料联合操作

// 同时设置指定样本的有符号距离和材料状态，距离小于等于零视为材料。
inline void setLeafDistance(MaskBlock& maskBlock, LeafBlock& leafBlock, unsigned int sampleIndex, float distance)
{
    assert(sampleIndex < LeafBlockSampleCount);
    assert(std::isfinite(static_cast<double>(distance)));
    leafBlock.distances[sampleIndex] = distance;
    setMaskBit(maskBlock, sampleIndex, distance <= 0.0f);
}

// 同时设置粗层角点和细层角点对应样本的有符号距离和材料状态。
inline void setLeafDistance(MaskBlock& maskBlock, LeafBlock& leafBlock, VoxelCorner coarseCorner, VoxelCorner fineCorner, float distance)
{
    setLeafDistance(maskBlock, leafBlock, leafSampleIndex(coarseCorner, fineCorner), distance);
}

// 根据当前64个距离样本重新生成完整材料掩码。
inline void rebuildMask(const LeafBlock& leafBlock, MaskBlock& maskBlock)
{
    maskBlock.materialMask = 0;
    for (unsigned int sampleIndex = 0; sampleIndex < LeafBlockSampleCount; ++sampleIndex)
    {
        if (leafBlock.distances[sampleIndex] <= 0.0f)
        {
            maskBlock.materialMask |= maskBitValue(sampleIndex);
        }
    }
}

// 检查材料掩码是否与全部距离样本的符号一致。
inline bool isMaskConsistent(const MaskBlock& maskBlock, const LeafBlock& leafBlock)
{
    for (unsigned int sampleIndex = 0; sampleIndex < LeafBlockSampleCount; ++sampleIndex)
    {
        const float distance = leafBlock.distances[sampleIndex];
        if (!std::isfinite(static_cast<double>(distance)) || maskBit(maskBlock, sampleIndex) != (distance <= 0.0f))
        {
            return false;
        }
    }
    return true;
}

// 检查64个距离样本是否全部为有限数值。
inline bool isValid(const LeafBlock& block)
{
    for (unsigned int sampleIndex = 0; sampleIndex < LeafBlockSampleCount; ++sampleIndex)
    {
        if (!std::isfinite(static_cast<double>(block.distances[sampleIndex])))
        {
            return false;
        }
    }
    return true;
}

}

#endif // MYVOXEL_CORE_STORAGE_LEAFBLOCK_H