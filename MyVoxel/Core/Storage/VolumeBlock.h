#ifndef MYVOXEL_CORE_STORAGE_VOLUMEBLOCK_H
#define MYVOXEL_CORE_STORAGE_VOLUMEBLOCK_H

#include <cassert>

#include "VoxelBlock.h"

namespace MyVoxel
{

const unsigned int VolumeBlockSampleCount = static_cast<unsigned int>(VoxelCornerCount) * static_cast<unsigned int>(VoxelCornerCount); // 一个距离块保存4×4×4个细层体素中心的64个距离样本。

// 保存一个逻辑掩码叶区中64个细层体素中心的有符号距离。
//
// 距离样本与VoxelLeafBlock材料位使用相同顺序：
// sampleIndex = coarseCorner×8+fineCorner。
// 负值表示材料内部，正值表示材料外部，零值表示等值面。
struct VolumeBlock
{
    float distances[VolumeBlockSampleCount]; // 与VoxelLeafBlock材料位逐项对应的64个有符号距离。
};

static_assert(VolumeBlockSampleCount == 64, "VolumeBlock requires exactly 64 distance samples.");
static_assert(sizeof(VolumeBlock) == sizeof(float) * VolumeBlockSampleCount, "VolumeBlock must contain exactly 64 floats.");

/// 距离块初始化

// 将距离块中的全部64个距离样本设置为指定值。
inline void reset(VolumeBlock& block, float distance = 0.0f)
{
    for (unsigned int sampleIndex = 0; sampleIndex < VolumeBlockSampleCount; ++sampleIndex)
    {
        block.distances[sampleIndex] = distance;
    }
}

/// 距离样本索引

// 返回粗层角点和细层角点对应的距离样本索引。
inline unsigned int volumeSampleIndex(VoxelCorner coarseCorner, VoxelCorner fineCorner)
{
    const unsigned int fineIndex = static_cast<unsigned int>(fineCorner);

    assert(fineIndex < static_cast<unsigned int>(VoxelCornerCount));
    return leafMaskOffset(coarseCorner) + fineIndex;
}

/// 距离样本访问

// 返回指定样本索引对应的可写距离值。
inline float& volumeDistance(VolumeBlock& block, unsigned int sampleIndex)
{
    assert(sampleIndex < VolumeBlockSampleCount);
    return block.distances[sampleIndex];
}

// 返回指定样本索引对应的只读距离值。
inline float volumeDistance(const VolumeBlock& block, unsigned int sampleIndex)
{
    assert(sampleIndex < VolumeBlockSampleCount);
    return block.distances[sampleIndex];
}

// 返回指定粗层角点和细层角点对应的可写距离值。
inline float& volumeDistance(VolumeBlock& block, VoxelCorner coarseCorner, VoxelCorner fineCorner)
{
    return volumeDistance(block, volumeSampleIndex(coarseCorner, fineCorner));
}

// 返回指定粗层角点和细层角点对应的只读距离值。
inline float volumeDistance(const VolumeBlock& block, VoxelCorner coarseCorner, VoxelCorner fineCorner)
{
    return volumeDistance(block, volumeSampleIndex(coarseCorner, fineCorner));
}

}

#endif // MYVOXEL_CORE_STORAGE_VOLUMEBLOCK_H