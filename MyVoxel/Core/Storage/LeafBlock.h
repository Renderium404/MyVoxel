#ifndef MYVOXEL_CORE_STORAGE_LEAFBLOCK_H
#define MYVOXEL_CORE_STORAGE_LEAFBLOCK_H

#include <cassert>
#include <cmath>

#include "MyVoxel/Core/VoxelTypes.h"

namespace MyVoxel
{

const unsigned int LeafBlockSampleCount = static_cast<unsigned int>(VoxelCornerCount) * static_cast<unsigned int>(VoxelCornerCount); // 一个叶块保存连续两级4×4×4共64个最高层体素中心的有符号距离。

// 保存一个逻辑叶区中64个最高层体素中心的有符号距离。
//
// 距离样本按照coarseCorner×8+fineCorner排列。
// 负值表示材料内部，正值表示材料外部，零值表示等值面。
struct LeafBlock
{
    float distances[LeafBlockSampleCount]; // 64个最高层体素中心的有符号距离。
};

static_assert(LeafBlockSampleCount == 64, "LeafBlock requires exactly 64 distance samples.");
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

// 返回指定样本索引对应的可写距离值。
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

// 返回粗层角点和细层角点对应的可写距离值。
inline float& leafDistance(LeafBlock& block, VoxelCorner coarseCorner, VoxelCorner fineCorner)
{
    return leafDistance(block, leafSampleIndex(coarseCorner, fineCorner));
}

// 返回粗层角点和细层角点对应的只读距离值。
inline float leafDistance(const LeafBlock& block, VoxelCorner coarseCorner, VoxelCorner fineCorner)
{
    return leafDistance(block, leafSampleIndex(coarseCorner, fineCorner));
}

/// 数据检查

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