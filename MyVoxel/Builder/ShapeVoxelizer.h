#ifndef MYVOXEL_SHAPEVOXELIZER_H
#define MYVOXEL_SHAPEVOXELIZER_H

#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Core/VoxelTypes.h"
#include "MyVoxel/Shape/Shape.h"

namespace MyVoxel
{

// 指定连续Shape体素化时使用的离散参数。
struct VoxelizationParameters
{
    double baseVoxelEdgeLength = 1.0; // 第0层体素边长。
    VoxelLevel maximumLevel = BaseVoxelLevel; // 体素化使用的最高细分层级。
};

// 将连续Shape按指定参数离散为VoxelShape。
VoxelShape voxelize(const Shape& shape, const VoxelizationParameters& parameters);

// 将连续Shape按指定第0层边长和最高层级离散为VoxelShape。
VoxelShape voxelize(const Shape& shape, double baseVoxelEdgeLength, VoxelLevel maximumLevel);

}

#endif // MYVOXEL_SHAPEVOXELIZER_H