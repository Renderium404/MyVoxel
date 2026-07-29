#ifndef MYVOXEL_VOXELFORESTQUERY_H
#define MYVOXEL_VOXELFORESTQUERY_H

#include "VoxelRegionQuery.h"
#include "VoxelPackedForestQuery.h"

namespace MyVoxel
{

// 生产区域查询器使用Packed森林递归查询实现。
using VoxelForestQuery = VoxelPackedForestQuery;

}

#endif // MYVOXEL_VOXELFORESTQUERY_H