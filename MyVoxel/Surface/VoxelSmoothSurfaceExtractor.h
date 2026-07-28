#ifndef MYVOXEL_VOXELSMOOTHSURFACEEXTRACTOR_H
#define MYVOXEL_VOXELSMOOTHSURFACEEXTRACTOR_H

#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Surface/VoxelSurfaceGeneration.h"
#include "MyVoxel/Surface/VoxelSurfaceMesh.h"

namespace MyVoxel
{

// 使用连续等值面算法从体素形体中提取光滑预览网格。
class VoxelSmoothSurfaceExtractor
{
public:
    // 使用Marching Tetrahedra提取连续表面，并根据材料场梯度生成平滑法线。
    static VoxelSurfaceMesh extract(const VoxelShape& shape, const VoxelSurfaceColor& color, const VoxelSurfaceGenerationOptions& options);

private:
    VoxelSmoothSurfaceExtractor() = delete;
};

}

#endif // MYVOXEL_VOXELSMOOTHSURFACEEXTRACTOR_H