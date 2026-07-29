#ifndef MYVOXEL_SURFACE_VOXELSURFACEEXTRACTOR_H
#define MYVOXEL_SURFACE_VOXELSURFACEEXTRACTOR_H

#include "MyVoxel/Core/VoxelAddress.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Surface/VoxelSurfaceMesh.h"

namespace MyVoxel
{

// 使用轴对齐贪心合并算法从VoxelShape中提取局部空间外表面网格。
class VoxelSurfaceExtractor
{
public:
    /// 完整表面

    // 提取当前体素形体的完整外表面。
    static VoxelSurfaceMesh extract(const VoxelShape& shape, const VoxelSurfaceColor& color);

    /// 根级表面

    // 仅提取指定第0层根节点内材料产生的外表面，用于后续根级显示缓存更新。
    static VoxelSurfaceMesh extractRoot(const VoxelShape& shape, const VoxelCellIndex& rootIndex, const VoxelSurfaceColor& color);

private:
    VoxelSurfaceExtractor() = delete;
};

}

#endif // MYVOXEL_SURFACE_VOXELSURFACEEXTRACTOR_H