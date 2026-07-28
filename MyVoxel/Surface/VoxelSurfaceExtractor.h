#ifndef MYVOXEL_VOXELSURFACEEXTRACTOR_H
#define MYVOXEL_VOXELSURFACEEXTRACTOR_H

#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Surface/VoxelSurfaceGeneration.h"
#include "MyVoxel/Surface/VoxelSurfaceMesh.h"

namespace MyVoxel
{

// 根据指定生成策略从体素形体中提取外表面三角网格。
class VoxelSurfaceExtractor
{
public:
    /// 完整表面

    // 使用贪心网格算法提取体素外表面，兼容原有默认行为。
    static VoxelSurfaceMesh extract(const VoxelShape& shape, const VoxelSurfaceColor& color);

    // 使用指定生成模式提取体素外表面。
    static VoxelSurfaceMesh extract(const VoxelShape& shape, const VoxelSurfaceColor& color, VoxelSurfaceGenerationMode mode);

    // 使用完整生成选项提取体素外表面。
    static VoxelSurfaceMesh extract(const VoxelShape& shape, const VoxelSurfaceColor& color, const VoxelSurfaceGenerationOptions& options);

    /// 根级表面

    // 使用贪心网格算法提取指定第0层根节点内材料产生的外表面。
    static VoxelSurfaceMesh extractRoot(const VoxelShape& shape, const VoxelCellIndex& rootIndex, const VoxelSurfaceColor& color);

private:
    VoxelSurfaceExtractor() = delete;
};

}

#endif // MYVOXEL_VOXELSURFACEEXTRACTOR_H