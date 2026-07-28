#ifndef MYVOXEL_VOXELSURFACEGENERATION_H
#define MYVOXEL_VOXELSURFACEGENERATION_H

namespace MyVoxel
{

// 指定体素表面三角网格的生成方式。
enum class VoxelSurfaceGenerationMode
{
    GreedyVoxel, // 沿体素边界生成并贪心合并轴对齐矩形，保留明显阶梯结构。
    SmoothMarchingTetrahedra // 从二值体素场提取连续等值面，并生成平滑法线。
};

// 控制体素表面三角网格的生成方式和连续曲面参数。
struct VoxelSurfaceGenerationOptions
{
    VoxelSurfaceGenerationMode mode = VoxelSurfaceGenerationMode::GreedyVoxel; // 当前网格生成方式。
    double isoValue = 0.5; // 连续等值面阈值，二值材料场默认使用0.5。
    int boundaryPadding = 1; // 连续曲面候选单元在体素边界外扩的最小体素数量。
};

}

#endif // MYVOXEL_VOXELSURFACEGENERATION_H