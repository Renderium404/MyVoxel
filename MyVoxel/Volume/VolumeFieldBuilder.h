#ifndef MYVOXEL_VOLUME_VOLUMEFIELDBUILDER_H
#define MYVOXEL_VOLUME_VOLUMEFIELDBUILDER_H

#include <cstddef>

namespace MyVoxel
{

class VoxelShape;

// 控制VoxelShape全量窄带距离场构建。
struct VolumeFieldBuildOptions
{
    double exteriorBandWidth = 3.0; // 材料外部窄带宽度，单位为最高层体素边长。
    double interiorBandWidth = 3.0; // 材料内部窄带宽度，单位为最高层体素边长。
};

// 保存一次全量距离场构建的基础统计。
struct VolumeFieldBuildStatistics
{
    VolumeFieldBuildStatistics();

    // 清空全部统计。
    void clear();

    std::size_t surfaceTriangleCount; // 体素边界网格的三角形数量。
    std::size_t candidateBlockCount; // 根据表面三角形窄带包围盒收集的候选块数量。
    std::size_t distanceTestedBlockCount; // 通过块中心距离执行窄带测试的块数量。
    std::size_t allocatedBlockCount; // 最终实际保存的窄带距离块数量。
    std::size_t sampleQueryCount; // 实际执行距离计算的最高层样本数量。
};

// 从VoxelShape当前材料状态全量构建与最高层对齐的稀疏窄带有符号距离场。
//
// 第一阶段标准路径先从体素材料提取封闭边界网格，再使用MeshQuery计算样本到
// 体素边界的欧氏距离；距离符号始终由VoxelShape最高层材料状态决定。构建过程
// 先写入临时VolumeField，全部成功后再与Shape资源交换，异常不会提交半成品。
class VolumeFieldBuilder
{
public:
    // 全量重建Shape持有的距离场，Shape最高层级必须支持64样本VolumeBlock。
    static void build(VoxelShape& shape,
                      const VolumeFieldBuildOptions& options = VolumeFieldBuildOptions(),
                      VolumeFieldBuildStatistics* statistics = nullptr);

private:
    VolumeFieldBuilder() = delete;
};

}

#endif // MYVOXEL_VOLUME_VOLUMEFIELDBUILDER_H