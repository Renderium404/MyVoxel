#ifndef MYVOXEL_MESHING_VOLUMEMESHER_H
#define MYVOXEL_MESHING_VOLUMEMESHER_H

#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Core/VoxelShape.h"
#include "MyVoxel/Display/Base/Display_Color.h"
#include "MyVoxel/Mesh/Mesh.h"

namespace MyVoxel
{
namespace Meshing
{

class VolumeMeshingWorkspace;

// 指定TSDF到网格的顶点生成方式。
enum class VolumeMeshingMode
{
    StandardSurfaceNets, // 使用同一边组全部零交叉点的平均位置生成顶点。
    FeatureSensitiveSurfaceNets, // 平滑区域保持平均位置，明显特征区域使用零交叉法线QEF恢复棱角。
    FeatureConstrainedSurfaceNets // 优先使用VoxelShape显式Feature约束顶点，无可用显式Feature时退回FeatureSensitive路径。
};

// 指定单元边组拓扑的计算方式。
enum class VolumeTopologyMode
{
    StandardDynamic, // 动态分析六个面的边连通关系，作为独立标准路径。
    FastLookup // 使用由标准路径预先生成的256项固定查找表。
};

// 指定角点位置的获取方式。
enum class VolumeSamplingMode
{
    StandardOnDemand, // 每个活动单元分别读取八个TSDF样本位置。
    FastCached // 从单元第0角点和固定最高层间距直接推导其余七个位置。
};

// 指定单元符号掩码的获取方式。
enum class VolumeSignMode
{
    StandardDirect, // 每个单元直接读取八个TSDF值并生成符号掩码。
    FastPrecomputed // 每个采样点只判定一次符号，再预计算全部单元符号掩码。
};

// 控制VoxelShape最高层Cell-Centered TSDF到统一三角网格的Surface Nets提取。
struct VolumeMeshingOptions
{
    double isoValue = 0.0; // 需要提取的TSDF等值面，必须严格位于(-B,+B)。
    Display_Color color; // 输出全部三角形统一使用的显示颜色。
    VolumeMeshingMode mode = VolumeMeshingMode::StandardSurfaceNets; // 默认保持已经验证的Uniform Surface Nets。
    VolumeTopologyMode topologyMode = VolumeTopologyMode::FastLookup; // 默认使用256项拓扑查找表。
    VolumeSamplingMode samplingMode = VolumeSamplingMode::FastCached; // 默认从单元起点快速推导角点位置。
    VolumeSignMode signMode = VolumeSignMode::FastPrecomputed; // 默认预计算采样符号和单元符号掩码。
    double featureAngleDegrees = 30.0; // Feature模式下零交叉法线最大夹角达到30度才启用QEF，平滑区域继续使用平均顶点。
    double featureSnapDistanceScale = 0.75; // 显式Feature约束允许的最大候选距离，相对于最高层TSDF采样间距h。
};

// 直接从VoxelShape当前最高层TSDF提取OpenVDB风格多边组Surface Nets三角网格。
class VolumeMesher
{
public:
    // 根据全部显式Root展开到最高层后的索引范围并增加一层采样Halo；空Shape返回无效范围。
    static VoxelCellRange defaultSampleRange(const VoxelShape& shape);

    // 使用自动采样范围和内部临时工作区提取TSDF等值面。
    static Mesh build(const VoxelShape& shape, const VolumeMeshingOptions& options = VolumeMeshingOptions());

    // 使用指定最高层有限采样范围和内部临时工作区提取TSDF等值面。
    static Mesh build(const VoxelShape& shape, const VoxelCellRange& sampleRange,
                      const VolumeMeshingOptions& options = VolumeMeshingOptions());

    // 使用调用者提供的连续工作区提取TSDF等值面，工作区采样范围必须属于Shape最高层。
    static Mesh buildWithWorkspace(const VoxelShape& shape, VolumeMeshingWorkspace& workspace,
                                   const VolumeMeshingOptions& options = VolumeMeshingOptions());

private:
    VolumeMesher() = delete;
};

}
}

#endif // MYVOXEL_MESHING_VOLUMEMESHER_H