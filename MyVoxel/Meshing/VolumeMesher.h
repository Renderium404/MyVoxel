#ifndef MYVOXEL_MESHING_VOLUMEMESHER_H
#define MYVOXEL_MESHING_VOLUMEMESHER_H

#include "MyVoxel/Core/VoxelGrid.h"
#include "MyVoxel/Geometry/Mesh/Mesh.h"

namespace MyVoxel
{

class VolumeFieldView;

namespace Meshing
{

class VolumeMeshingWorkspace;

// 指定距离场到网格的顶点生成方式。
enum class VolumeMeshingMode
{
    StandardSurfaceNets, // 使用各边组交点平均值生成顶点。
    FeatureSensitiveSurfaceNets // 使用各边组交点法线和QEF生成特征敏感顶点。
};

// 指定单元边组拓扑的计算方式。
enum class VolumeTopologyMode
{
    StandardDynamic, // 动态分析六个面的边连通关系，作为独立标准路径。
    FastLookup // 使用由标准路径预先生成的256项固定查找表。
};

// 指定角点位置和梯度的获取方式。
enum class VolumeSamplingMode
{
    StandardOnDemand, // 每个活动单元独立计算八个角点位置和角点梯度。
    FastCached // 根据单元起点推导角点位置，并在工作区中懒缓存采样梯度。
};

// 指定单元符号掩码的获取方式。
enum class VolumeSignMode
{
    StandardDirect, // 每个单元直接读取八个距离值并生成符号掩码。
    FastPrecomputed // 每个采样点只读取一次符号，再预计算全部单元符号掩码。
};

// 控制当前VoxelShape稀疏距离场到统一三角网格的网格化过程。
struct VolumeMeshingOptions
{
    double isoValue = 0.0; // 需要提取的距离等值面，必须位于内部和外部背景距离之间。
    Geometry::MeshColor color; // 输出三角形统一使用的颜色。
    VolumeMeshingMode mode = VolumeMeshingMode::StandardSurfaceNets; // 顶点定位模式。
    VolumeTopologyMode topologyMode = VolumeTopologyMode::FastLookup; // 默认使用快速拓扑表。
    VolumeSamplingMode samplingMode = VolumeSamplingMode::FastCached; // 默认复用角点梯度。
    VolumeSignMode signMode = VolumeSignMode::FastPrecomputed; // 默认预计算采样符号和单元符号掩码。
};

// 从VoxelShape当前有效稀疏距离场中生成统一多边组Surface Nets三角网格。
class VolumeMesher
{
public:
    // 根据全部已分配VolumeBlock推导最高层采样范围，并在六个方向扩展一个样本；空距离场返回无效范围。
    static VoxelCellRange defaultSampleRange(const VolumeFieldView& view);

    // 使用自动推导的全量采样范围生成等值面网格；非空Shape必须具有已分配距离块。
    static Geometry::Mesh build(const VolumeFieldView& view, const VolumeMeshingOptions& options = VolumeMeshingOptions());

    // 使用调用者指定的有限最高层采样范围生成等值面网格。
    static Geometry::Mesh build(const VolumeFieldView& view, const VoxelCellRange& sampleRange,
                                const VolumeMeshingOptions& options = VolumeMeshingOptions());

    // 使用调用者提供的连续工作区生成等值面网格，工作区范围决定本次网格化区域。
    static Geometry::Mesh buildWithWorkspace(const VolumeFieldView& view, VolumeMeshingWorkspace& workspace,
                                             const VolumeMeshingOptions& options = VolumeMeshingOptions());

private:
    VolumeMesher() = delete;
};

}
}

#endif // MYVOXEL_MESHING_VOLUMEMESHER_H