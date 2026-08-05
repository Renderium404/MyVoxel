#ifndef MYVOXEL_CONVERSION_MESHTOVOLUME_H
#define MYVOXEL_CONVERSION_MESHTOVOLUME_H

#include "MyVoxel/Geometry/Mesh/Mesh.h"
#include "MyVoxel/Volume/LevelSetVolume.h"

namespace MyVoxel
{
namespace Conversion
{

// 控制三角网格到有符号距离场的标准转换。
struct MeshToVolumeOptions
{
    double exteriorBandWidth = 3.0; // 外部窄带宽度，单位为最高层体素边长。
    double interiorBandWidth = 3.0; // 内部窄带宽度，单位为最高层体素边长。
    bool unsignedDistance = false; // 是否仅生成非负无符号距离。
};

// 将三角网格转换为最高层体素中心采样的有限距离场。
class MeshToVolume
{
public:
    // 使用指定体素网格生成距离场。
    static LevelSetVolume convert(const Geometry::Mesh& mesh,
                                  const VoxelGrid& grid,
                                  const MeshToVolumeOptions& options = MeshToVolumeOptions());

private:
    MeshToVolume() = delete;
};

}
}

#endif // MYVOXEL_CONVERSION_MESHTOVOLUME_H