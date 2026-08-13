#ifndef MYVOXEL_MODELING_FEATURE_SHAPEFEATUREEXTRACTOR_H
#define MYVOXEL_MODELING_FEATURE_SHAPEFEATUREEXTRACTOR_H

#include "MyVoxel/Core/Feature/VoxelFeatureSet.h"
#include "MyVoxel/Topology/Shape/Topology_Shape.h"

namespace MyVoxel
{

class ShapeQuery;

namespace Modeling
{

// 从连续Shape提取需要在VoxelShape零等值面上显式保留的拓扑特征。
//
// Topology_Shape入口返回Shape局部空间特征；ShapeQuery入口返回查询坐标系特征。
// 当前支持Box、Sphere、Cylinder和ConeFrustum；查询空间提取要求ShapeQuery提供精确signed distance，
// 从而保证局部到查询变换为刚体或统一缩放并保持当前解析Feature曲线类型不变。
class ShapeFeatureExtractor
{
public:
    /// 能力查询

    // 判断当前Topology_Shape是否具有已经实现且语义明确的局部特征提取规则。
    static bool supports(const Topology_Shape& shape);
    // 判断当前ShapeQuery是否能够在查询坐标系中保持解析特征类型并执行特征提取。
    static bool supports(const ShapeQuery& query);

    /// 特征提取

    // 提取Topology_Shape自身局部几何空间中的显式特征点和特征边。
    static VoxelFeatureSet extract(const Topology_Shape& shape);
    // 提取ShapeQuery查询坐标系中的显式特征点和特征边，输出可直接与该查询空间中的TSDF对应。
    static VoxelFeatureSet extract(const ShapeQuery& query);
};

}
}

#endif // MYVOXEL_MODELING_FEATURE_SHAPEFEATUREEXTRACTOR_H
