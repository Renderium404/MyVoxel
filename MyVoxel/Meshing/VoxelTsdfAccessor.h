#ifndef MYVOXEL_MESHING_VOXELTSDFACCESSOR_H
#define MYVOXEL_MESHING_VOXELTSDFACCESSOR_H

#include "MyMath/Vector3.h"
#include "MyVoxel/Core/VoxelAddress.h"

namespace MyVoxel
{

class VoxelGrid;
class VoxelShape;

namespace Meshing
{

// 提供VoxelShape最高层Cell-Centered TSDF的统一只读采样入口，供Surface Nets及后续梯度计算直接使用。
//
// 采样索引始终位于VoxelGrid最高层；位置返回VoxelShape局部体素空间坐标，不应用VoxelShape实例Transform。
// 类不拥有VoxelShape，调用方必须保证被引用Shape在Accessor使用期间保持存活且不被修改。
class VoxelTsdfAccessor
{
public:
    // 绑定指定有效VoxelShape。
    explicit VoxelTsdfAccessor(const VoxelShape& shape);

    /// 状态与绑定对象

    // 判断当前Accessor是否绑定有效VoxelShape。
    bool isValid() const;
    // 返回当前绑定的VoxelShape。
    const VoxelShape& shape() const;
    // 返回当前Shape使用的VoxelGrid。
    const VoxelGrid& grid() const;
    // 返回TSDF样本所在的最高体素层级。
    VoxelLevel sampleLevel() const;
    // 返回相邻最高层TSDF样本中心之间的固定间距。
    double sampleSpacing() const;
    // 返回当前TSDF固定截断背景距离B。
    float backgroundDistance() const;
    // 判断指定地址是否对应当前最高层TSDF样本。
    bool supportsSampleAddress(const VoxelCellAddress& address) const;

    /// TSDF采样

    // 返回指定最高层体素中心的TSDF值，缺失Root由VoxelShape直接解析为+B。
    float value(const VoxelCellAddress& address) const;
    // 返回指定最高层体素索引对应中心的TSDF值。
    float value(const VoxelCellIndex& index) const;
    // 返回指定最高层体素中心在Shape局部体素空间中的位置。
    MyMath::Vector3 samplePosition(const VoxelCellAddress& address) const;
    // 返回指定最高层体素索引对应中心在Shape局部体素空间中的位置。
    MyMath::Vector3 samplePosition(const VoxelCellIndex& index) const;

    /// 一阶微分

    // 使用六邻域二阶中心差分返回指定最高层样本处的局部空间TSDF梯度，结果包含1/(2h)尺度。
    MyMath::Vector3 gradient(const VoxelCellIndex& index) const;

private:
    const VoxelShape* m_shape; // 当前非拥有绑定的VoxelShape。
};

}
}

#endif // MYVOXEL_MESHING_VOXELTSDFACCESSOR_H