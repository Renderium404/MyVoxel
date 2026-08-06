#ifndef MYVOXEL_VOLUME_VOLUMEFIELDVIEW_H
#define MYVOXEL_VOLUME_VOLUMEFIELDVIEW_H

#include "MyMath/Vector3.h"
#include "MyVoxel/Core/VoxelAddress.h"

namespace MyVoxel
{

class VolumeField;
class VoxelGrid;
class VoxelShape;

// 提供VoxelShape稀疏距离场的统一只读采样视图。
//
// 已分配距离块直接返回其64个真实距离样本；未分配且有效的窄带外区域根据
// VoxelShape材料状态返回内部或外部背景距离。完全失效或目标块局部失效时禁止读取。
// 视图不拥有VoxelShape，调用方必须保证被引用Shape在视图使用期间保持存活。
class VolumeFieldView
{
public:
    // 绑定指定VoxelShape，Shape必须支持VolumeField资源。
    explicit VolumeFieldView(const VoxelShape& shape);

    /// 状态判断

    // 检查Shape、VolumeField及其采样层级是否一致。
    bool isValid() const;
    // 判断当前距离场不存在完全失效或局部失效区域。
    bool isCurrent() const;
    // 判断指定最高层采样地址当前是否允许读取。
    bool isSampleReadable(const VoxelCellAddress& sampleAddress) const;
    // 判断指定最高层采样地址是否由实际VolumeBlock保存。
    bool hasStoredSample(const VoxelCellAddress& sampleAddress) const;

    /// 绑定资源

    // 返回当前视图绑定的VoxelShape。
    const VoxelShape& shape() const;
    // 返回当前Shape使用的体素网格。
    const VoxelGrid& grid() const;
    // 返回当前Shape持有的距离场资源。
    const VolumeField& field() const;
    // 返回距离样本所在的最高体素层级。
    VoxelLevel sampleLevel() const;
    // 判断指定地址是否使用当前距离场采样层级。
    bool supportsSampleAddress(const VoxelCellAddress& sampleAddress) const;

    /// 距离采样

    // 返回指定最高层体素中心的有符号距离，目标块必须有效。
    float value(const VoxelCellAddress& sampleAddress) const;
    // 返回指定最高层体素索引对应中心的有符号距离，目标块必须有效。
    float value(const VoxelCellIndex& sampleIndex) const;
    // 返回指定最高层体素中心的局部空间位置。
    MyMath::Vector3 samplePosition(const VoxelCellAddress& sampleAddress) const;
    // 返回指定最高层体素索引对应中心的局部空间位置。
    MyMath::Vector3 samplePosition(const VoxelCellIndex& sampleIndex) const;

private:
    // 返回未分配距离块中指定样本应使用的内部或外部背景距离。
    float backgroundValue(const VoxelCellAddress& sampleAddress) const;

private:
    const VoxelShape* m_shape; // 当前视图非拥有地引用的VoxelShape。
};

}

#endif // MYVOXEL_VOLUME_VOLUMEFIELDVIEW_H