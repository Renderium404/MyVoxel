#ifndef MYVOXEL_SURFACE_VOXELFACEADDRESS_H
#define MYVOXEL_SURFACE_VOXELFACEADDRESS_H

#include <cstdint>

#include "MyVoxel/Core/VoxelAddress.h"

namespace MyVoxel
{

// 标识最高层材料体素的一个轴对齐外表面方向。
enum class VoxelFaceDirection : std::uint8_t
{
    NegativeX = 0,
    PositiveX = 1,
    NegativeY = 2,
    PositiveY = 3,
    NegativeZ = 4,
    PositiveZ = 5
};

const unsigned int VoxelFaceDirectionCount = 6;

// 使用最高层材料体素索引和外法线方向唯一标识一个有向单位体素面。
//
// cellIndex始终属于与当前面集合关联的VoxelGrid::maximumLevel()。
// 当前地址本身不保存层级，不能脱离对应VoxelGrid单独解释其空间尺寸。
class VoxelFaceAddress
{
public:
    // 构造位于原点最高层体素NegativeX方向的默认面地址。
    VoxelFaceAddress();

    // 使用最高层材料体素索引和外法线方向构造面地址。
    VoxelFaceAddress(const VoxelCellIndex& cellIndexValue, VoxelFaceDirection directionValue);

    bool operator==(const VoxelFaceAddress& other) const;
    bool operator!=(const VoxelFaceAddress& other) const;
    bool operator<(const VoxelFaceAddress& other) const;

    VoxelCellIndex cellIndex; // 拥有当前外表面的最高层材料体素索引。
    VoxelFaceDirection direction; // 当前表面的外法线方向。
};

/// 面方向查询

// 判断指定值是否为有效体素面方向。
bool isValidVoxelFaceDirection(VoxelFaceDirection direction);

// 返回指定面方向的相反方向。
VoxelFaceDirection oppositeVoxelFaceDirection(VoxelFaceDirection direction);

}

#endif // MYVOXEL_SURFACE_VOXELFACEADDRESS_H