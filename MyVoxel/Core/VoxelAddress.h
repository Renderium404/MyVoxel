#ifndef MYVOXEL_VOXELADDRESS_H
#define MYVOXEL_VOXELADDRESS_H

#include "VoxelTypes.h"

namespace MyVoxel
{

// 表示指定层级网格中的一个三维整数体素索引。
// child.x = parent.x × 2 + xBit
// child.y = parent.y × 2 + yBit
// child.z = parent.z × 2 + zBit
// 其中 xBit、yBit、zBit 只能是 0 或 1，由 VoxelCorner 的三个二进制位决定
class VoxelCellIndex
{
public:
    VoxelCellIndex() = default;

    // 使用三个方向的网格索引创建体素索引。
    VoxelCellIndex(VoxelIndex xValue, VoxelIndex yValue, VoxelIndex zValue);

    bool operator==(const VoxelCellIndex& other) const;
    bool operator!=(const VoxelCellIndex& other) const;
    bool operator<(const VoxelCellIndex& other) const;

    VoxelIndex x = 0; // 当前层级网格中的X方向索引。
    VoxelIndex y = 0; // 当前层级网格中的Y方向索引。
    VoxelIndex z = 0; // 当前层级网格中的Z方向索引。
};

// 使用层级和当前层级索引唯一标识一个体素单元。
class VoxelCellAddress
{
public:
    VoxelCellAddress() = default;

    // 使用指定体素索引和层级创建体素地址。
    VoxelCellAddress(const VoxelCellIndex& indexValue, VoxelLevel levelValue = BaseVoxelLevel);

    bool operator==(const VoxelCellAddress& other) const;
    bool operator!=(const VoxelCellAddress& other) const;
    bool operator<(const VoxelCellAddress& other) const;

    VoxelCellIndex index; // 当前层级网格中的体素索引。
    VoxelLevel level = BaseVoxelLevel; // 当前体素所处层级。
};

// 判断指定体素是否存在父体素。
bool hasParentCell(const VoxelCellAddress& address);

// 返回指定体素的上一级父体素地址。
VoxelCellAddress parentCellAddress(const VoxelCellAddress& address);

// 返回指定体素对应角点方向的下一级子体素地址。
VoxelCellAddress childCellAddress(const VoxelCellAddress& address, VoxelCorner childCorner);

// 返回指定非基础层体素在其父体素中的角点方向。
VoxelCorner childCornerInParent(const VoxelCellAddress& address);

}

#endif // MYVOXEL_VOXELADDRESS_H