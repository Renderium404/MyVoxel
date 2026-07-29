#ifndef MYVOXEL_CORE_VOXELADDRESS_H
#define MYVOXEL_CORE_VOXELADDRESS_H

#include <cstdint>

#include "VoxelTypes.h"

namespace MyVoxel
{

// 表示指定层级网格中的一个三维整数体素索引。
class VoxelCellIndex
{
public:
    VoxelCellIndex();

    // 使用三个方向的整数索引创建体素索引。
    VoxelCellIndex(VoxelIndex xValue, VoxelIndex yValue, VoxelIndex zValue);

    bool operator==(const VoxelCellIndex& other) const;
    bool operator!=(const VoxelCellIndex& other) const;
    bool operator<(const VoxelCellIndex& other) const;

    VoxelIndex x; // 当前层级网格中的X方向索引。
    VoxelIndex y; // 当前层级网格中的Y方向索引。
    VoxelIndex z; // 当前层级网格中的Z方向索引。
};

// 使用层级和当前层级索引唯一标识一个体素单元。
class VoxelCellAddress
{
public:
    VoxelCellAddress();

    // 使用指定体素索引和层级创建体素地址。
    VoxelCellAddress(const VoxelCellIndex& indexValue, VoxelLevel levelValue = BaseVoxelLevel);

    bool operator==(const VoxelCellAddress& other) const;
    bool operator!=(const VoxelCellAddress& other) const;
    bool operator<(const VoxelCellAddress& other) const;

    VoxelCellIndex index; // 当前层级网格中的体素索引。
    VoxelLevel level; // 当前体素所处层级。
};

// 表示同一体素层级中包含最小和最大索引的闭区间。
class VoxelCellRange
{
public:
    // 构造无效体素索引范围。
    VoxelCellRange();

    // 使用同一层级中的最小和最大索引创建有效范围。
    VoxelCellRange(const VoxelCellIndex& minimumValue, const VoxelCellIndex& maximumValue, VoxelLevel levelValue = BaseVoxelLevel);

    /// 状态判断

    // 判断当前范围的三个方向是否均满足最小索引不大于最大索引。
    bool isValid() const;

    // 判断指定体素索引是否位于当前闭区间中。
    bool contains(const VoxelCellIndex& cellIndex) const;

    // 判断指定体素地址是否位于当前层级和闭区间中。
    bool contains(const VoxelCellAddress& address) const;

    /// 范围属性

    // 返回X方向包含的体素数量，无效范围返回零。
    std::uint64_t countX() const;

    // 返回Y方向包含的体素数量，无效范围返回零。
    std::uint64_t countY() const;

    // 返回Z方向包含的体素数量，无效范围返回零。
    std::uint64_t countZ() const;

    VoxelCellIndex minimum; // 当前范围包含的最小体素索引。
    VoxelCellIndex maximum; // 当前范围包含的最大体素索引。
    VoxelLevel level; // 当前范围所属的体素层级。
};

/// 体素层级关系

// 判断指定体素是否存在上一级父体素。
bool hasParentCell(const VoxelCellAddress& address);

// 返回指定体素的上一级父体素地址。
VoxelCellAddress parentCellAddress(const VoxelCellAddress& address);

// 返回指定父体素对应角点方向的下一级子体素地址。
VoxelCellAddress childCellAddress(const VoxelCellAddress& address, VoxelCorner childCorner);

// 返回指定非基础层体素在其父体素中的角点方向。
VoxelCorner childCornerInParent(const VoxelCellAddress& address);

// 返回指定体素在目标上级层级中的祖先体素地址。
VoxelCellAddress ancestorCellAddress(const VoxelCellAddress& address, VoxelLevel targetLevel);

// 返回指定体素所属的第0层根体素地址。
VoxelCellAddress rootCellAddress(const VoxelCellAddress& address);

}

#endif // MYVOXEL_CORE_VOXELADDRESS_H