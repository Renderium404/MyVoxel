#ifndef MYVOXEL_CORE_VOXELGRID_H
#define MYVOXEL_CORE_VOXELGRID_H

#include "MyMath/Vector3.h"
#include "MyVoxel/Base/Bounds3.h"
#include "VoxelAddress.h"
#include "VoxelTypes.h"

namespace MyVoxel
{

// 定义无限稀疏体素的三维离散网格空间。
// 用于三维坐标与三维离散坐标进行映射
class VoxelGrid
{
public:
    // 构造原点为零、基础边长为1且最高层级为0的标准体素网格。
    VoxelGrid();
    // 构造原点为零的体素网格。
    VoxelGrid(double baseCellEdgeLength, VoxelLevel maximumLevel = BaseVoxelLevel);
    // 使用指定原点、基础体素边长和最高层级构造体素网格。
    VoxelGrid(const MyMath::Vector3& origin, double baseCellEdgeLength, VoxelLevel maximumLevel = BaseVoxelLevel);

    /// 状态判断
    // 判断网格原点、基础边长和最高层级边长是否能够构成有效空间网格。
    bool isValid() const;
    // 判断两个体素网格参数是否在指定误差内相等。
    bool isEqualTo(const VoxelGrid& other, double epsilon = MyMath::Vector3::DefaultEpsilon) const;

    /// 网格参数
    // 返回第0层索引(0, 0, 0)体素的最小角点世界坐标。
    const MyMath::Vector3& origin() const;
    // 返回第0层体素边长。
    double baseCellEdgeLength() const;
    // 返回当前网格允许使用的最高细分层级。
    VoxelLevel maximumLevel() const;
    // 返回指定层级的体素边长。
    double cellEdgeLength(VoxelLevel level) const;
    // 返回最高细分层级的体素边长。
    double minimumCellEdgeLength() const;

    /// 坐标与索引转换
    // 返回指定世界坐标点所属的体素索引，位于网格面上的点归入正方向体素。
    VoxelCellIndex cellIndex(const MyMath::Vector3& point, VoxelLevel level = BaseVoxelLevel) const;
    // 返回指定世界坐标点所属的体素地址。
    VoxelCellAddress cellAddress(const MyMath::Vector3& point, VoxelLevel level = BaseVoxelLevel) const;
    // 返回内部与指定有体积包围盒重叠的体素索引范围，最大边界按开区间处理。
    VoxelCellRange cellRange(const Bounds3& bounds, VoxelLevel level = BaseVoxelLevel) const;

    /// 体素空间属性
    // 返回指定体素地址对应的世界坐标轴对齐包围盒。
    Bounds3 cellBounds(const VoxelCellAddress& address) const;
    // 返回指定体素地址对应的世界坐标中心。
    MyMath::Vector3 cellCenter(const VoxelCellAddress& address) const;
    // 返回指定体素地址和角点方向对应的世界坐标。
    MyMath::Vector3 cellCorner(const VoxelCellAddress& address, VoxelCorner corner) const;

private:
    MyMath::Vector3 m_origin;       // 第0层索引(0, 0, 0)体素的最小角点世界坐标。
    double m_baseCellEdgeLength;    // 第0层体素边长。
    VoxelLevel m_maximumLevel;      // 当前网格允许使用的最高细分层级。
};

}

#endif // MYVOXEL_CORE_VOXELGRID_H