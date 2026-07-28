#ifndef MYVOXEL_BOXGEOMETRY_H
#define MYVOXEL_BOXGEOMETRY_H

#include "ShapeGeometry.h"

namespace MyVoxel
{

// 表示局部坐标轴对齐的连续盒体几何。
class BoxGeometry : public ShapeGeometry
{
public:
    BoxGeometry(double minimumX, double minimumY, double minimumZ, double maximumX, double maximumY, double maximumZ);

    /// 几何参数

    double minimumX() const;
    double minimumY() const;
    double minimumZ() const;
    double maximumX() const;
    double maximumY() const;
    double maximumZ() const;

    /// 局部空间查询

    ShapeBounds localBounds() const override;
    bool containsLocalPoint(const MyMath::Vector3& point) const override;
    ShapeRegionRelation classifyLocalBounds(const ShapeBounds& bounds) const override;

private:
    ShapeBounds m_bounds; // 盒体在自身局部坐标系中的范围。
};

}

#endif // MYVOXEL_BOXGEOMETRY_H