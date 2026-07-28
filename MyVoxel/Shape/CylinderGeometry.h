#ifndef MYVOXEL_CYLINDERGEOMETRY_H
#define MYVOXEL_CYLINDERGEOMETRY_H

#include "ShapeGeometry.h"

namespace MyVoxel
{

// 表示局部轴线平行于Z轴的连续圆柱体几何。
class CylinderGeometry : public ShapeGeometry
{
public:
    CylinderGeometry(double centerX, double centerY, double minimumZ, double maximumZ, double radius);

    /// 几何参数

    double centerX() const;
    double centerY() const;
    double minimumZ() const;
    double maximumZ() const;
    double radius() const;

    /// 局部空间查询

    ShapeBounds localBounds() const override;
    bool containsLocalPoint(const MyMath::Vector3& point) const override;
    ShapeRegionRelation classifyLocalBounds(const ShapeBounds& bounds) const override;

private:
    double m_centerX = 0.0; // 圆柱轴线局部X坐标。
    double m_centerY = 0.0; // 圆柱轴线局部Y坐标。
    double m_minimumZ = 0.0; // 圆柱轴向最小坐标。
    double m_maximumZ = 0.0; // 圆柱轴向最大坐标。
    double m_radius = 0.0; // 圆柱半径。
    double m_radiusSquared = 0.0; // 圆柱半径平方，用于高频包含和分类判断。
};

}

#endif // MYVOXEL_CYLINDERGEOMETRY_H