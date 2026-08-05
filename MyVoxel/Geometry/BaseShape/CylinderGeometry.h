#ifndef MYVOXEL_GEOMETRY_CYLINDERGEOMETRY_H
#define MYVOXEL_GEOMETRY_CYLINDERGEOMETRY_H

#include "../ShapeGeometry.h"

namespace MyVoxel
{
namespace Geometry
{

// 表示轴线沿局部Z轴并且中心位于局部原点的标准圆柱体。
class CylinderGeometry : public ShapeGeometry
{
public:
    // 使用指定半径和完整高度创建标准圆柱体。
    CylinderGeometry(double radius, double height);

    /// 几何参数

    // 返回圆柱体半径。
    double radius() const;

    // 返回圆柱体完整高度。
    double height() const;

    /// 几何属性

    // 返回标准圆柱体类型。
    ShapeKind kind() const override;

    // 返回圆柱体局部轴对齐包围盒。
    Bounds3 localBounds() const override;

    /// 标准空间查询

    // 判断指定局部坐标点是否位于圆柱体内部或边界上。
    bool containsLocalPoint(const MyMath::Vector3& point) const override;

    // 返回指定局部轴对齐包围盒与圆柱体之间的保守空间关系。
    ShapeRelation classifyLocalBounds(const Bounds3& bounds) const override;


protected:
    // 通过侵入式引用计数管理标准圆柱体生命周期。
    ~CylinderGeometry() override = default;

private:
    double m_radius; // 圆柱体半径。
    double m_radiusSquared; // 圆柱体半径平方。
    double m_height; // 圆柱体完整高度。
    double m_halfHeight; // 圆柱体半高度。
    Bounds3 m_bounds; // 圆柱体局部轴对齐包围盒。
};

}
}

#endif // MYVOXEL_GEOMETRY_CYLINDERGEOMETRY_H