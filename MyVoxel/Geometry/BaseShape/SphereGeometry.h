#ifndef MYVOXEL_GEOMETRY_SPHEREGEOMETRY_H
#define MYVOXEL_GEOMETRY_SPHEREGEOMETRY_H

#include "../ShapeGeometry.h"

namespace MyVoxel
{
namespace Geometry
{

// 表示球心位于局部原点的标准球体。
class SphereGeometry : public ShapeGeometry
{
public:
    // 使用指定半径创建标准球体。
    explicit SphereGeometry(double radius);

    /// 几何参数

    // 返回球体半径。
    double radius() const;

    /// 几何属性

    // 返回标准球体类型。
    ShapeKind kind() const override;

    // 返回球体局部轴对齐包围盒。
    Bounds3 localBounds() const override;

    /// 空间查询

    // 判断指定局部坐标点是否位于球体内部或边界上。
    bool containsLocalPoint(const MyMath::Vector3& point) const override;

    // 返回指定局部轴对齐包围盒与球体之间的保守空间关系。
    ShapeRelation classifyLocalBounds(const Bounds3& bounds) const override;

protected:
    // 通过侵入式引用计数管理标准球体生命周期。
    ~SphereGeometry() override = default;

private:
    double m_radius; // 球体半径。
    double m_radiusSquared; // 球体半径平方。
    Bounds3 m_bounds; // 球体局部轴对齐包围盒。
};

}
}

#endif // MYVOXEL_GEOMETRY_SPHEREGEOMETRY_H