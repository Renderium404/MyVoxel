#ifndef MYVOXEL_GEOMETRY_SHAPE_GEOMETRY_CONEFRUSTUM_H
#define MYVOXEL_GEOMETRY_SHAPE_GEOMETRY_CONEFRUSTUM_H

#include "Geometry_Shape.h"

namespace MyVoxel
{


// 表示轴线沿局部Z轴并且中心位于局部原点的标准圆锥台。
class Geometry_ConeFrustum : public Geometry_Shape
{
public:
    // 使用底面半径、顶面半径和完整高度创建标准圆锥台，两个半径允许其中一个为零。
    Geometry_ConeFrustum(double bottomRadius, double topRadius, double height);

    /// 几何参数

    // 返回位于局部Z负方向底面的半径。
    double bottomRadius() const;

    // 返回位于局部Z正方向顶面的半径。
    double topRadius() const;

    // 返回圆锥台完整高度。
    double height() const;

    // 返回指定局部Z坐标处的截面半径，localZ必须位于[-height / 2, height / 2]。
    double radiusAt(double localZ) const;

    /// 几何属性

    // 返回标准圆锥台类型。
    ShapeKind kind() const override;

    /// 空间查询

    // 判断指定局部坐标点是否位于圆锥台内部或边界上。
    bool containsLocalPoint(const MyMath::Vector3& point) const override;
    // 当前标准圆锥台提供精确局部有符号距离查询。
    bool supportsSignedDistance() const override;
    // 返回指定局部点到有限封闭圆锥台边界的精确有符号距离。
    double signedDistanceLocalPoint(const MyMath::Vector3& point) const override;

    // 返回指定局部轴对齐包围盒与圆锥台之间的保守空间关系。
    ShapeRelation classifyLocalBounds(const Bounds3& bounds) const override;

protected:
    // 通过侵入式引用计数管理标准圆锥台生命周期。
    ~Geometry_ConeFrustum() override = default;

private:
    double m_bottomRadius; // 位于局部Z负方向底面的半径。
    double m_topRadius; // 位于局部Z正方向顶面的半径。
    double m_height; // 圆锥台完整高度。
    double m_halfHeight; // 圆锥台半高度。
};


}

#endif // MYVOXEL_GEOMETRY_SHAPE_GEOMETRY_CONEFRUSTUM_H
