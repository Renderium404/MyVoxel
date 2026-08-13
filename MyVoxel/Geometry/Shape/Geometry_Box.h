#ifndef MYVOXEL_GEOMETRY_SHAPE_GEOMETRY_BOX_H
#define MYVOXEL_GEOMETRY_SHAPE_GEOMETRY_BOX_H

#include "Geometry_Shape.h"

namespace MyVoxel
{


// 表示以局部原点为中心并与局部坐标轴平行的标准长方体。
class Geometry_Box : public Geometry_Shape
{
public:
    // 使用三个方向的完整尺寸创建标准长方体。
    Geometry_Box(double sizeX, double sizeY, double sizeZ);

    /// 几何参数

    // 返回长方体X方向完整尺寸。
    double sizeX() const;

    // 返回长方体Y方向完整尺寸。
    double sizeY() const;

    // 返回长方体Z方向完整尺寸。
    double sizeZ() const;

    /// 几何属性

    // 返回标准长方体类型。
    ShapeKind kind() const override;

    /// 空间查询

    // 判断指定局部坐标点是否位于长方体内部或边界上。
    bool containsLocalPoint(const MyMath::Vector3& point) const override;
    // 当前标准长方体提供精确局部有符号距离查询。
    bool supportsSignedDistance() const override;
    // 返回指定局部点到长方体边界的精确有符号距离。
    double signedDistanceLocalPoint(const MyMath::Vector3& point) const override;

    // 返回指定局部轴对齐包围盒与长方体之间的保守空间关系。
    ShapeRelation classifyLocalBounds(const Bounds3& bounds) const override;

protected:
    // 通过侵入式引用计数管理标准长方体生命周期。
    ~Geometry_Box() override = default;

private:
    double m_sizeX; // 长方体X方向完整尺寸。
    double m_sizeY; // 长方体Y方向完整尺寸。
    double m_sizeZ; // 长方体Z方向完整尺寸。
};


}

#endif // MYVOXEL_GEOMETRY_SHAPE_GEOMETRY_BOX_H
