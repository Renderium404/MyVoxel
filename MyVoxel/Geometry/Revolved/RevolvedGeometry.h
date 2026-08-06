#ifndef MYVOXEL_GEOMETRY_REVOLVED_REVOLVEDGEOMETRY_H
#define MYVOXEL_GEOMETRY_REVOLVED_REVOLVEDGEOMETRY_H

#include "MyVoxel/Geometry/Curve/CurveLoop.h"
#include "MyVoxel/Geometry/ShapeGeometry.h"

namespace MyVoxel
{
namespace Geometry
{

// 表示将局部XY平面闭合轮廓映射到XOZ母线平面并绕局部Z轴完整旋转形成的连续实体。
// 轮廓X对应旋转半径，轮廓Y对应实体局部Z坐标；轮廓必须整体位于局部Y轴同一侧并允许接触该轴。
class RevolvedGeometry : public ShapeGeometry
{
public:
    // 使用有效闭合轮廓创建完整回转实体，轮廓不得跨越局部Y轴。
    explicit RevolvedGeometry(const CurveLoop& profile);

    /// 几何数据

    // 返回用于生成回转实体的局部XY平面闭合轮廓。
    const CurveLoop& profile() const;

    // 返回轮廓半径方向符号，右侧轮廓返回1，左侧轮廓返回-1。
    double radialSign() const;

    /// 几何属性

    // 返回完整回转几何类型。
    ShapeKind kind() const override;

    // 返回回转实体局部轴对齐包围盒。
    Bounds3 localBounds() const override;

    /// 空间查询

    // 判断指定局部三维点是否位于回转实体内部或边界上。
    bool containsLocalPoint(const MyMath::Vector3& point) const override;

    // 返回指定局部轴对齐包围盒与回转实体之间的保守空间关系。
    ShapeRelation classifyLocalBounds(const Bounds3& bounds) const override;

    /// 快速空间查询

    // 使用已经计算好的局部包围盒中心和半尺寸执行保守分类。
    ShapeRelation classifyLocalBoundsFast(const MyMath::Vector3& center, const MyMath::Vector3& extent) const override;

protected:
    ~RevolvedGeometry() override = default;

private:
    // 使用三维XY范围和Z范围执行回转截面降维分类。
    ShapeRelation classifyRange(const MyMath::Vector3& minimum, const MyMath::Vector3& maximum) const;

private:
    CurveLoop m_profile; // 局部XY平面中的闭合母线轮廓。
    double m_radialSign; // 轮廓X映射到非负旋转半径时使用的方向符号。
    Bounds3 m_bounds; // 回转实体局部轴对齐包围盒。
};

}
}

#endif // MYVOXEL_GEOMETRY_REVOLVED_REVOLVEDGEOMETRY_H
