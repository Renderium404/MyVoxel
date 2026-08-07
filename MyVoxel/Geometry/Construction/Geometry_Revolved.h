#ifndef MYVOXEL_GEOMETRY_CONSTRUCTION_GEOMETRY_REVOLVED_H
#define MYVOXEL_GEOMETRY_CONSTRUCTION_GEOMETRY_REVOLVED_H

#include <cstddef>
#include <vector>

#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"
#include "MyVoxel/Geometry/Shape/Geometry_Shape.h"

namespace MyVoxel
{

// 表示由局部XY平面闭合几何轮廓映射到XOZ母线平面并绕局部Z轴完整旋转形成的连续实体几何。
// 轮廓X对应旋转半径，轮廓Y对应实体局部Z坐标；轮廓必须闭合、具有非零面积并整体位于局部Y轴同一侧。
class Geometry_Revolved : public Geometry_Shape
{
public:
    // 使用有序闭合曲线几何创建完整回转实体，建模层必须传入连续且不跨越局部Y轴的轮廓。
    Geometry_Revolved(const std::vector<Foundation::RefPtr<const Geometry_Curve> >& profileCurves, double connectionTolerance);

    /// 轮廓几何数据

    // 返回母线轮廓包含的曲线几何数量。
    std::size_t profileCurveCount() const;
    // 返回指定编号的母线曲线几何。
    const Geometry_Curve& profileCurve(std::size_t index) const;
    // 返回母线轮廓持有的全部不可变曲线几何资源。
    const std::vector<Foundation::RefPtr<const Geometry_Curve> >& profileCurves() const;
    // 返回相邻轮廓曲线允许的最大连接距离。
    double connectionTolerance() const;
    // 返回母线轮廓在局部XY平面中的轴对齐包围盒。
    const Bounds3& profileBounds() const;
    // 返回母线轮廓精确有符号面积，逆时针为正，顺时针为负。
    double profileSignedArea() const;
    // 返回轮廓X映射到非负旋转半径时使用的方向符号，右侧为1，左侧为-1。
    double radialSign() const;

    /// 几何属性

    // 返回完整回转实体类型。
    ShapeKind kind() const override;

    /// 标准空间查询

    // 判断指定局部三维点是否位于回转实体内部或边界上。
    bool containsLocalPoint(const MyMath::Vector3& point) const override;
    // 返回指定局部轴对齐包围盒与回转实体之间的保守空间关系。
    ShapeRelation classifyLocalBounds(const Bounds3& bounds) const override;

    /// 快速空间查询

    // 使用已经计算好的局部包围盒中心和半尺寸执行保守分类。
    ShapeRelation classifyLocalBoundsFast(const MyMath::Vector3& center, const MyMath::Vector3& extent) const override;

protected:
    // 通过侵入式引用计数管理回转实体几何生命周期。
    ~Geometry_Revolved() override = default;

private:
    // 验证轮廓几何并建立回转查询缓存。
    void rebuild();
    // 判断指定局部XY平面点是否位于母线轮廓区域内部或边界上。
    bool containsProfilePoint(const MyMath::Vector3& point, double tolerance) const;
    // 返回指定局部XY平面矩形与母线轮廓区域之间的保守空间关系。
    ShapeRelation classifyProfileBounds(const Bounds3& bounds, double tolerance) const;
    // 使用三维XY范围和Z范围执行回转截面降维分类。
    ShapeRelation classifyRange(const MyMath::Vector3& minimum, const MyMath::Vector3& maximum) const;

private:
    std::vector<Foundation::RefPtr<const Geometry_Curve> > m_profileCurves; // 按轮廓方向排列的不可变曲线几何资源。
    double m_connectionTolerance; // 相邻轮廓曲线允许的最大连接距离。
    Bounds3 m_profileBounds; // 母线轮廓在局部XY平面中的轴对齐包围盒。
    double m_profileSignedArea; // 母线轮廓精确有符号面积。
    double m_radialSign; // 轮廓X映射到非负旋转半径时使用的方向符号。
    bool m_valid; // 当前回转几何是否包含完整有效数据。
};

}

#endif // MYVOXEL_GEOMETRY_CONSTRUCTION_GEOMETRY_REVOLVED_H