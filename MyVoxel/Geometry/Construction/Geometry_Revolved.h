#ifndef MYVOXEL_GEOMETRY_CONSTRUCTION_GEOMETRY_REVOLVED_H
#define MYVOXEL_GEOMETRY_CONSTRUCTION_GEOMETRY_REVOLVED_H

#include <cstddef>
#include <vector>

#include "MyVoxel/Foundation/RefPtr.h"
#include "MyVoxel/Geometry/Curve/Geometry_Curve.h"
#include "MyVoxel/Geometry/Shape/Geometry_Shape.h"

namespace MyVoxel
{

// 表示由局部XY平面闭合母线轮廓映射到XOZ截面并绕局部Z轴完整旋转形成的连续实体几何。
//
// 母线X对应带符号旋转半径，母线Y对应实体局部Z坐标；母线必须闭合、具有非零面积并整体位于局部Y轴同一侧。
// 输入Line或Arc允许存在profileTolerance范围内的平面数值误差，内部统一规范化为严格位于局部XY平面的独立曲线资源。
class Geometry_Revolved : public Geometry_Shape
{
public:
    // 使用有序闭合曲线几何创建完整回转实体，profileTolerance用于母线平面、连接和边界数值判断。
    Geometry_Revolved(const std::vector<Foundation::RefPtr<const Geometry_Curve> >& profileCurves, double profileTolerance);

    /// 状态判断

    // 判断当前回转几何是否具有完整有效的闭合母线和非退化实体范围。
    bool isValid() const;

    /// 母线几何数据

    // 返回规范化母线包含的曲线数量。
    std::size_t profileCurveCount() const;
    // 返回指定编号的规范化母线曲线。
    const Geometry_Curve& profileCurve(std::size_t index) const;
    // 返回全部规范化且严格位于局部XY平面的不可变母线曲线资源。
    const std::vector<Foundation::RefPtr<const Geometry_Curve> >& profileCurves() const;
    // 返回母线平面、连接和区域查询使用的几何容差。
    double profileTolerance() const;
    // 返回规范化母线在局部XY平面中的轴对齐包围盒。
    const Bounds3& profileBounds() const;
    // 返回规范化闭合母线的精确有符号面积，逆时针为正，顺时针为负。
    double profileSignedArea() const;
    // 返回母线X映射到非负旋转半径时使用的方向符号，右侧为1，左侧为-1。
    double radialSign() const;

    /// 几何属性

    // 返回完整回转实体类型。
    ShapeKind kind() const override;

    /// 标准空间查询

    // 判断指定局部三维点是否位于回转实体内部或边界上。
    bool containsLocalPoint(const MyMath::Vector3& point) const override;
    // 当前完整回转实体提供精确局部有符号距离查询。
    bool supportsSignedDistance() const override;
    // 返回指定局部点到回转实体边界的精确有符号距离。
    double signedDistanceLocalPoint(const MyMath::Vector3& point) const override;
    // 返回指定局部轴对齐包围盒与回转实体之间的保守空间关系。
    ShapeRelation classifyLocalBounds(const Bounds3& bounds) const override;

    /// 快速空间查询

    // 使用已经计算好的局部包围盒中心和半尺寸执行保守分类。
    ShapeRelation classifyLocalBoundsFast(const MyMath::Vector3& center, const MyMath::Vector3& extent) const override;

protected:
    // 通过侵入式引用计数管理回转实体几何生命周期。
    ~Geometry_Revolved() override = default;

private:
    // 将输入Line和任意坐标系Arc规范化为严格位于当前局部XY平面的独立母线曲线。
    bool buildCanonicalProfile(const std::vector<Foundation::RefPtr<const Geometry_Curve> >& profileCurves);
    // 验证规范化母线并建立面积、半径方向和实体包围盒缓存。
    void rebuild();
    // 判断指定局部XY平面点是否位于母线区域内部或边界上。
    bool containsProfilePoint(const MyMath::Vector3& point, double tolerance) const;
    // 返回指定局部XY平面矩形与母线区域之间的保守空间关系。
    ShapeRelation classifyProfileBounds(const Bounds3& bounds, double tolerance) const;
    // 使用三维XY范围和Z范围执行回转截面降维分类。
    ShapeRelation classifyRange(const MyMath::Vector3& minimum, const MyMath::Vector3& maximum) const;

private:
    std::vector<Foundation::RefPtr<const Geometry_Curve> > m_profileCurves; // 严格位于局部XY平面并按轮廓方向排列的规范化母线曲线。
    double m_profileTolerance; // 母线平面、连接和区域查询使用的几何容差。
    Bounds3 m_profileBounds; // 规范化母线在局部XY平面中的轴对齐包围盒。
    double m_profileSignedArea; // 规范化闭合母线的有符号面积。
    double m_radialSign; // 母线X映射到非负旋转半径时使用的方向符号。
    bool m_valid; // 当前回转几何是否具有完整有效数据。
};

}

#endif // MYVOXEL_GEOMETRY_CONSTRUCTION_GEOMETRY_REVOLVED_H
