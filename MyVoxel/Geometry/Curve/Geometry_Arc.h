#ifndef MYVOXEL_GEOMETRY_CURVE_GEOMETRY_ARC_H
#define MYVOXEL_GEOMETRY_CURVE_GEOMETRY_ARC_H

#include "Geometry_Curve.h"

namespace MyVoxel
{

// 表示局部XY平面中由圆心、半径、起始角和带符号扫掠角定义的有限有向圆弧几何。
class Geometry_Arc : public Geometry_Curve
{
public:
    // 使用弧度制角度创建圆弧，正扫掠为逆时针，扫掠角绝对值不得大于完整圆。
    Geometry_Arc(const MyMath::Vector3& center, double radius, double startAngle, double sweepAngle);

    /// 圆弧参数

    // 返回圆心。
    const MyMath::Vector3& center() const;

    // 返回圆弧半径。
    double radius() const;

    // 返回圆弧起始角，单位为弧度。
    double startAngle() const;

    // 返回带符号扫掠角，单位为弧度。
    double sweepAngle() const;

    // 判断指定弧度角是否位于当前有向圆弧范围内。
    bool containsAngle(double angle, double tolerance = 0.0) const;

    /// 曲线属性

    // 返回圆弧类型。
    CurveKind kind() const override;

    // 返回圆弧起点。
    const MyMath::Vector3& startPoint() const override;

    // 返回圆弧终点。
    const MyMath::Vector3& endPoint() const override;

    // 返回圆弧长度。
    double length() const override;

    // 返回圆弧局部轴对齐包围盒。
    const Bounds3& bounds() const override;

    /// 参数查询

    // 返回规范化参数t对应的圆弧点。
    MyMath::Vector3 pointAt(double t) const override;

    // 返回规范化参数t对应且沿扫掠方向的单位切向量。
    MyMath::Vector3 tangentAt(double t) const override;

    /// 曲线创建

    // 返回起终点交换且扫掠方向相反的新圆弧几何。
    Foundation::RefPtr<const Geometry_Curve> reversed() const override;

protected:
    // 通过侵入式引用计数管理圆弧几何生命周期。
    ~Geometry_Arc() override = default;

private:
    // 根据当前标准参数重新计算端点和包围盒。
    void updateCachedData();

private:
    MyMath::Vector3 m_center; // 圆心。
    double m_radius; // 圆弧半径。
    double m_startAngle; // 圆弧起始角，单位为弧度。
    double m_sweepAngle; // 带符号扫掠角，单位为弧度。
    MyMath::Vector3 m_startPoint; // 圆弧起点。
    MyMath::Vector3 m_endPoint; // 圆弧终点。
    double m_length; // 圆弧完整长度。
    Bounds3 m_bounds; // 圆弧局部轴对齐包围盒。
};


}

#endif // MYVOXEL_GEOMETRY_CURVE_GEOMETRY_ARC_H
