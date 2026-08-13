#ifndef MYVOXEL_GEOMETRY_CURVE_GEOMETRY_ARC_H
#define MYVOXEL_GEOMETRY_CURVE_GEOMETRY_ARC_H

#include "MyMath/CoordinateSystem.h"
#include "Geometry_Curve.h"

namespace MyVoxel
{

// 表示三维几何空间中位于指定正交坐标系局部XY平面内的有限有向圆弧。
class Geometry_Arc : public Geometry_Curve
{
public:
    // 在世界XY平面中使用圆心、半径、起始角和带符号扫掠角创建圆弧。
    Geometry_Arc(const MyMath::Vector3& center, double radius, double startAngle, double sweepAngle);
    // 在指定正交坐标系的局部XY平面中创建圆弧，正扫掠方向由局部+X指向局部+Y。
    Geometry_Arc(const MyMath::CoordinateSystem& coordinateSystem, double radius, double startAngle, double sweepAngle);

    /// 圆弧坐标系

    // 返回定义圆心和圆平面方向的正交坐标系。
    const MyMath::CoordinateSystem& coordinateSystem() const;
    // 返回圆心。
    const MyMath::Vector3& center() const;
    // 返回圆平面局部X轴。
    MyMath::Vector3 xAxis() const;
    // 返回圆平面局部Y轴。
    MyMath::Vector3 yAxis() const;
    // 返回圆平面法向轴。
    MyMath::Vector3 normal() const;

    /// 圆弧参数

    // 返回圆弧半径。
    double radius() const;
    // 返回相对于圆弧坐标系局部+X轴的起始角，单位为弧度。
    double startAngle() const;
    // 返回带符号扫掠角，单位为弧度。
    double sweepAngle() const;
    // 判断指定局部弧度角是否位于当前有向圆弧范围内。
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
    // 返回圆弧三维轴对齐包围盒。
    const Bounds3& bounds() const override;

    /// 参数查询

    // 返回规范化参数t对应的圆弧点。
    MyMath::Vector3 pointAt(double t) const override;
    // 返回规范化参数t对应且沿扫掠方向的单位切向量。
    MyMath::Vector3 tangentAt(double t) const override;

    /// 曲线创建

    // 返回圆平面和几何轨迹相同、起终点交换且扫掠方向相反的新圆弧几何。
    Foundation::RefPtr<const Geometry_Curve> reversed() const override;

protected:
    // 通过侵入式引用计数管理圆弧几何生命周期。
    ~Geometry_Arc() override = default;

private:
    // 返回指定局部角度对应的三维圆弧点。
    MyMath::Vector3 pointAtAngle(double angle) const;
    // 根据当前圆弧定义重新计算端点和精确三维轴对齐包围盒。
    void updateCachedData();

private:
    MyMath::CoordinateSystem m_coordinateSystem; // 定义圆心、圆平面局部XY轴和法向的正交坐标系。
    MyMath::Vector3 m_center; // 圆心缓存，用于保持稳定引用接口。
    double m_radius; // 圆弧半径。
    double m_startAngle; // 相对局部+X轴的起始角，单位为弧度。
    double m_sweepAngle; // 带符号扫掠角，单位为弧度。
    MyMath::Vector3 m_startPoint; // 圆弧起点。
    MyMath::Vector3 m_endPoint; // 圆弧终点。
    double m_length; // 圆弧完整长度。
    Bounds3 m_bounds; // 圆弧三维轴对齐包围盒。
};

}

#endif // MYVOXEL_GEOMETRY_CURVE_GEOMETRY_ARC_H
