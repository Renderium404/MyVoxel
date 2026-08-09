#ifndef MYVOXEL_GEOMETRY_CURVE_GEOMETRY_LINE_H
#define MYVOXEL_GEOMETRY_CURVE_GEOMETRY_LINE_H

#include "Geometry_Curve.h"

namespace MyVoxel
{

// 表示三维几何空间中的有限有向直线段。
class Geometry_Line : public Geometry_Curve
{
public:
    // 使用两个不同的有限三维点创建直线段。
    Geometry_Line(const MyMath::Vector3& startPoint, const MyMath::Vector3& endPoint);

    /// 曲线属性

    // 返回直线段类型。
    CurveKind kind() const override;
    // 返回直线段起点。
    const MyMath::Vector3& startPoint() const override;
    // 返回直线段终点。
    const MyMath::Vector3& endPoint() const override;
    // 返回直线段长度。
    double length() const override;
    // 返回直线段三维轴对齐包围盒。
    const Bounds3& bounds() const override;

    /// 参数查询

    // 返回规范化参数t对应的线性插值点。
    MyMath::Vector3 pointAt(double t) const override;
    // 返回沿起点指向终点的单位方向。
    MyMath::Vector3 tangentAt(double t) const override;

    /// 曲线创建

    // 返回起终点交换后的反向直线段几何。
    Foundation::RefPtr<const Geometry_Curve> reversed() const override;

protected:
    // 通过侵入式引用计数管理直线段几何生命周期。
    ~Geometry_Line() override = default;

private:
    MyMath::Vector3 m_startPoint; // 直线段起点。
    MyMath::Vector3 m_endPoint; // 直线段终点。
    MyMath::Vector3 m_direction; // 起点指向终点的单位方向。
    double m_length; // 直线段完整长度。
    Bounds3 m_bounds; // 直线段三维轴对齐包围盒。
};

}

#endif // MYVOXEL_GEOMETRY_CURVE_GEOMETRY_LINE_H
