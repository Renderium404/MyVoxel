#ifndef MYVOXEL_GEOMETRY_SURFACE_GEOMETRY_PLANE_H
#define MYVOXEL_GEOMETRY_SURFACE_GEOMETRY_PLANE_H

#include "Geometry_Surface.h"

namespace MyVoxel
{

// 表示由原点和两个不共线参数轴定义的无限平面，P(u,v)=origin+axisU*u+axisV*v。
class Geometry_Plane : public Geometry_Surface
{
public:
    // 使用有限原点和两个有限非零且不共线的参数轴创建平面。
    Geometry_Plane(const MyMath::Vector3& origin, const MyMath::Vector3& axisU, const MyMath::Vector3& axisV);

    /// 平面数据

    // 返回参数原点。
    const MyMath::Vector3& origin() const;
    // 返回u参数增加一个单位时使用的局部方向向量。
    const MyMath::Vector3& axisU() const;
    // 返回v参数增加一个单位时使用的局部方向向量。
    const MyMath::Vector3& axisV() const;
    // 返回axisU叉乘axisV确定的单位正向法线。
    const MyMath::Vector3& normal() const;

    /// 曲面属性

    // 返回平面曲面类型。
    SurfaceKind kind() const override;

    /// 参数查询

    // 返回参数(u,v)对应的局部三维平面点。
    MyMath::Vector3 pointAt(double u, double v) const override;
    // 返回平面恒定单位法线。
    MyMath::Vector3 normalAt(double u, double v) const override;
    // 返回平面恒定u偏导向量axisU。
    MyMath::Vector3 derivativeUAt(double u, double v) const override;
    // 返回平面恒定v偏导向量axisV。
    MyMath::Vector3 derivativeVAt(double u, double v) const override;

    /// 范围查询

    // 变换参数矩形四个角点并返回精确局部三维轴对齐包围盒。
    Bounds3 localBounds(const Bounds3& parameterBounds) const override;

protected:
    // 通过侵入式引用计数管理平面几何生命周期。
    ~Geometry_Plane() override = default;

private:
    MyMath::Vector3 m_origin; // 参数原点。
    MyMath::Vector3 m_axisU; // u参数方向及单位参数尺度。
    MyMath::Vector3 m_axisV; // v参数方向及单位参数尺度。
    MyMath::Vector3 m_normal; // axisU叉乘axisV得到的单位正向法线。
};

}

#endif // MYVOXEL_GEOMETRY_SURFACE_GEOMETRY_PLANE_H