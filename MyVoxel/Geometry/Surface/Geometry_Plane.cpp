#include "Geometry_Plane.h"

#include <cmath>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

// 判断参数标量是否为有限值。
bool isFiniteValue(double value)
{
    return std::isfinite(value);
}

// 判断包围盒是否位于局部UV参数平面。
bool isPlanarParameterBounds(const MyVoxel::Bounds3& bounds)
{
    return bounds.isValid() && bounds.minimum().z() == 0.0 && bounds.maximum().z() == 0.0;
}

}

namespace MyVoxel
{

Geometry_Plane::Geometry_Plane(const MyMath::Vector3& origin, const MyMath::Vector3& axisU, const MyMath::Vector3& axisV)
    : m_origin(origin)
    , m_axisU(axisU)
    , m_axisV(axisV)
    , m_normal(MyMath::Vector3::cross(axisU, axisV).normalized(0.0))
{
    MYVOXEL_ASSERT_MESSAGE(origin.isFinite(), "Geometry_Plane origin must be finite.");
    MYVOXEL_ASSERT_MESSAGE(axisU.isFinite() && axisU.isVector(0.0), "Geometry_Plane axisU must be a finite non-zero vector.");
    MYVOXEL_ASSERT_MESSAGE(axisV.isFinite() && axisV.isVector(0.0), "Geometry_Plane axisV must be a finite non-zero vector.");
    MYVOXEL_ASSERT_MESSAGE(m_normal.isVector(0.0), "Geometry_Plane parameter axes must not be parallel.");
}

/// 平面数据

const MyMath::Vector3& Geometry_Plane::origin() const
{
    return m_origin;
}

const MyMath::Vector3& Geometry_Plane::axisU() const
{
    return m_axisU;
}

const MyMath::Vector3& Geometry_Plane::axisV() const
{
    return m_axisV;
}

const MyMath::Vector3& Geometry_Plane::normal() const
{
    return m_normal;
}

/// 曲面属性

SurfaceKind Geometry_Plane::kind() const
{
    return SurfaceKind::Plane;
}

/// 参数查询

MyMath::Vector3 Geometry_Plane::pointAt(double u, double v) const
{
    MYVOXEL_ASSERT_MESSAGE(isFiniteValue(u) && isFiniteValue(v), "Geometry_Plane parameters must be finite.");
    return m_origin + m_axisU * u + m_axisV * v;
}

MyMath::Vector3 Geometry_Plane::normalAt(double u, double v) const
{
    MYVOXEL_ASSERT_MESSAGE(isFiniteValue(u) && isFiniteValue(v), "Geometry_Plane parameters must be finite.");
    return m_normal;
}

MyMath::Vector3 Geometry_Plane::derivativeUAt(double u, double v) const
{
    MYVOXEL_ASSERT_MESSAGE(isFiniteValue(u) && isFiniteValue(v), "Geometry_Plane parameters must be finite.");
    return m_axisU;
}

MyMath::Vector3 Geometry_Plane::derivativeVAt(double u, double v) const
{
    MYVOXEL_ASSERT_MESSAGE(isFiniteValue(u) && isFiniteValue(v), "Geometry_Plane parameters must be finite.");
    return m_axisV;
}

/// 范围查询

Bounds3 Geometry_Plane::localBounds(const Bounds3& parameterBounds) const
{
    MYVOXEL_ASSERT_MESSAGE(isPlanarParameterBounds(parameterBounds), "Geometry_Plane parameter bounds must be valid and lie in the local UV plane.");

    if (!isPlanarParameterBounds(parameterBounds))
    {
        return Bounds3();
    }

    const double minimumU = parameterBounds.minimum().x();
    const double minimumV = parameterBounds.minimum().y();
    const double maximumU = parameterBounds.maximum().x();
    const double maximumV = parameterBounds.maximum().y();
    Bounds3 result;
    result.include(pointAt(minimumU, minimumV));
    result.include(pointAt(maximumU, minimumV));
    result.include(pointAt(maximumU, maximumV));
    result.include(pointAt(minimumU, maximumV));
    return result;
}

}