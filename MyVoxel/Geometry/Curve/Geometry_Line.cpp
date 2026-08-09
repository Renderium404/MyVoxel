#include "Geometry_Line.h"

#include <algorithm>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

Geometry_Line::Geometry_Line(const MyMath::Vector3& startPoint, const MyMath::Vector3& endPoint)
    : m_startPoint(startPoint)
    , m_endPoint(endPoint)
    , m_direction((endPoint - startPoint).normalized(0.0))
    , m_length(startPoint.distanceTo(endPoint))
    , m_bounds(MyMath::Vector3((std::min)(startPoint.x(), endPoint.x()), (std::min)(startPoint.y(), endPoint.y()), (std::min)(startPoint.z(), endPoint.z())),
               MyMath::Vector3((std::max)(startPoint.x(), endPoint.x()), (std::max)(startPoint.y(), endPoint.y()), (std::max)(startPoint.z(), endPoint.z())))
{
    MYVOXEL_ASSERT_MESSAGE(startPoint.isFinite(), "Geometry_Line start point must be finite.");
    MYVOXEL_ASSERT_MESSAGE(endPoint.isFinite(), "Geometry_Line end point must be finite.");
    MYVOXEL_ASSERT_MESSAGE(m_length > 0.0, "Geometry_Line start and end points must be different.");
}

/// 曲线属性

CurveKind Geometry_Line::kind() const
{
    return CurveKind::Line;
}

const MyMath::Vector3& Geometry_Line::startPoint() const
{
    return m_startPoint;
}

const MyMath::Vector3& Geometry_Line::endPoint() const
{
    return m_endPoint;
}

double Geometry_Line::length() const
{
    return m_length;
}

const Bounds3& Geometry_Line::bounds() const
{
    return m_bounds;
}

/// 参数查询

MyMath::Vector3 Geometry_Line::pointAt(double t) const
{
    MYVOXEL_ASSERT_MESSAGE(t >= 0.0 && t <= 1.0, "Geometry_Line parameter must be in [0,1].");
    return m_startPoint + (m_endPoint - m_startPoint) * t;
}

MyMath::Vector3 Geometry_Line::tangentAt(double t) const
{
    MYVOXEL_ASSERT_MESSAGE(t >= 0.0 && t <= 1.0, "Geometry_Line parameter must be in [0,1].");
    return m_direction;
}

/// 曲线创建

Foundation::RefPtr<const Geometry_Curve> Geometry_Line::reversed() const
{
    return Foundation::RefPtr<const Geometry_Curve>(new Geometry_Line(m_endPoint, m_startPoint));
}

}
