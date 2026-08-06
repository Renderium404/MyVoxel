#include "Geometry_Line.h"

#include <algorithm>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

// 判断指定点是否为局部XY平面中的有限点。
bool isFinitePlanarPoint(const MyMath::Vector3& point)
{
    return point.isFinite() && point.z() == 0.0;
}

}

namespace MyVoxel
{


Geometry_Line::Geometry_Line(const MyMath::Vector3& startPoint, const MyMath::Vector3& endPoint)
    : m_startPoint(startPoint)
    , m_endPoint(endPoint)
    , m_direction((endPoint - startPoint).normalized(0.0))
    , m_length(startPoint.distanceTo(endPoint))
    , m_bounds(MyMath::Vector3((std::min)(startPoint.x(), endPoint.x()), (std::min)(startPoint.y(), endPoint.y()), 0.0), MyMath::Vector3((std::max)(startPoint.x(), endPoint.x()), (std::max)(startPoint.y(), endPoint.y()), 0.0))
{
    MYVOXEL_ASSERT_MESSAGE(isFinitePlanarPoint(startPoint), "Geometry_Line start point must be finite and lie in the local XY plane.");
    MYVOXEL_ASSERT_MESSAGE(isFinitePlanarPoint(endPoint), "Geometry_Line end point must be finite and lie in the local XY plane.");
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