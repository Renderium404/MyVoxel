#include "Topology_Curve.h"

#include <limits>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

// 判断标量是否为有限值。
bool isFiniteValue(double value)
{
    const double infinity = std::numeric_limits<double>::infinity();
    return value == value && value != infinity && value != -infinity;
}

// 判断点是否为局部XY平面中的有限点。
bool isFinitePlanarPoint(const MyMath::Vector3& point)
{
    return point.isFinite() && point.z() == 0.0;
}

}

namespace MyVoxel
{

Topology_Curve::Topology_Curve()
{
}

Topology_Curve::Topology_Curve(const Foundation::RefPtr<const Geometry_Curve>& geometry)
{
    MYVOXEL_ASSERT_MESSAGE(geometry, "Topology_Curve geometry must not be null.");

    if (!geometry)
    {
        return;
    }

    const bool valid = geometry->kind() != CurveKind::Unknown && isFinitePlanarPoint(geometry->startPoint()) && isFinitePlanarPoint(geometry->endPoint()) && isFiniteValue(geometry->length()) && geometry->length() > 0.0 && geometry->bounds().isValid();

    MYVOXEL_ASSERT_MESSAGE(valid, "Geometry_Curve must provide valid finite planar curve data.");

    if (!valid)
    {
        return;
    }

    m_geometry = geometry;
}

/// 状态判断

bool Topology_Curve::isValid() const
{
    return static_cast<bool>(m_geometry);
}

bool Topology_Curve::isNull() const
{
    return !m_geometry;
}

Topology_Curve::operator bool() const
{
    return isValid();
}

bool Topology_Curve::sharesGeometryWith(const Topology_Curve& other) const
{
    return m_geometry && m_geometry == other.m_geometry;
}

/// 几何资源

const Geometry_Curve& Topology_Curve::geometry() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the geometry of an invalid Topology_Curve.");
    return *m_geometry;
}

const Geometry_Curve* Topology_Curve::geometryPointer() const
{
    return m_geometry.get();
}

const Foundation::RefPtr<const Geometry_Curve>& Topology_Curve::geometryResource() const
{
    return m_geometry;
}

CurveKind Topology_Curve::kind() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the kind of an invalid Topology_Curve.");
    return m_geometry->kind();
}

/// 局部空间数据与查询

const MyMath::Vector3& Topology_Curve::startPoint() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the start point of an invalid Topology_Curve.");
    return m_geometry->startPoint();
}

const MyMath::Vector3& Topology_Curve::endPoint() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the end point of an invalid Topology_Curve.");
    return m_geometry->endPoint();
}

double Topology_Curve::length() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the length of an invalid Topology_Curve.");
    return m_geometry->length();
}

const Bounds3& Topology_Curve::bounds() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the bounds of an invalid Topology_Curve.");
    return m_geometry->bounds();
}

MyMath::Vector3 Topology_Curve::pointAt(double t) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Topology_Curve.");
    return m_geometry->pointAt(t);
}

MyMath::Vector3 Topology_Curve::tangentAt(double t) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Topology_Curve.");
    return m_geometry->tangentAt(t);
}

/// 拓扑创建

Topology_Curve Topology_Curve::reversed() const
{
    return isValid() ? Topology_Curve(m_geometry->reversed()) : Topology_Curve();
}

}