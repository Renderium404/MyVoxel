#include "Curve.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

Curve::Curve()
    : Instance_Object()
{
}

Curve::Curve(const Topology_Edge& topology)
    : Instance_Object()
    , m_topology(topology)
{
    initialize();
}

Curve::Curve(const Topology_Edge& topology, const MyMath::Matrix4& localToWorld)
    : Instance_Object(localToWorld)
    , m_topology(topology)
{
    initialize();
}

/// 状态判断

bool Curve::isValid() const
{
    return m_topology.isValid() && isPlacementValid() && m_worldBounds.isValid();
}

bool Curve::isNull() const
{
    return m_topology.isNull();
}

Curve::operator bool() const
{
    return isValid();
}

bool Curve::sharesGeometryWith(const Curve& other) const
{
    const Geometry_Curve* currentGeometry = geometryPointer();
    const Geometry_Curve* otherGeometry = other.geometryPointer();
    return currentGeometry && currentGeometry == otherGeometry;
}

/// 局部拓扑与几何资源

const Topology_Edge& Curve::topology() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the topology of an invalid Curve.");
    return m_topology;
}

const Geometry_Curve& Curve::geometry() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the geometry of an invalid Curve.");
    return m_topology.geometry();
}

const Geometry_Curve* Curve::geometryPointer() const
{
    return m_topology.isValid() ? &m_topology.geometry() : nullptr;
}

CurveKind Curve::kind() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the kind of an invalid Curve.");
    return geometry().kind();
}

/// 空间数据

const Bounds3& Curve::localBounds() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the local bounds of an invalid Curve.");
    return geometry().bounds();
}

const MyMath::Matrix4& Curve::localToWorld() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the transform of an invalid Curve.");
    return Instance_Object::localToWorld();
}

const MyMath::Matrix4& Curve::worldToLocal() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the inverse transform of an invalid Curve.");
    return Instance_Object::worldToLocal();
}

const Bounds3& Curve::worldBounds() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the world bounds of an invalid Curve.");
    return m_worldBounds;
}

/// 曲线属性

MyMath::Vector3 Curve::startPoint() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the start point of an invalid Curve.");
    return m_topology.startVertex().point();
}

MyMath::Vector3 Curve::endPoint() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the end point of an invalid Curve.");
    return m_topology.endVertex().point();
}

MyMath::Vector3 Curve::worldStartPoint() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the world start point of an invalid Curve.");
    return Instance_Object::localToWorld().transformPoint(startPoint());
}

MyMath::Vector3 Curve::worldEndPoint() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the world end point of an invalid Curve.");
    return Instance_Object::localToWorld().transformPoint(endPoint());
}

double Curve::length() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the length of an invalid Curve.");
    return m_topology.length();
}

/// 局部空间查询

MyMath::Vector3 Curve::pointAt(double t) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Curve.");
    return m_topology.pointAt(t);
}

MyMath::Vector3 Curve::tangentAt(double t) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Curve.");
    return m_topology.tangentAt(t);
}

/// 世界空间查询

MyMath::Vector3 Curve::worldPointAt(double t) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Curve.");
    return Instance_Object::localToWorld().transformPoint(m_topology.pointAt(t));
}

MyMath::Vector3 Curve::worldTangentAt(double t) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Curve.");

    const MyMath::Vector3 tangent = Instance_Object::localToWorld().transformVector(m_topology.tangentAt(t));
    const MyMath::Vector3 normalizedTangent = tangent.normalized(0.0);
    MYVOXEL_ASSERT_MESSAGE(!normalizedTangent.isZero(0.0), "Curve world tangent must remain non-zero under an invertible affine transform.");
    return normalizedTangent;
}

/// 初始化

void Curve::initialize()
{
    MYVOXEL_ASSERT_MESSAGE(m_topology.isValid(), "Curve topology must be valid.");

    if (!m_topology.isValid() || !isPlacementValid())
    {
        return;
    }

    m_worldBounds = m_topology.geometry().bounds().transformed(Instance_Object::localToWorld());
    MYVOXEL_ASSERT_MESSAGE(m_worldBounds.isValid(), "Curve transformed world bounds must be valid.");

    if (!m_worldBounds.isValid())
    {
        m_worldBounds.clear();
    }
}

}