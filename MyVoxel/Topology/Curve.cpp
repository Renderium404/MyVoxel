#include "Curve.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

Curve::Curve()
    : m_localToWorld(MyMath::Matrix4::identity())
    , m_worldToLocal(MyMath::Matrix4::identity())
    , m_valid(false)
{
}

Curve::Curve(const Topology_Curve& topology)
    : m_localToWorld(MyMath::Matrix4::identity())
    , m_worldToLocal(MyMath::Matrix4::identity())
    , m_valid(false)
{
    initialize(topology, MyMath::Matrix4::identity());
}

Curve::Curve(const Topology_Curve& topology, const MyMath::Matrix4& localToWorld)
    : m_localToWorld(MyMath::Matrix4::identity())
    , m_worldToLocal(MyMath::Matrix4::identity())
    , m_valid(false)
{
    initialize(topology, localToWorld);
}

/// 状态判断

bool Curve::isValid() const
{
    return m_valid;
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
    return m_topology.sharesGeometryWith(other.m_topology);
}

/// 局部拓扑与几何资源

const Topology_Curve& Curve::topology() const
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
    return m_topology.geometryPointer();
}

CurveKind Curve::kind() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the kind of an invalid Curve.");
    return m_topology.kind();
}

/// 空间数据

const Bounds3& Curve::localBounds() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the local bounds of an invalid Curve.");
    return m_topology.bounds();
}

const MyMath::Matrix4& Curve::localToWorld() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the transform of an invalid Curve.");
    return m_localToWorld;
}

const MyMath::Matrix4& Curve::worldToLocal() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the inverse transform of an invalid Curve.");
    return m_worldToLocal;
}

const Bounds3& Curve::worldBounds() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the world bounds of an invalid Curve.");
    return m_worldBounds;
}

/// 局部空间查询

const MyMath::Vector3& Curve::localStartPoint() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the local start point of an invalid Curve.");
    return m_topology.startPoint();
}

const MyMath::Vector3& Curve::localEndPoint() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the local end point of an invalid Curve.");
    return m_topology.endPoint();
}

double Curve::localLength() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the local length of an invalid Curve.");
    return m_topology.length();
}

MyMath::Vector3 Curve::localPointAt(double t) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Curve.");
    return m_topology.pointAt(t);
}

MyMath::Vector3 Curve::localTangentAt(double t) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Curve.");
    return m_topology.tangentAt(t);
}

/// 世界空间查询

MyMath::Vector3 Curve::worldStartPoint() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the world start point of an invalid Curve.");
    return m_localToWorld.transformPoint(m_topology.startPoint());
}

MyMath::Vector3 Curve::worldEndPoint() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the world end point of an invalid Curve.");
    return m_localToWorld.transformPoint(m_topology.endPoint());
}

MyMath::Vector3 Curve::worldPointAt(double t) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Curve.");
    return m_localToWorld.transformPoint(m_topology.pointAt(t));
}

MyMath::Vector3 Curve::worldTangentAt(double t) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Curve.");
    const MyMath::Vector3 tangent = m_localToWorld.transformVector(m_topology.tangentAt(t));
    MYVOXEL_ASSERT_MESSAGE(tangent.isFinite() && tangent.lengthSquared() > 0.0, "Curve transformed tangent must be finite and non-zero.");
    return tangent.normalized(0.0);
}

/// 曲线创建

Curve Curve::reversed() const
{
    return isValid() ? Curve(m_topology.reversed(), m_localToWorld) : Curve();
}

/// 初始化

void Curve::initialize(const Topology_Curve& topology, const MyMath::Matrix4& localToWorld)
{
    MYVOXEL_ASSERT_MESSAGE(topology.isValid(), "Curve topology must be valid.");
    MYVOXEL_ASSERT_MESSAGE(localToWorld.isAffine(), "Curve transform must be affine.");

    if (!topology.isValid() || !localToWorld.isAffine())
    {
        return;
    }

    MyMath::Matrix4 worldToLocal;
    const bool inverted = localToWorld.inverted(worldToLocal);

    MYVOXEL_ASSERT_MESSAGE(inverted, "Curve transform must be invertible.");

    if (!inverted)
    {
        return;
    }

    const Bounds3 worldBounds = topology.bounds().transformed(localToWorld);

    MYVOXEL_ASSERT_MESSAGE(worldBounds.isValid(), "Curve transformed world bounds must be valid.");

    if (!worldBounds.isValid())
    {
        return;
    }

    m_topology = topology;
    m_localToWorld = localToWorld;
    m_worldToLocal = worldToLocal;
    m_worldBounds = worldBounds;
    m_valid = true;
}

}