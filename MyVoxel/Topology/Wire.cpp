#include "Wire.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

Wire::Wire()
    : m_localToWorld(MyMath::Matrix4::identity())
    , m_worldToLocal(MyMath::Matrix4::identity())
    , m_valid(false)
{
}

Wire::Wire(const Topology_Wire& topology)
    : m_localToWorld(MyMath::Matrix4::identity())
    , m_worldToLocal(MyMath::Matrix4::identity())
    , m_valid(false)
{
    initialize(topology, MyMath::Matrix4::identity());
}

Wire::Wire(const Topology_Wire& topology, const MyMath::Matrix4& localToWorld)
    : m_localToWorld(MyMath::Matrix4::identity())
    , m_worldToLocal(MyMath::Matrix4::identity())
    , m_valid(false)
{
    initialize(topology, localToWorld);
}

/// 状态判断

bool Wire::isValid() const
{
    return m_valid;
}

bool Wire::isNull() const
{
    return m_topology.isNull();
}

Wire::operator bool() const
{
    return isValid();
}

bool Wire::isClosed() const
{
    return isValid() && m_topology.isClosed();
}

bool Wire::hasArea() const
{
    return isValid() && m_topology.hasArea();
}

/// 局部拓扑

const Topology_Wire& Wire::topology() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the topology of an invalid Wire.");
    return m_topology;
}

std::size_t Wire::curveCount() const
{
    return isValid() ? m_topology.curveCount() : 0;
}

const Topology_Curve& Wire::localCurve(std::size_t index) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access a curve of an invalid Wire.");
    return m_topology.curve(index);
}

Curve Wire::worldCurve(std::size_t index) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot create a world curve from an invalid Wire.");
    return Curve(m_topology.curve(index), m_localToWorld);
}

/// 空间数据

const Bounds3& Wire::localBounds() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the local bounds of an invalid Wire.");
    return m_topology.bounds();
}

const MyMath::Matrix4& Wire::localToWorld() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the transform of an invalid Wire.");
    return m_localToWorld;
}

const MyMath::Matrix4& Wire::worldToLocal() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the inverse transform of an invalid Wire.");
    return m_worldToLocal;
}

const Bounds3& Wire::worldBounds() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the world bounds of an invalid Wire.");
    return m_worldBounds;
}

/// 局部空间查询

const MyMath::Vector3& Wire::localStartPoint() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the local start point of an invalid Wire.");
    return m_topology.startPoint();
}

const MyMath::Vector3& Wire::localEndPoint() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the local end point of an invalid Wire.");
    return m_topology.endPoint();
}

double Wire::localLength() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the local length of an invalid Wire.");
    return m_topology.length();
}

double Wire::localSignedArea() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the local area of an invalid Wire.");
    return m_topology.signedArea();
}

bool Wire::containsLocalPoint(const MyMath::Vector3& point, double tolerance) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Wire.");
    return m_topology.containsPoint(point, tolerance);
}

ShapeRelation Wire::classifyLocalBounds(const Bounds3& bounds, double tolerance) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Wire.");
    return m_topology.classifyBounds(bounds, tolerance);
}

/// 世界空间查询

MyMath::Vector3 Wire::worldStartPoint() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the world start point of an invalid Wire.");
    return m_localToWorld.transformPoint(m_topology.startPoint());
}

MyMath::Vector3 Wire::worldEndPoint() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the world end point of an invalid Wire.");
    return m_localToWorld.transformPoint(m_topology.endPoint());
}

/// Wire创建

Wire Wire::reversed() const
{
    return isValid() ? Wire(m_topology.reversed(), m_localToWorld) : Wire();
}

/// 初始化

void Wire::initialize(const Topology_Wire& topology, const MyMath::Matrix4& localToWorld)
{
    MYVOXEL_ASSERT_MESSAGE(topology.isValid(), "Wire topology must be valid.");
    MYVOXEL_ASSERT_MESSAGE(localToWorld.isAffine(), "Wire transform must be affine.");

    if (!topology.isValid() || !localToWorld.isAffine())
    {
        return;
    }

    MyMath::Matrix4 worldToLocal;
    const bool inverted = localToWorld.inverted(worldToLocal);

    MYVOXEL_ASSERT_MESSAGE(inverted, "Wire transform must be invertible.");

    if (!inverted)
    {
        return;
    }

    const Bounds3 worldBounds = topology.bounds().transformed(localToWorld);

    MYVOXEL_ASSERT_MESSAGE(worldBounds.isValid(), "Wire transformed world bounds must be valid.");

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