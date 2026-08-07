#include "Shape.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

Shape::Shape()
    : m_localToWorld(MyMath::Matrix4::identity())
    , m_worldToLocal(MyMath::Matrix4::identity())
    , m_valid(false)
{
}

Shape::Shape(const Topology_Shape& topology)
    : m_localToWorld(MyMath::Matrix4::identity())
    , m_worldToLocal(MyMath::Matrix4::identity())
    , m_valid(false)
{
    initialize(topology, MyMath::Matrix4::identity());
}

Shape::Shape(const Topology_Shape& topology, const MyMath::Matrix4& localToWorld)
    : m_localToWorld(MyMath::Matrix4::identity())
    , m_worldToLocal(MyMath::Matrix4::identity())
    , m_valid(false)
{
    initialize(topology, localToWorld);
}

/// 状态判断

bool Shape::isValid() const
{
    return m_valid;
}

bool Shape::isNull() const
{
    return m_topology.isNull();
}

Shape::operator bool() const
{
    return isValid();
}

bool Shape::sharesGeometryWith(const Shape& other) const
{
    return m_topology.sharesGeometryWith(other.m_topology);
}

/// 局部拓扑与几何资源

const Topology_Shape& Shape::topology() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the topology of an invalid Shape.");
    return m_topology;
}

const Geometry_Shape& Shape::geometry() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the geometry of an invalid Shape.");
    return m_topology.geometry();
}

const Geometry_Shape* Shape::geometryPointer() const
{
    return m_topology.geometryPointer();
}

ShapeKind Shape::kind() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the kind of an invalid Shape.");
    return m_topology.kind();
}

/// 空间数据

const Bounds3& Shape::localBounds() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the local bounds of an invalid Shape.");
    return m_topology.bounds();
}

const MyMath::Matrix4& Shape::localToWorld() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the transform of an invalid Shape.");
    return m_localToWorld;
}

const MyMath::Matrix4& Shape::worldToLocal() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the inverse transform of an invalid Shape.");
    return m_worldToLocal;
}

const Bounds3& Shape::worldBounds() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the world bounds of an invalid Shape.");
    return m_worldBounds;
}

/// 局部空间查询

bool Shape::containsLocalPoint(const MyMath::Vector3& point) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Shape.");
    return m_topology.containsPoint(point);
}

ShapeRelation Shape::classifyLocalBounds(const Bounds3& bounds) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Shape.");
    return m_topology.classifyBounds(bounds);
}

ShapeRelation Shape::classifyLocalBoundsFast(const MyMath::Vector3& center, const MyMath::Vector3& extent) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Shape.");
    return m_topology.classifyBoundsFast(center, extent);
}

/// 世界空间查询

bool Shape::containsWorldPoint(const MyMath::Vector3& point) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Shape.");
    MYVOXEL_ASSERT_MESSAGE(point.isFinite(), "Shape world query point must be finite.");

    if (!m_worldBounds.contains(point))
    {
        return false;
    }

    return m_topology.containsPoint(m_worldToLocal.transformPoint(point));
}

ShapeRelation Shape::classifyWorldBounds(const Bounds3& bounds) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Shape.");
    MYVOXEL_ASSERT_MESSAGE(bounds.isValid(), "Shape world query bounds must be valid.");

    if (!m_worldBounds.intersects(bounds))
    {
        return ShapeRelation::Outside;
    }

    return m_topology.classifyBounds(bounds.transformed(m_worldToLocal));
}

/// 初始化

void Shape::initialize(const Topology_Shape& topology, const MyMath::Matrix4& localToWorld)
{
    MYVOXEL_ASSERT_MESSAGE(topology.isValid(), "Shape topology must be valid.");
    MYVOXEL_ASSERT_MESSAGE(localToWorld.isAffine(), "Shape transform must be affine.");

    if (!topology.isValid() || !localToWorld.isAffine())
    {
        return;
    }

    MyMath::Matrix4 worldToLocal;
    const bool inverted = localToWorld.inverted(worldToLocal);

    MYVOXEL_ASSERT_MESSAGE(inverted, "Shape transform must be invertible.");

    if (!inverted)
    {
        return;
    }

    const Bounds3 worldBounds = topology.bounds().transformed(localToWorld);

    MYVOXEL_ASSERT_MESSAGE(worldBounds.isValid(), "Shape transformed world bounds must be valid.");

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