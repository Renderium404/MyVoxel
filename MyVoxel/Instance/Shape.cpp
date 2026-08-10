#include "Shape.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

Shape::Shape()
    : Instance_Object()
{
}

Shape::Shape(const Topology_Shape& topology)
    : Instance_Object()
    , m_topology(topology)
{
    initialize();
}

Shape::Shape(const Topology_Shape& topology, const MyMath::Matrix4& localToWorld)
    : Instance_Object(localToWorld)
    , m_topology(topology)
{
    initialize();
}

/// 状态判断

bool Shape::isValid() const
{
    return m_topology.isValid() && isPlacementValid() && m_worldBounds.isValid();
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
    const Geometry_Shape* currentGeometry = geometryPointer();
    const Geometry_Shape* otherGeometry = other.geometryPointer();
    return currentGeometry && currentGeometry == otherGeometry;
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
    return m_topology.isValid() ? &m_topology.geometry() : nullptr;
}

ShapeKind Shape::kind() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the kind of an invalid Shape.");
    return geometry().kind();
}

/// 空间数据

const Bounds3& Shape::localBounds() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the local bounds of an invalid Shape.");
    return geometry().localBounds();
}

const MyMath::Matrix4& Shape::localToWorld() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the transform of an invalid Shape.");
    return Instance_Object::localToWorld();
}

const MyMath::Matrix4& Shape::worldToLocal() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the inverse transform of an invalid Shape.");
    return Instance_Object::worldToLocal();
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
    return geometry().containsLocalPoint(point);
}

ShapeRelation Shape::classifyLocalBounds(const Bounds3& bounds) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Shape.");
    return geometry().classifyLocalBounds(bounds);
}

ShapeRelation Shape::classifyLocalBoundsFast(const MyMath::Vector3& center, const MyMath::Vector3& extent) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Shape.");
    return geometry().classifyLocalBoundsFast(center, extent);
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

    return geometry().containsLocalPoint(Instance_Object::worldToLocal().transformPoint(point));
}

ShapeRelation Shape::classifyWorldBounds(const Bounds3& bounds) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Shape.");
    MYVOXEL_ASSERT_MESSAGE(bounds.isValid(), "Shape world query bounds must be valid.");

    if (!m_worldBounds.intersects(bounds))
    {
        return ShapeRelation::Outside;
    }

    const Bounds3 localBounds = bounds.transformed(Instance_Object::worldToLocal());
    return geometry().classifyLocalBounds(localBounds);
}

/// 初始化

void Shape::initialize()
{
    MYVOXEL_ASSERT_MESSAGE(m_topology.isValid(), "Shape topology must be valid.");

    if (!m_topology.isValid() || !isPlacementValid())
    {
        return;
    }

    m_worldBounds = m_topology.geometry().localBounds().transformed(Instance_Object::localToWorld());
    MYVOXEL_ASSERT_MESSAGE(m_worldBounds.isValid(), "Shape transformed world bounds must be valid.");

    if (!m_worldBounds.isValid())
    {
        m_worldBounds.clear();
    }
}

}