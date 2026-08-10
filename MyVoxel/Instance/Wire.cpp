#include "Wire.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

Wire::Wire()
    : Instance_Object()
{
}

Wire::Wire(const Topology_Wire& topology)
    : Instance_Object()
    , m_topology(topology)
{
    initialize();
}

Wire::Wire(const Topology_Wire& topology, const MyMath::Matrix4& localToWorld)
    : Instance_Object(localToWorld)
    , m_topology(topology)
{
    initialize();
}

/// 状态判断

bool Wire::isValid() const
{
    return m_topology.isValid() && isPlacementValid() && m_localBounds.isValid() && m_worldBounds.isValid();
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

/// 局部拓扑

const Topology_Wire& Wire::topology() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the topology of an invalid Wire.");
    return m_topology;
}

std::size_t Wire::edgeCount() const
{
    return m_topology.edgeCount();
}

Topology_Edge Wire::edge(std::size_t index) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access an edge of an invalid Wire.");
    return m_topology.edge(index);
}

std::vector<Topology_Edge> Wire::edges() const
{
    return m_topology.edges();
}

/// Curve实例

Curve Wire::curve(std::size_t index) const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access a curve of an invalid Wire.");
    MYVOXEL_ASSERT_MESSAGE(index < edgeCount(), "Wire curve index is out of range.");

    if (!isValid() || index >= edgeCount())
    {
        return Curve();
    }

    return Curve(m_topology.edge(index), Instance_Object::localToWorld());
}

/// 空间数据

const Bounds3& Wire::localBounds() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the local bounds of an invalid Wire.");
    return m_localBounds;
}

const MyMath::Matrix4& Wire::localToWorld() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the transform of an invalid Wire.");
    return Instance_Object::localToWorld();
}

const MyMath::Matrix4& Wire::worldToLocal() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the inverse transform of an invalid Wire.");
    return Instance_Object::worldToLocal();
}

const Bounds3& Wire::worldBounds() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the world bounds of an invalid Wire.");
    return m_worldBounds;
}

/// 有向端点

MyMath::Vector3 Wire::localStartPoint() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the local start point of an invalid Wire.");
    return m_topology.edge(0).pointAt(0.0);
}

MyMath::Vector3 Wire::localEndPoint() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the local end point of an invalid Wire.");
    return m_topology.edge(edgeCount() - 1).pointAt(1.0);
}

MyMath::Vector3 Wire::worldStartPoint() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the world start point of an invalid Wire.");
    return Instance_Object::localToWorld().transformPoint(localStartPoint());
}

MyMath::Vector3 Wire::worldEndPoint() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot access the world end point of an invalid Wire.");
    return Instance_Object::localToWorld().transformPoint(localEndPoint());
}

/// 方向操作

Wire Wire::reversed() const
{
    MYVOXEL_ASSERT_MESSAGE(isValid(), "Cannot reverse an invalid Wire.");

    if (!isValid())
    {
        return Wire();
    }

    return Wire(m_topology.reversed(), Instance_Object::localToWorld());
}

/// 初始化

void Wire::initialize()
{
    m_localBounds.clear();
    m_worldBounds.clear();

    MYVOXEL_ASSERT_MESSAGE(m_topology.isValid(), "Wire topology must be valid.");

    if (!m_topology.isValid() || !isPlacementValid())
    {
        return;
    }

    for (std::size_t index = 0; index < m_topology.edgeCount(); ++index)
    {
        const Topology_Edge edgeValue = m_topology.edge(index);
        const Bounds3& geometryBounds = edgeValue.geometry().bounds();

        MYVOXEL_ASSERT_MESSAGE(geometryBounds.isValid(), "Wire Edge geometry bounds must be valid.");

        if (!geometryBounds.isValid())
        {
            m_localBounds.clear();
            m_worldBounds.clear();
            return;
        }

        m_localBounds.include(geometryBounds);
        m_worldBounds.include(geometryBounds.transformed(Instance_Object::localToWorld()));
    }

    MYVOXEL_ASSERT_MESSAGE(m_localBounds.isValid() && m_worldBounds.isValid(), "Wire bounds must be valid after initialization.");

    if (!m_localBounds.isValid() || !m_worldBounds.isValid())
    {
        m_localBounds.clear();
        m_worldBounds.clear();
    }
}

}