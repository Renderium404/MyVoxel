#include "Topology_TEdge.h"

#include <limits>

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

// 判断数值是否为有限非负数。
bool isFiniteNonNegative(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value && value != infinity && value != -infinity && value >= 0.0;
}

}

namespace MyVoxel
{

Topology_TEdge::Topology_TEdge(const Topology_Vertex& startVertex, const Topology_Vertex& endVertex, const Foundation::RefPtr<const Geometry_Curve>& geometry, double connectionTolerance)
    : m_startVertex(startVertex)
    , m_endVertex(endVertex)
    , m_geometry(geometry)
{
    MYVOXEL_ASSERT_MESSAGE(startVertex.isValid(), "Topology_TEdge start vertex must be valid.");
    MYVOXEL_ASSERT_MESSAGE(endVertex.isValid(), "Topology_TEdge end vertex must be valid.");
    MYVOXEL_ASSERT_MESSAGE(geometry, "Topology_TEdge geometry must be non-null.");
    MYVOXEL_ASSERT_MESSAGE(isFiniteNonNegative(connectionTolerance), "Topology_TEdge connection tolerance must be finite and non-negative.");

    if (startVertex.isValid() && endVertex.isValid() && geometry && isFiniteNonNegative(connectionTolerance))
    {
        MYVOXEL_ASSERT_MESSAGE(startVertex.point().isEqualTo(geometry->startPoint(), connectionTolerance), "Topology_TEdge start vertex must match the geometry start point.");
        MYVOXEL_ASSERT_MESSAGE(endVertex.point().isEqualTo(geometry->endPoint(), connectionTolerance), "Topology_TEdge end vertex must match the geometry end point.");
    }
}

/// 拓扑数据

const Topology_Vertex& Topology_TEdge::startVertex() const
{
    return m_startVertex;
}

const Topology_Vertex& Topology_TEdge::endVertex() const
{
    return m_endVertex;
}

/// 几何数据

const Geometry_Curve& Topology_TEdge::geometry() const
{
    return *m_geometry;
}

}