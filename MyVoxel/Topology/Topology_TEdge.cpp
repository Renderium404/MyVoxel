#include "Topology_TEdge.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace
{

// 将有效Topology_Vertex规范为Forward句柄，同时保持Topology_TVertex身份不变。
MyVoxel::Topology_Vertex forwardVertex(const MyVoxel::Topology_Vertex& vertex)
{
    return vertex.isForward() ? vertex : vertex.reversed();
}

}

namespace MyVoxel
{

Topology_TEdge::Topology_TEdge(const Foundation::RefPtr<const Geometry_Curve>& geometry, double firstParameter, double lastParameter,
                               const Topology_Vertex& firstVertex, const Topology_Vertex& lastVertex)
    : m_geometry(geometry)
    , m_firstParameter(firstParameter)
    , m_lastParameter(lastParameter)
    , m_firstVertex(firstVertex.isValid() ? forwardVertex(firstVertex) : Topology_Vertex())
    , m_lastVertex(lastVertex.isValid() ? forwardVertex(lastVertex) : Topology_Vertex())
{
    MYVOXEL_ASSERT_MESSAGE(geometry, "Topology_TEdge requires a non-null Geometry_Curve.");
    MYVOXEL_ASSERT_MESSAGE(firstParameter >= 0.0 && firstParameter < lastParameter && lastParameter <= 1.0,
                           "Topology_TEdge parameters must satisfy 0 <= firstParameter < lastParameter <= 1.");
    MYVOXEL_ASSERT_MESSAGE(firstVertex.isValid() && lastVertex.isValid(), "Topology_TEdge requires valid topology vertices.");
}

Topology_TEdge::~Topology_TEdge()
{
}

/// 几何支撑

const Foundation::RefPtr<const Geometry_Curve>& Topology_TEdge::geometry() const
{
    return m_geometry;
}

double Topology_TEdge::firstParameter() const
{
    return m_firstParameter;
}

double Topology_TEdge::lastParameter() const
{
    return m_lastParameter;
}

/// 拓扑端点

const Topology_Vertex& Topology_TEdge::firstVertex() const
{
    return m_firstVertex;
}

const Topology_Vertex& Topology_TEdge::lastVertex() const
{
    return m_lastVertex;
}

}
