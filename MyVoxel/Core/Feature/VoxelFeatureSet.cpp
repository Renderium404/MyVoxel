#include "VoxelFeatureSet.h"

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

// 判断Edge是否引用指定Topology_Vertex身份。
bool edgeUsesVertex(const MyVoxel::Topology_Edge& edge, const MyVoxel::Topology_Vertex& vertex)
{
    return edge.startVertex().isSame(vertex) || edge.endVertex().isSame(vertex);
}

}

namespace MyVoxel
{

VoxelFeatureSet::VoxelFeatureSet()
{
}

/// 状态与数据

bool VoxelFeatureSet::isValid() const
{
    for (std::size_t vertexIndex = 0; vertexIndex < m_vertices.size(); ++vertexIndex)
    {
        if (!m_vertices[vertexIndex].isValid())
        {
            return false;
        }

        for (std::size_t otherIndex = vertexIndex + 1; otherIndex < m_vertices.size(); ++otherIndex)
        {
            if (m_vertices[vertexIndex].isSame(m_vertices[otherIndex]))
            {
                return false;
            }
        }
    }

    for (std::size_t edgeIndex = 0; edgeIndex < m_edges.size(); ++edgeIndex)
    {
        if (!m_edges[edgeIndex].isValid() || !containsVertex(m_edges[edgeIndex].startVertex()) || !containsVertex(m_edges[edgeIndex].endVertex()))
        {
            return false;
        }

        for (std::size_t otherIndex = edgeIndex + 1; otherIndex < m_edges.size(); ++otherIndex)
        {
            if (m_edges[edgeIndex].isSame(m_edges[otherIndex]))
            {
                return false;
            }
        }
    }

    Bounds3 expectedBounds;

    for (std::size_t index = 0; index < m_vertices.size(); ++index)
    {
        expectedBounds.include(m_vertices[index].point());
    }

    for (std::size_t index = 0; index < m_edges.size(); ++index)
    {
        expectedBounds.include(m_edges[index].geometry().bounds());
    }

    return expectedBounds.isEqualTo(m_bounds, 0.0);
}

bool VoxelFeatureSet::isEmpty() const
{
    return m_vertices.empty() && m_edges.empty();
}

std::size_t VoxelFeatureSet::vertexCount() const
{
    return m_vertices.size();
}

std::size_t VoxelFeatureSet::edgeCount() const
{
    return m_edges.size();
}

const std::vector<Topology_Vertex>& VoxelFeatureSet::vertices() const
{
    return m_vertices;
}

const std::vector<Topology_Edge>& VoxelFeatureSet::edges() const
{
    return m_edges;
}

const Bounds3& VoxelFeatureSet::bounds() const
{
    return m_bounds;
}

/// 身份查询

bool VoxelFeatureSet::containsVertex(const Topology_Vertex& vertex) const
{
    if (!vertex.isValid())
    {
        return false;
    }

    for (std::size_t index = 0; index < m_vertices.size(); ++index)
    {
        if (m_vertices[index].isSame(vertex))
        {
            return true;
        }
    }

    return false;
}

bool VoxelFeatureSet::containsEdge(const Topology_Edge& edge) const
{
    if (!edge.isValid())
    {
        return false;
    }

    for (std::size_t index = 0; index < m_edges.size(); ++index)
    {
        if (m_edges[index].isSame(edge))
        {
            return true;
        }
    }

    return false;
}

std::size_t VoxelFeatureSet::incidentEdgeCount(const Topology_Vertex& vertex) const
{
    if (!vertex.isValid())
    {
        return 0;
    }

    std::size_t count = 0;

    for (std::size_t index = 0; index < m_edges.size(); ++index)
    {
        if (edgeUsesVertex(m_edges[index], vertex))
        {
            ++count;
        }
    }

    return count;
}

void VoxelFeatureSet::incidentEdges(const Topology_Vertex& vertex, std::vector<Topology_Edge>& result) const
{
    result.clear();

    if (!vertex.isValid())
    {
        return;
    }

    for (std::size_t index = 0; index < m_edges.size(); ++index)
    {
        if (edgeUsesVertex(m_edges[index], vertex))
        {
            result.push_back(m_edges[index]);
        }
    }
}

/// 集合修改

bool VoxelFeatureSet::addVertex(const Topology_Vertex& vertex)
{
    MYVOXEL_ASSERT_MESSAGE(vertex.isValid(), "VoxelFeatureSet requires a valid Topology_Vertex.");

    if (!vertex.isValid() || containsVertex(vertex))
    {
        return false;
    }

    m_vertices.push_back(vertex);
    m_bounds.include(vertex.point());
    return true;
}

bool VoxelFeatureSet::addEdge(const Topology_Edge& edge)
{
    MYVOXEL_ASSERT_MESSAGE(edge.isValid(), "VoxelFeatureSet requires a valid Topology_Edge.");

    if (!edge.isValid() || containsEdge(edge))
    {
        return false;
    }

    addVertex(edge.startVertex());
    addVertex(edge.endVertex());
    m_edges.push_back(edge);
    m_bounds.include(edge.geometry().bounds());
    return true;
}

bool VoxelFeatureSet::removeEdge(const Topology_Edge& edge)
{
    if (!edge.isValid())
    {
        return false;
    }

    for (std::vector<Topology_Edge>::iterator iterator = m_edges.begin(); iterator != m_edges.end(); ++iterator)
    {
        if (iterator->isSame(edge))
        {
            m_edges.erase(iterator);
            rebuildBounds();
            return true;
        }
    }

    return false;
}

bool VoxelFeatureSet::removeVertex(const Topology_Vertex& vertex)
{
    if (!vertex.isValid() || incidentEdgeCount(vertex) != 0)
    {
        return false;
    }

    for (std::vector<Topology_Vertex>::iterator iterator = m_vertices.begin(); iterator != m_vertices.end(); ++iterator)
    {
        if (iterator->isSame(vertex))
        {
            m_vertices.erase(iterator);
            rebuildBounds();
            return true;
        }
    }

    return false;
}

void VoxelFeatureSet::clear()
{
    m_vertices.clear();
    m_edges.clear();
    m_bounds.clear();
}

/// 空间查询

void VoxelFeatureSet::queryVertices(const Bounds3& queryBounds, std::vector<Topology_Vertex>& result, double tolerance) const
{
    MYVOXEL_ASSERT_MESSAGE(queryBounds.isValid(), "VoxelFeatureSet vertex query requires valid bounds.");
    MYVOXEL_ASSERT_MESSAGE(isFiniteNonNegative(tolerance), "VoxelFeatureSet query tolerance must be finite and non-negative.");
    result.clear();

    if (!queryBounds.isValid() || !isFiniteNonNegative(tolerance))
    {
        return;
    }

    for (std::size_t index = 0; index < m_vertices.size(); ++index)
    {
        if (queryBounds.contains(m_vertices[index].point(), tolerance))
        {
            result.push_back(m_vertices[index]);
        }
    }
}

void VoxelFeatureSet::queryEdges(const Bounds3& queryBounds, std::vector<Topology_Edge>& result, double tolerance) const
{
    MYVOXEL_ASSERT_MESSAGE(queryBounds.isValid(), "VoxelFeatureSet edge query requires valid bounds.");
    MYVOXEL_ASSERT_MESSAGE(isFiniteNonNegative(tolerance), "VoxelFeatureSet query tolerance must be finite and non-negative.");
    result.clear();

    if (!queryBounds.isValid() || !isFiniteNonNegative(tolerance))
    {
        return;
    }

    for (std::size_t index = 0; index < m_edges.size(); ++index)
    {
        if (m_edges[index].geometry().bounds().intersects(queryBounds, tolerance))
        {
            result.push_back(m_edges[index]);
        }
    }
}

/// 内部辅助

void VoxelFeatureSet::rebuildBounds()
{
    m_bounds.clear();

    for (std::size_t index = 0; index < m_vertices.size(); ++index)
    {
        m_bounds.include(m_vertices[index].point());
    }

    for (std::size_t index = 0; index < m_edges.size(); ++index)
    {
        m_bounds.include(m_edges[index].geometry().bounds());
    }
}

}