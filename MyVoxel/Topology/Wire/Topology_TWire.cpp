#include "Topology_TWire.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

Topology_TWire::Topology_TWire(const std::vector<Topology_Edge>& edges, bool closed)
    : m_edges(edges)
    , m_closed(closed)
{
    MYVOXEL_ASSERT_MESSAGE(!edges.empty(), "Topology_TWire requires at least one Topology_Edge.");
}

Topology_TWire::~Topology_TWire()
{
}

/// Edge序列

std::size_t Topology_TWire::edgeCount() const
{
    return m_edges.size();
}

const Topology_Edge& Topology_TWire::edge(std::size_t index) const
{
    MYVOXEL_ASSERT_MESSAGE(index < m_edges.size(), "Topology_TWire edge index is out of range.");
    return m_edges[index];
}

const std::vector<Topology_Edge>& Topology_TWire::edges() const
{
    return m_edges;
}

/// 拓扑状态

bool Topology_TWire::isClosed() const
{
    return m_closed;
}

}