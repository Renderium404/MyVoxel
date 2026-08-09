#include "Topology_TVertex.h"

#include "MyVoxel/Foundation/Diagnostic.h"

namespace MyVoxel
{

Topology_TVertex::Topology_TVertex(const Foundation::RefPtr<const Geometry_Point>& geometry)
    : m_geometry(geometry)
{
    MYVOXEL_ASSERT_MESSAGE(geometry, "Topology_TVertex requires a non-null Geometry_Point.");
}

Topology_TVertex::~Topology_TVertex()
{
}

/// 几何支撑

const Foundation::RefPtr<const Geometry_Point>& Topology_TVertex::geometry() const
{
    return m_geometry;
}

}
